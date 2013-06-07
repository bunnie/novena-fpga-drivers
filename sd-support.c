#include <stdio.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <strings.h>
#include <string.h>
#include <time.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include "sd.h"
#include "gpio.h"
#include "crc-16.h"

/* CHUMBY_BEND - 89
 * IR_DETECT - 102
 */

/* Pin connection:
 * SD  | MX233
 * 9   | 0
 * 1   | 1
 * 2   | 2
 * 3   | GND
 * DET | 3
 * 4   | [power switch]
 * 5   | 4
 * 6   | GND
 * 7   | 7
 * 8   | NC (was: 6)
 */

/** Definitions for Kovan test jig */
/*
#define CS_PIN 50
#define MISO_PIN 62
#define CLK_PIN 46
#define MOSI_PIN 48
#define POWER_PIN 55
*/

/** Definitions for Novena EIM interface */
#define CS_PIN    GPIO_IS_EIM | 3
#define MISO_PIN  GPIO_IS_EIM | 0
#define CLK_PIN   GPIO_IS_EIM | 4
#define MOSI_PIN  GPIO_IS_EIM | 5
#define POWER_PIN 17 //GPIO1_IO17


// R1 Response Codes (from SD Card Product Manual v1.9 section 5.2.3.1)
#define R1_IN_IDLE_STATE    (1<<0)  // The card is in idle state and running initializing process.
#define R1_ERASE_RESET      (1<<1)  // An erase sequence was cleared before executing because of an out of erase sequence command was received.
#define R1_ILLEGAL_COMMAND  (1<<2)  // An illegal command code was detected
#define R1_COM_CRC_ERROR    (1<<3)  // The CRC check of the last command failed.
#define R1_ERASE_SEQ_ERROR  (1<<4)  // An error in the sequence of erase commands occured.
#define R1_ADDRESS_ERROR    (1<<5)  // A misaligned address, which did not match the block length was used in the command.
#define R1_PARAMETER        (1<<6)  // The command's argument (e.g. address, block length) was out of the allowed range for this card.



static int print_header(uint8_t *bfr) {
	printf(" CMD %2d {%02x %02x %02x %02x %02x %02x}  ", bfr[0]&0x3f, bfr[0], bfr[1], bfr[2], bfr[3], bfr[4], bfr[5]);
	return 0;
}


int populateCmd4(uint8_t *bfr, int a2, char one, char zero) {
	bfr[0] = 0x40 | 4;
	if ( a2 < 0 ) {
LABEL_4:
		if ( a2 < 100 )
			goto LABEL_7;
		goto LABEL_5;
	}
	if ( a2 < 100 ) {
		bfr[1] = a2 / 10;
		bfr[2] = a2 % 10;
		goto LABEL_4;
	}
LABEL_5:
	if ( a2 <= 110 ) {
		bfr[1] = 15;
		bfr[2] = a2 - 100;
	}
LABEL_7:
	if ( a2 == -1 ) {
		bfr[1] = 17;
		bfr[2] = 17;
	}
	if ( a2 == -2 ) {
		bfr[1] = 16;
		bfr[2] = 16;
	}
	bfr[3] = one;
	bfr[4] = one;

	bfr[5] = 0;

	return 0;
}

int sendTestBoot(struct sd_state *state, int buffersize, void *buffer) {
	uint32_t offset_bs = htonl(buffersize);
	uint8_t bfr[6];

	bfr[0] = 0x40 | 5;

	bfr[1] = offset_bs>>24;
	bfr[2] = offset_bs>>16;
	bfr[3]  = offset_bs>>8;
	bfr[4]  = offset_bs>>0;

	bfr[5] = 0;

	return sd_write_file(state, bfr, sizeof(bfr), buffer, buffersize);
}

static int openAndSendTestboot(struct sd_state *state, char *filename) {
	int fd = open(filename, O_RDONLY);
	if (fd == -1) {
		perror("Unable to open file");
		return -1;
	}

	int filesize = lseek(fd, 0, SEEK_END);
	if (-1 == lseek(fd, 0, SEEK_SET)) {
		perror("Unable to seek file");
		close(fd);
		return -1;
	}

	if (filesize & 0x1ff) {
		printf("Correcting %d to %d bytes\n", filesize, (filesize+0x1ff)&0x1ff);
		filesize = (filesize+0x1ff)&0x1ff;
	}
	uint8_t bfr[filesize];
	if (-1 == read(fd, bfr, sizeof(bfr))) {
		perror("Unable to read file");
		close(fd);
		return -1;
	}

	close(fd);

	return sendTestBoot(state, filesize, bfr);
}

int sendFile(struct sd_state *state, int offset, int buffersize, void *buffer) {
	uint8_t bfr[6];

	bfr[0] = 0x40 | 59;

	bfr[1] = 0;
	bfr[2] = 0;
	bfr[3]  = 0;
	bfr[4]  = 0;

	bfr[5] = (crc7(bfr, 5)<<1)|1;

	print_header(bfr);
	printf(" Buffersize: %d\n", buffersize);
	return sd_write_file(state, bfr, sizeof(bfr), buffer, buffersize);
}


static int openAndSendFile(struct sd_state *state, int offset, char *filename) {
	int fd = open(filename, O_RDONLY);
	if (fd == -1) {
		perror("Unable to open file");
		return -1;
	}

	int filesize = lseek(fd, 0, SEEK_END);
	if (-1 == lseek(fd, 0, SEEK_SET)) {
		perror("Unable to seek file");
		close(fd);
		return -1;
	}

	if (filesize & 0x1ff) {
		printf("Correcting %d to %d bytes\n", filesize, (filesize+0x1ff)&0x1ff);
		filesize = (filesize+0x1ff)&0x1ff;
	}
	uint8_t bfr[filesize];
	if (-1 == read(fd, bfr, sizeof(bfr))) {
		perror("Unable to read file");
		close(fd);
		return -1;
	}

	close(fd);

	return sendFile(state, offset, filesize, bfr);
}




int send_cmdX(struct sd_state *state, 
		uint8_t cmd,
		uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4,
		int print_size) {
	uint8_t bfr[6];
	uint8_t out_bfr[print_size];
	int result;
	static int run = 0;
	int i;

	memset(out_bfr, 0, sizeof(out_bfr));
	cmd = (cmd&0x3f)|0x40;
	bfr[0] = cmd;
	bfr[1] = a1;
	bfr[2] = a2;
	bfr[3] = a3;
	bfr[4] = a4;
	bfr[5] = (crc7(bfr, 5)<<1)|1;

	for (i=0; i<sizeof(bfr); i++)
	  pkt_send_sd_cmd_arg(state, i, bfr[i]);

	result = sd_dump_rom(state, bfr, sizeof(bfr), out_bfr, print_size);

	if (result!=-1 && !(result&R1_ILLEGAL_COMMAND)) {
		out_bfr[0] = result;
		printf("Run %-4d  ", run);
		print_header(bfr);
		print_hex(out_bfr, print_size);
	}
	run++;
	return result;
}

int blind_reset(struct sd_state *state, uint8_t *file, uint8_t *response, int bfr_size) {
	uint8_t bfr[6];
	
	bfr[0] = 63|0x40;
	bfr[1] = 'A';
	bfr[2] = 'P';
	bfr[3] = 'P';
	bfr[4] = 'O';
	bfr[5] = (crc7(bfr, 5)<<1)|1;

	return sd_blind_xfer(state, bfr, sizeof(bfr), file, response, bfr_size);
}


int do_get_csd_cid(struct sd_state *state) {
	sd_reset(state, 2);

	printf("CSD:\n");
	send_cmdX(state, 9, 0, 0, 0, 0, 32);
	printf("\n");

	printf("CID:\n");
	send_cmdX(state, 10, 0, 0, 0, 0, 32);
	printf("\n");
	return 0;
}

