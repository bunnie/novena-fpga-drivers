#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <arpa/inet.h>
#include "sd.h"

void get_time(unsigned int *s, unsigned int *ns);

enum FPGAFrequency {
	FPGA_FREQUENCY = 130000000,
};

#define PKT_VERSION_NUMBER 2
#define PKT_HEADER_SIZE (1+4+4+2)

enum PacketType {
	PACKET_UNKNOWN = 0,
	PACKET_ERROR = 1,
	PACKET_NAND_CYCLE = 2,
	PACKET_SD_DATA = 3,
	PACKET_SD_CMD_ARG = 4,
	PACKET_SD_RESPONSE = 5,
	PACKET_SD_CID = 6,
	PACKET_SD_CSD = 7,
	PACKET_BUFFER_OFFSET = 8,
	PACKET_BUFFER_CONTENTS = 9,
	PACKET_COMMAND = 10,
	PACKET_RESET = 11,
	PACKET_BUFFER_DRAIN = 12,
	PACKET_HELLO = 13,
};

int ofd;

void init_packet_log(int filedesc) {
  ofd = filedesc;
}

static int log_write_data(struct sd *sd, char *pkt, size_t len) {
  return write(ofd, pkt, len);
}

/* Generic packet header
 *  Offset | Size | Description
 * --------+------+-------------
 *     0   |  1   | Packet type (as defined in PacketType)
 *     1   |  4   | Seconds since reset
 *     5   |  4   | Nanoseconds since reset
 *     9   |  2   | Header size
 */

int sd_get_elapsed(struct sd *state, time_t *tv_sec, long *tv_nsec) {
  get_time( (unsigned int *) tv_sec, (unsigned int *) tv_nsec );

  return 0;
}

static int pkt_set_header(struct sd *sd, char *pkt, int type, int size) {
	long real_sec, real_nsec;
	uint32_t sec, nsec;
	uint16_t sz;
	pkt[0] = type;
	sd_get_elapsed(sd, &real_sec, &real_nsec); 
	sec = htonl(real_sec); 
	nsec = htonl(real_nsec);
	sz = htons(size);
	memcpy(pkt+1, &sec, sizeof(sec));
	memcpy(pkt+5, &nsec, sizeof(nsec));
	memcpy(pkt+9, &sz, sizeof(sz));
	return 0;
}


/* Generic packet header (for FPGA ticks)
 *  Offset | Size | Description
 * --------+------+-------------
 *     0   |  1   | Packet type (as defined in WPacketType)
 *     1   |  4   | Seconds since reset
 *     5   |  4   | Nanoseconds since reset
 */
static int pkt_set_header_fpga(struct sd *sd, char *pkt, uint32_t fpga_counter, int type, int size) {

	uint32_t sec, nsec;
	uint16_t sz;

	get_time( (unsigned int *) &sec, (unsigned int *) &nsec );

	pkt[0] = type;
	sec = htonl(sec);
	nsec = htonl(nsec);
	sz = htons(size);
	memcpy(pkt+1, &sec, sizeof(sec));
	memcpy(pkt+5, &nsec, sizeof(nsec));
	memcpy(pkt+9, &sz, sizeof(sz));
	return 0;
}

static int pkt_set_header_fpga_fulltime(struct sd *sd, char *pkt, uint32_t nsec, uint32_t sec, int type, int size) {
	uint16_t sz;

	pkt[0] = type;
	sec = htonl(sec);
	nsec = htonl(nsec);
	sz = htons(size);
	memcpy(pkt+1, &sec, sizeof(sec));
	memcpy(pkt+5, &nsec, sizeof(nsec));
	memcpy(pkt+9, &sz, sizeof(sz));
	return 0;
}


/* PKT_ERROR
 *  Offset | Size | Description
 * --------+------+-------------
 *     0   |  11  | Header
 *    11   |   4  | Error code
 *    15   | 512  | Textual error message (NULL-padded)
 */
int pkt_send_error(struct sd *sd, uint32_t code, char *msg) {
	char pkt[PKT_HEADER_SIZE+4+512];
	uint32_t real_code;

	bzero(pkt, sizeof(pkt));

	pkt_set_header(sd, pkt, PACKET_ERROR, sizeof(pkt));
	real_code = htonl(code);
	memcpy(pkt+PKT_HEADER_SIZE+0, &real_code, sizeof(real_code));

	strncpy(pkt+PKT_HEADER_SIZE+4, msg, 512-1);
	return log_write_data(sd, pkt, sizeof(pkt));
}


/*
 * PACKET_NAND_CYCLE format (FPGA):
 *  Offset | Size | Description
 * --------+------+-------------
 *     0   | 11   | Header
 *    11   |  1   | Data/Command pins
 *    12   |  1   | Bits [0..4] are ALE, CLE, WE, RE, and CS (in order)
 *    13   |  2   | Bits [0..9] are the unknown pins
 */
int pkt_send_nand_cycle(struct sd *sd, uint32_t nsec, uint32_t sec, uint8_t data, uint8_t ctrl, uint8_t unk[2]) {
	char pkt[PKT_HEADER_SIZE+1+1+2];
	pkt_set_header_fpga_fulltime(sd, pkt, nsec, sec, PACKET_NAND_CYCLE, sizeof(pkt));
	pkt[PKT_HEADER_SIZE+0] = data;
	pkt[PKT_HEADER_SIZE+1] = ctrl;
	pkt[PKT_HEADER_SIZE+2] = unk[0];
	pkt[PKT_HEADER_SIZE+3] = unk[1];
	return log_write_data(sd, pkt, sizeof(pkt));
}


/*
 * PACKET_SD_DATA format (CPU):
 *  Offset | Size | Description
 * --------+------+-------------
 *     0   | 11   | Header
 *    11   | 512  | One block of data from the card
 */
int pkt_send_sd_data(struct sd *sd, uint8_t *block) {
	char pkt[PKT_HEADER_SIZE+512];
	pkt_set_header(sd, pkt, PACKET_SD_DATA, sizeof(pkt));
	memcpy(pkt+PKT_HEADER_SIZE, block, 512);
	return log_write_data(sd, pkt, sizeof(pkt));
}


/*
 * PACKET_SD_CMD_ARG format (FPGA):
 *  Offset | Size | Description
 * --------+------+-------------
 *     0   |  11  | Header
 *    11   |   1  | Register number (1, 2, 3, or 4), or 0 for the CMD byte
 *    12   |   1  | Value of the register or CMD number
 */

int pkt_send_sd_cmd_arg_fpga(struct sd *sd, uint32_t fpga_counter, uint8_t regnum, uint8_t val) {
	char pkt[PKT_HEADER_SIZE+1+1];
	pkt_set_header_fpga(sd, pkt, fpga_counter, PACKET_SD_CMD_ARG, sizeof(pkt));
	pkt[PKT_HEADER_SIZE+0] = regnum;
	pkt[PKT_HEADER_SIZE+1] = val;
	return log_write_data(sd, pkt, sizeof(pkt));
}


/*
 * PACKET_SD_CMD_ARG format (FPGA):
 *  Offset | Size | Description
 * --------+------+-------------
 *     0   |  11  | Header
 *    11   |   1  | Register number (1, 2, 3, or 4), or 0 for the CMD byte
 *    12   |   1  | Value of the register or CMD number
 */

int pkt_send_sd_cmd_arg(struct sd *sd, uint8_t regnum, uint8_t val) {
	char pkt[PKT_HEADER_SIZE+1+1];
	pkt_set_header(sd, pkt, PACKET_SD_CMD_ARG, sizeof(pkt));
	pkt[PKT_HEADER_SIZE+0] = regnum;
	pkt[PKT_HEADER_SIZE+1] = val;
	return log_write_data(sd, pkt, sizeof(pkt));
}


/*
 * PACKET_SD_RESPONSE format (FPGA):
 *  Offset | Size | Description
 * --------+------+-------------
 *     0   |  11  | Header
 *    11   |   1  | The contents of the first byte that the card answered with
 */
int pkt_send_sd_response_fpga(struct sd *sd, uint32_t fpga_counter, uint8_t byte) {
	char pkt[PKT_HEADER_SIZE+1];
	pkt_set_header_fpga(sd, pkt, fpga_counter, PACKET_SD_RESPONSE, sizeof(pkt));
	pkt[PKT_HEADER_SIZE+0] = byte;
	return log_write_data(sd, pkt, sizeof(pkt));
}


/*
 * PACKET_SD_RESPONSE format (FPGA):
 *  Offset | Size | Description
 * --------+------+-------------
 *     0   |  11  | Header
 *    11   |   1  | The contents of the first byte that the card answered with
 */
int pkt_send_sd_response(struct sd *sd, uint8_t byte) {
	char pkt[PKT_HEADER_SIZE+1];
	pkt_set_header(sd, pkt, PACKET_SD_RESPONSE, sizeof(pkt));
	pkt[PKT_HEADER_SIZE+0] = byte;
	return log_write_data(sd, pkt, sizeof(pkt));
}


/*
 * PACKET_SD_CID format (CPU):
 *  Offset | Size | Description
 * --------+------+-------------
 *     0   |  11  | Header
 *    11   |  16  | Contents of the card's CID
 */
int pkt_send_sd_cid(struct sd *sd, uint8_t cid[16]) {
	char pkt[PKT_HEADER_SIZE+16];
	pkt_set_header(sd, pkt, PACKET_SD_CID, sizeof(pkt));
	memcpy(pkt+PKT_HEADER_SIZE, cid, 16);
	return log_write_data(sd, pkt, sizeof(pkt));
}


/*
 * PACKET_SD_CSD format (CPU):
 *  Offset | Size | Description
 * --------+------+-------------
 *     0   |  11  | Header
 *    11   |  16  | Contents of the card's CSD
 */
int pkt_send_sd_csd(struct sd *sd, uint8_t csd[16]) {
	char pkt[PKT_HEADER_SIZE+16];
	pkt_set_header(sd, pkt, PACKET_SD_CSD, sizeof(pkt));
	memcpy(pkt+PKT_HEADER_SIZE, csd, 16);
	return log_write_data(sd, pkt, sizeof(pkt));
}


/*
 * PACKET_BUFFER_OFFSET format (CPU):
 *  Offset | Size | Description
 * --------+------+-------------
 *     0   |  11  | Header
 *    11   |   1  | 1 if this is the read buffer, 2 if it's write
 *    12   |   4  | Offset of the current buffer pointer
 */
int pkt_send_buffer_offset(struct sd *sd, uint8_t buffertype, uint32_t offset) {
	char pkt[PKT_HEADER_SIZE+1+4];
	uint32_t real_offset;
	pkt_set_header(sd, pkt, PACKET_BUFFER_OFFSET, sizeof(pkt));
	real_offset = htonl(offset);
	pkt[PKT_HEADER_SIZE+0] = buffertype;
	memcpy(pkt+PKT_HEADER_SIZE+1, &real_offset, sizeof(real_offset));
	return log_write_data(sd, pkt, sizeof(pkt));
}


/*
 * PACKET_BUFFER_CONTENTS format (CPU):
 *  Offset | Size | Description
 * --------+------+-------------
 *     0   |  11  | Header
 *    11   |  1   | 1 if this is the read buffer, 2 if it's write
 *    12   | 512  | Contents of the buffer
 */
int pkt_send_buffer_contents(struct sd *sd, uint8_t buffertype, uint8_t *buffer) {
	char pkt[PKT_HEADER_SIZE+1+512];
	pkt_set_header(sd, pkt, PACKET_BUFFER_CONTENTS, sizeof(pkt));
	pkt[PKT_HEADER_SIZE+0] = buffertype;
	memcpy(pkt+PKT_HEADER_SIZE+1, buffer, 512);
	return log_write_data(sd, pkt, sizeof(pkt));
}


/*
 * PACKET_COMMAND format (CPU):
 *  Offset | Size | Description
 * --------+------+-------------
 *     0   |  11  | Header
 *    11   |   2  | Two-character command code
 *    13   |   4  | 32-bit command argument
 *    14   |   1  | 1 if the command is starting, 2 if it's ending
 */
int pkt_send_command(struct sd *sd, struct sd_cmd *cmd, uint8_t start_stop) {
	char pkt[PKT_HEADER_SIZE+2+4+1];
	uint32_t arg;
	arg = htonl(cmd->arg);
	pkt_set_header(sd, pkt, PACKET_COMMAND, sizeof(pkt));
	pkt[PKT_HEADER_SIZE+0] = cmd->cmd[0];
	pkt[PKT_HEADER_SIZE+1] = cmd->cmd[1];
	memcpy(pkt+PKT_HEADER_SIZE+2, &arg, sizeof(arg));
	pkt[PKT_HEADER_SIZE+2+4] = start_stop;
	return log_write_data(sd, pkt, sizeof(pkt));
}


/*
 * PACKET_RESET format (CPU):
 *  Offset | Size | Description
 * --------+------+-------------
 *     0   |  11  | Header
 *    11   |   1  | Command stream version number
 */
int pkt_send_reset(struct sd *sd) {
	char pkt[PKT_HEADER_SIZE+1];
	pkt_set_header(sd, pkt, PACKET_RESET, sizeof(pkt));
	pkt[PKT_HEADER_SIZE+0] = PKT_VERSION_NUMBER;
	return log_write_data(sd, pkt, sizeof(pkt));
}


/*
 * PACKET_BUFFER_DRAIN format (CPU):
 *  Offset | Size | Description
 * --------+------+-------------
 *     0   |  11  | Header
 *    11   |   1  | 1 if it's a buffer drain start, 2 if it's an end
 */
int pkt_send_buffer_drain(struct sd *sd, uint8_t start_stop) {
	char pkt[PKT_HEADER_SIZE+1];
	pkt_set_header(sd, pkt, PACKET_BUFFER_DRAIN, sizeof(pkt));
	pkt[PKT_HEADER_SIZE+0] = start_stop;
	return log_write_data(sd, pkt, sizeof(pkt));
}


/*
 * PACKET_HELLO format (CPU):
 *  Offset | Size | Description
 * --------+------+-------------
 *     0   |  11  | Header
 *    11   |   1  | Command stream version number
 */
int pkt_send_hello(struct sd *sd) {
	char pkt[PKT_HEADER_SIZE+1];
	pkt_set_header(sd, pkt, PACKET_HELLO, sizeof(pkt));
	pkt[PKT_HEADER_SIZE+0] = PKT_VERSION_NUMBER;
	return log_write_data(sd, pkt, sizeof(pkt));
}
