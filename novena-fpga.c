//#define _GNU_SOURCE // for O_DIRECT

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
//#include <sys/types.h>
//#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include "gpio.h"
#include "sd.h"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

/** Definitions for Novena EIM interface */
#define CS_PIN    GPIO_IS_EIM | 3
#define MISO_PIN  GPIO_IS_EIM | 0
#define CLK_PIN   GPIO_IS_EIM | 4
#define MOSI_PIN  GPIO_IS_EIM | 5
#define POWER_PIN 17 //GPIO1_IO17

struct reg_info {
  char *name;
  int offset;
  int size;
  char *description;
};

extern int pkt_send_hello(struct sd *sd);
extern int pkt_send_reset(struct sd *sd);
extern int pkt_send_nand_cycle(struct sd *sd, uint32_t nsec, uint32_t sec, uint8_t data, uint8_t ctrl, uint8_t unk[2]);
extern int send_cmdX(struct sd_state *state, 
		     uint8_t cmd,
		     uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4,
		     int print_size);
extern void init_packet_log(int filedesc);

static int fd = 0;
static int   *mem_32 = 0;
static short *mem_16 = 0;
static char  *mem_8  = 0;
static int *prev_mem_range = 0;

#define FPGA_REG_OFFSET    0x08040000
#define FPGA_CS1_REG_OFFSET    0x0C040000

#define FPGA_MAP(x)         ( (x - FPGA_REG_OFFSET) >> 1 )
#define F(x)                ( (x - FPGA_REG_OFFSET) >> 1 )
#define F1(x)                ( (x - FPGA_CS1_REG_OFFSET) >> 3 )

#define FPGA_W_TEST0       0x08040000
#define FPGA_W_TEST1       0x08040002
#define FPGA_W_GPIOA_DOUT  0x08040010
#define FPGA_W_GPIOA_DIR   0x08040012
#define FPGA_W_GPIOB_DOUT  0x08040014
#define FPGA_W_GPIOB_DIR   0x08040016

#define FPGA_W_DDR3_P2_CMD  0x08040020
#define FPGA_W_DDR3_P2_LADR 0x08040022
#define FPGA_W_DDR3_P2_HADR 0x08040024
#define FPGA_W_DDR3_P2_WEN  0x08040026
#define FPGA_W_DDR3_P2_LDAT 0x08040028
#define FPGA_W_DDR3_P2_HDAT 0x0804002A

#define FPGA_W_DDR3_P3_CMD  0x08040030
#define FPGA_W_DDR3_P3_LADR 0x08040032
#define FPGA_W_DDR3_P3_HADR 0x08040034
#define FPGA_W_DDR3_P3_REN  0x08040036

#define FPGA_W_NAND_UK_CTL  0x08040100
#define FPGA_W_NAND_PWR_CTL 0x08040102

#define FPGA_W_LOG_CMD      0x08040200
#define FPGA_W_LOG_CONFIG   0x08040202
#define FPGA_W_MCUDRV       0x08040210

#define FPGA_R_TEST0        0x08041000
#define FPGA_R_TEST1        0x08041002
#define FPGA_R_DDR3_CAL     0x08041004
#define FPGA_R_GPIOA_DIN    0x08041010
#define FPGA_R_GPIOB_DIN    0x08041012

#define FPGA_R_DDR3_P2_STAT 0x08041020
#define FPGA_R_DDR3_P3_STAT 0x08041030
#define FPGA_R_DDR3_P3_LDAT 0x08041032
#define FPGA_R_DDR3_P3_HDAT 0x08041034

#define FPGA_R_NAND_UK_DATA 0x08041100 // every time I read it auto-advances the queue
#define FPGA_R_NAND_UK_STAT 0x08041102

#define FPGA_R_NAND_CMD_DATA 0x08041104 // every time I read it auto-advances the queue
#define FPGA_R_NAND_CMD_STAT 0x08041106

#define FPGA_R_NAND_ADR_STAT 0x08041108
#define FPGA_R_NAND_ADR_LOW  0x0804110A
#define FPGA_R_NAND_ADR_HI   0x0804110C // this auto-advances the queue

#define FPGA_R_NAND_DDR_STAT 0x0804110E

#define FPGA_R_LOG_STAT     0x08041200
#define FPGA_R_LOG_ENTRY_L  0x08041202
#define FPGA_R_LOG_ENTRY_H  0x08041204
#define FPGA_R_LOG_TIME_NSL 0x08041206 // this auto-loads time_t
#define FPGA_R_LOG_TIME_NSH 0x08041208
#define FPGA_R_LOG_TIME_SL  0x0804120A
#define FPGA_R_LOG_TIME_SH  0x0804120C
#define FPGA_R_LOG_SAN_FLSH 0x08041210
#define FPGA_R_LOG_SAN_UK   0x08041212
#define FPGA_R_LOG_DEBUG    0x08041220

#define FPGA_R_DDR3_V_MINOR 0x08041FFC
#define FPGA_R_DDR3_V_MAJOR 0x08041FFE

// burst access registers (in CS1 bank -- only 64-bit access allowed)
#define FPGA_WB_LOOP0       0x0C040000
#define FPGA_WB_LOOP1       0x0C040008

#define FPGA_WB_DDR3_RD_CTL 0x0C040100

#define FPGA_RB_LOOP0       0x0C041000
#define FPGA_RB_LOOP1       0x0C041008

#define FPGA_RB_DDR3_RD_DATA 0x0C041100
#define FPGA_RB_DDR3_RD_STAT 0x0C041108



#define B_WE  ((unsigned short) 0x100)
#define B_RE  ((unsigned short) 0x200)
#define B_ALE ((unsigned short) 0x400)
#define B_CLE ((unsigned short) 0x800)
#define B_CS  ((unsigned short) 0x1000)

#define CMD_ID  0x90
#define CMD_RESET 0xFF
#define CMD_PARAM 0x55
#define CMD_STATUS 0x70
#define CMD_CHARGE2 0x60
#define CMD_CHARGE1 0x65
#define CMD_COLSET  0x05
#define CMD_READ    0x00
#define CMD_CACHE1  0x30
#define CMD_CACHE2  0xA2
#define CMD_CACHE3  0x69
#define CMD_CACHE4  0xFD
#define CMD_VSTART  0x5C

int read_kernel_memory(long offset, int virtualized, int size) {
  int result;

  int *mem_range = (int *)(offset & ~0xFFFF);
  if( mem_range != prev_mem_range ) {
    //        fprintf(stderr, "New range detected.  Reopening at memory range %p\n", mem_range);
    prev_mem_range = mem_range;

    if(mem_32)
      munmap(mem_32, 0xFFFF);
    if(fd)
      close(fd);

    if(virtualized) {
      fd = open("/dev/kmem", O_RDWR);
      if( fd < 0 ) {
	perror("Unable to open /dev/kmem");
	fd = 0;
	return -1;
      }
    }
    else {
      fd = open("/dev/mem", O_RDWR);
      if( fd < 0 ) {
	perror("Unable to open /dev/mem");
	fd = 0;
	return -1;
      }
    }

    mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset&~0xFFFF);
    if( -1 == (int)mem_32 ) {
      perror("Unable to mmap file");

      if( -1 == close(fd) )
	perror("Also couldn't close file");

      fd=0;
      return -1;
    }
    mem_16 = (short *)mem_32;
    mem_8  = (char  *)mem_32;
  }

  int scaled_offset = (offset-(offset&~0xFFFF));
  //    fprintf(stderr, "Returning offset 0x%08x\n", scaled_offset);
  if(size==1)
    result = mem_8[scaled_offset/sizeof(char)];
  else if(size==2)
    result = mem_16[scaled_offset/sizeof(short)];
  else
    result = mem_32[scaled_offset/sizeof(long)];

  return result;
}

#define TEST_LEN 32768

void test_fpga(void) {
  volatile unsigned short *cs0;
  int i;
  unsigned short test[TEST_LEN];
  unsigned short tval;
  unsigned int iters = 0, errs = 0;

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08000000);
  cs0 = (volatile unsigned short *)mem_32;

  i = 0;
  while(1) {
    for( i = 0; i < TEST_LEN; i ++ ) {
      test[i] = (unsigned short) rand();
#if 0
      if( !(i % 16) )
	printf( "\n" );
      printf( "%04x ", test[i] );
#endif
    }

    for( i = 0; i < TEST_LEN; i ++ ) {
      cs0[i] = test[i];
    }
#if 0
    printf( "\n. . . . . . . . .\n" );
#endif
    
    for( i = 0; i < TEST_LEN; i ++ ) {
      iters++;
#if 0
      if( !(i % 16) )
	printf( "\n" );
      printf( "%04x ", cs0[i] );
#else
      tval = cs0[i];
      if( test[i] != tval ) {
	printf( "\nFail at %d: wrote %04x, got %04x/%04x\n", i, test[i], tval, cs0[i] );
	errs++;
      } else {
#if 0
	if( !(i % 16) )
	  printf( "\n" );
	printf( "%04x ", test[i] );
#endif
      }
#endif
    }
#if 0
    printf( "\n---------------\n" );
#endif
    if( !(iters % 0x100000) ) {
      printf( "%d iters, %d errs\n", iters, errs );
      fflush(stdout);
    }
  }
    
}

int write_kernel_memory(long offset, long value, int virtualized, int size) {
  int old_value = read_kernel_memory(offset, virtualized, size);
  int scaled_offset = (offset-(offset&~0xFFFF));
  if(size==1)
    mem_8[scaled_offset/sizeof(char)]   = value;
  else if(size==2)
    mem_16[scaled_offset/sizeof(short)] = value;
  else
    mem_32[scaled_offset/sizeof(long)]  = value;
  return old_value;
}

void print_usage(char *progname) {
  printf("Usage:\n"
        "%s [-h]\n"
        "\t-h  This help message\n"
        "\t-s  Set up FPGA comms parameter\n"
	"\t-t  Test FPGA memory interface\n"
	 "", progname);
}


//static inline int swab(int arg) {
//  return ((arg&0xff)<<24) | ((arg&0xff00)<<8) | ((arg&0xff0000)>>8) | ((arg&0xff000000)>>24);
//}

void setup_fpga() {
  int i;
  printf( "setting up EIM CS0 (register interface) pads and configuring timing\n" );
  // set up pads to be mapped to EIM
  for( i = 0; i < 16; i++ ) {
    write_kernel_memory( 0x20e0114 + i*4, 0x0, 0, 4 );  // mux mapping
    write_kernel_memory( 0x20e0428 + i*4, 0xb0b1, 0, 4 ); // pad strength config'd for a 100MHz rate 
  }

  // mux mapping
  write_kernel_memory( 0x20e046c - 0x314, 0x0, 0, 4 ); // BCLK
  write_kernel_memory( 0x20e040c - 0x314, 0x0, 0, 4 ); // CS0
  write_kernel_memory( 0x20e0410 - 0x314, 0x0, 0, 4 ); // CS1
  write_kernel_memory( 0x20e0414 - 0x314, 0x0, 0, 4 ); // OE
  write_kernel_memory( 0x20e0418 - 0x314, 0x0, 0, 4 ); // RW
  write_kernel_memory( 0x20e041c - 0x314, 0x0, 0, 4 ); // LBA
  write_kernel_memory( 0x20e0468 - 0x314, 0x0, 0, 4 ); // WAIT
  write_kernel_memory( 0x20e0408 - 0x314, 0x0, 0, 4 ); // A16
  write_kernel_memory( 0x20e0404 - 0x314, 0x0, 0, 4 ); // A17
  write_kernel_memory( 0x20e0400 - 0x314, 0x0, 0, 4 ); // A18

  // pad strength
  write_kernel_memory( 0x20e046c, 0xb0b1, 0, 4 ); // BCLK
  write_kernel_memory( 0x20e040c, 0xb0b1, 0, 4 ); // CS0
  write_kernel_memory( 0x20e0410, 0xb0b1, 0, 4 ); // CS1
  write_kernel_memory( 0x20e0414, 0xb0b1, 0, 4 ); // OE
  write_kernel_memory( 0x20e0418, 0xb0b1, 0, 4 ); // RW
  write_kernel_memory( 0x20e041c, 0xb0b1, 0, 4 ); // LBA
  write_kernel_memory( 0x20e0468, 0xb0b1, 0, 4 ); // WAIT
  write_kernel_memory( 0x20e0408, 0xb0b1, 0, 4 ); // A16
  write_kernel_memory( 0x20e0404, 0xb0b1, 0, 4 ); // A17
  write_kernel_memory( 0x20e0400, 0xb0b1, 0, 4 ); // A18

  write_kernel_memory( 0x020c4080, 0xcf3, 0, 4 ); // ungate eim slow clocks

  // rework timing for sync use
  // 0011 0  001 1   001    0   001 00  00  1  011  1    0   1   1   1   1   1   1
  // PSZ  WP GBC AUS CSREC  SP  DSZ BCS BCD WC BL   CREP CRE RFL WFL MUM SRD SWR CSEN
  //
  // PSZ = 0011  64 words page size
  // WP = 0      (not protected)
  // GBC = 001   min 1 cycles between chip select changes
  // AUS = 0     address shifted according to port size
  // CSREC = 001 min 1 cycles between CS, OE, WE signals
  // SP = 0      no supervisor protect (user mode access allowed)
  // DSZ = 001   16-bit port resides on DATA[15:0]
  // BCS = 00    0 clock delay for burst generation
  // BCD = 00    divide EIM clock by 0 for burst clock
  // WC = 1      write accesses are continuous burst length
  // BL = 011    32 word memory wrap length
  // CREP = 1    non-PSRAM, set to 1
  // CRE = 0     CRE is disabled
  // RFL = 1     fixed latency reads
  // WFL = 1     fixed latency writes
  // MUM = 1     multiplexed mode enabled
  // SRD = 1     synch reads
  // SWR = 1     synch writes
  // CSEN = 1    chip select is enabled

  //  write_kernel_memory( 0x21b8000, 0x5191C0B9, 0, 4 );
  write_kernel_memory( 0x21b8000, 0x31910BBF, 0, 4 );

  // EIM_CS0GCR2   
  //  MUX16_BYP_GRANT = 1
  //  ADH = 1 (1 cycles)
  //  0x1001
  write_kernel_memory( 0x21b8004, 0x1000, 0, 4 );


  // EIM_CS0RCR1   
  // 00 000101 0 000   0   000   0 000 0 000 0 000 0 000
  //    RWSC     RADVA RAL RADVN   OEA   OEN   RCSA  RCSN
  // RWSC 000101    5 cycles for reads to happen
  //
  // 0000 0111 0000   0011   0000 0000 0000 0000
  //  0    7     0     3      0  0    0    0
  // 0000 0101 0000   0000   0 000 0 000 0 000 0 000
//  write_kernel_memory( 0x21b8008, 0x05000000, 0, 4 );
//  write_kernel_memory( 0x21b8008, 0x0A024000, 0, 4 );
  write_kernel_memory( 0x21b8008, 0x09014000, 0, 4 );
  // EIM_CS0RCR2  
  // 0000 0000 0   000 00 00 0 010  0 001 
  //           APR PAT    RL   RBEA   RBEN
  // APR = 0   mandatory because MUM = 1
  // PAT = XXX because APR = 0
  // RL = 00   because async mode
  // RBEA = 000  these match RCSA/RCSN from previous field
  // RBEN = 000
  // 0000 0000 0000 0000 0000  0000
  write_kernel_memory( 0x21b800c, 0x00000000, 0, 4 );

  // EIM_CS0WCR1
  // 0   0    000100 000   000   000  000  010 000 000  000
  // WAL WBED WWSC   WADVA WADVN WBEA WBEN WEA WEN WCSA WCSN
  // WAL = 0       use WADVN
  // WBED = 0      allow BE during write
  // WWSC = 000100 4 write wait states
  // WADVA = 000   same as RADVA
  // WADVN = 000   this sets WE length to 1 (this value +1)
  // WBEA = 000    same as RBEA
  // WBEN = 000    same as RBEN
  // WEA = 010     2 cycles between beginning of access and WE assertion
  // WEN = 000     1 cycles to end of WE assertion
  // WCSA = 000    cycles to CS assertion
  // WCSN = 000    cycles to CS negation
  // 1000 0111 1110 0001 0001  0100 0101 0001
  // 8     7    E    1    1     4    5    1
  // 0000 0111 0000 0100 0000  1000 0000 0000
  // 0      7    0   4    0     8    0     0
  // 0000 0100 0000 0000 0000  0100 0000 0000
  //  0    4    0    0     0    4     0    0

  write_kernel_memory( 0x21b8010, 0x09080800, 0, 4 );
  //  write_kernel_memory( 0x21b8010, 0x02040400, 0, 4 );

  // EIM_WCR
  // BCM = 1   free-run BCLK
  // GBCD = 0  don't divide the burst clock
  write_kernel_memory( 0x21b8090, 0x701, 0, 4 );

  // EIM_WIAR 
  // ACLK_EN = 1
  write_kernel_memory( 0x21b8094, 0x10, 0, 4 );

  printf( "done.\n" );
}

void setup_fpga_cs1() { 
  int i;
  printf( "setting up EIM CS1 (burst interface) pads and configuring timing\n" );
  // ASSUME: setup_fpga() is already called to configure gpio mux setting.
  // this just gets the pads set to high-speed mode

  // set up pads to be mapped to EIM
  for( i = 0; i < 16; i++ ) {
    write_kernel_memory( 0x20e0428 + i*4, 0xb0f1, 0, 4 ); // pad strength config'd for a 200MHz rate 
  }

  // pad strength
  write_kernel_memory( 0x20e046c, 0xb0f1, 0, 4 ); // BCLK
  //  write_kernel_memory( 0x20e040c, 0xb0b1, 0, 4 ); // CS0
  write_kernel_memory( 0x20e0410, 0xb0f1, 0, 4 ); // CS1
  write_kernel_memory( 0x20e0414, 0xb0f1, 0, 4 ); // OE
  write_kernel_memory( 0x20e0418, 0xb0f1, 0, 4 ); // RW
  write_kernel_memory( 0x20e041c, 0xb0f1, 0, 4 ); // LBA
  write_kernel_memory( 0x20e0468, 0xb0f1, 0, 4 ); // WAIT
  write_kernel_memory( 0x20e0408, 0xb0f1, 0, 4 ); // A16
  write_kernel_memory( 0x20e0404, 0xb0f1, 0, 4 ); // A17
  write_kernel_memory( 0x20e0400, 0xb0f1, 0, 4 ); // A18

  // EIM_CS1GCR1   
  // 0011 0  001 1   001    0   001 00  00  1  011  1    0   1   1   1   1   1   1
  // PSZ  WP GBC AUS CSREC  SP  DSZ BCS BCD WC BL   CREP CRE RFL WFL MUM SRD SWR CSEN
  //
  // PSZ = 0011  64 words page size
  // WP = 0      (not protected)
  // GBC = 001   min 1 cycles between chip select changes
  // AUS = 0     address shifted according to port size
  // CSREC = 001 min 1 cycles between CS, OE, WE signals
  // SP = 0      no supervisor protect (user mode access allowed)
  // DSZ = 001   16-bit port resides on DATA[15:0]
  // BCS = 00    0 clock delay for burst generation
  // BCD = 00    divide EIM clock by 0 for burst clock
  // WC = 1      write accesses are continuous burst length
  // BL = 011    32 word memory wrap length
  // CREP = 1    non-PSRAM, set to 1
  // CRE = 0     CRE is disabled
  // RFL = 1     fixed latency reads
  // WFL = 1     fixed latency writes
  // MUM = 1     multiplexed mode enabled
  // SRD = 1     synch reads
  // SWR = 1     synch writes
  // CSEN = 1    chip select is enabled

  // 0101 0111 1111    0001 1100  0000  1011   1   0   0   1
  // 0x5  7    F        1   C     0     B    9

  // 0101 0001 1001    0001 1100  0000  1011   1001
  // 5     1    9       1    c     0     B      9

  // 0011 0001 1001    0001 0000  1011  1011   1111

  write_kernel_memory( 0x21b8000 + 0x18, 0x31910BBF, 0, 4 );

  // EIM_CS1GCR2   
  //  MUX16_BYP_GRANT = 1
  //  ADH = 0 (0 cycles)
  //  0x1000
  write_kernel_memory( 0x21b8004 + 0x18, 0x1000, 0, 4 );


  // 9 cycles is total length of read
  // 2 cycles for address
  // +4 more cycles for first data to show up

  // EIM_CS1RCR1   
  // 00 000100 0 000   0   001   0 010 0 000 0 000 0 000
  //    RWSC     RADVA RAL RADVN   OEA   OEN   RCSA  RCSN
  //
  // 00 001001 0 000   0   001   0 110 0 000 0 000 0 000
  //    RWSC     RADVA RAL RADVN   OEA   OEN   RCSA  RCSN
  //
  // 0000 0111 0000   0011   0000 0000 0000 0000
  //  0    7     0     3      0  0    0    0
  // 0000 0101 0000   0000   0 000 0 000 0 000 0 000
//  write_kernel_memory( 0x21b8008, 0x05000000, 0, 4 );
  // 0000 0011 0000   0001   0001 0000 0000 0000

  // 0000 1001 0000   0001   0110 0000 0000 0000
  // 
  write_kernel_memory( 0x21b8008 + 0x18, 0x09014000, 0, 4 );

  // EIM_CS1RCR2  
  // 0000 0000 0   000 00 00 0 010  0 001 
  //           APR PAT    RL   RBEA   RBEN
  // APR = 0   mandatory because MUM = 1
  // PAT = XXX because APR = 0
  // RL = 00   because async mode
  // RBEA = 000  these match RCSA/RCSN from previous field
  // RBEN = 000
  // 0000 0000 0000 0000 0000  0000
  write_kernel_memory( 0x21b800c + 0x18, 0x00000200, 0, 4 );

  // EIM_CS1WCR1
  // 0   0    000010 000   001   000  000  010 000 000  000
  // WAL WBED WWSC   WADVA WADVN WBEA WBEN WEA WEN WCSA WCSN
  // WAL = 0       use WADVN
  // WBED = 0      allow BE during write
  // WWSC = 000100 4 write wait states
  // WADVA = 000   same as RADVA
  // WADVN = 000   this sets WE length to 1 (this value +1)
  // WBEA = 000    same as RBEA
  // WBEN = 000    same as RBEN
  // WEA = 010     2 cycles between beginning of access and WE assertion
  // WEN = 000     1 cycles to end of WE assertion
  // WCSA = 000    cycles to CS assertion
  // WCSN = 000    cycles to CS negation
  // 1000 0111 1110 0001 0001  0100 0101 0001
  // 8     7    E    1    1     4    5    1
  // 0000 0111 0000 0100 0000  1000 0000 0000
  // 0      7    0   4    0     8    0     0
  // 0000 0100 0000 0000 0000  0100 0000 0000
  //  0    4    0    0     0    4     0    0

  // 0000 0010 0000 0000 0000  0010 0000 0000
  // 0000 0010 0000 0100 0000  0100 0000 0000

  write_kernel_memory( 0x21b8010 + 0x18, 0x02040400, 0, 4 );

  // EIM_WCR
  // BCM = 1   free-run BCLK
  // GBCD = 0  divide the burst clock by 1
  // add timeout watchdog after 1024 bclk cycles
  write_kernel_memory( 0x21b8090, 0x701, 0, 4 );

  // EIM_WIAR 
  // ACLK_EN = 1
  write_kernel_memory( 0x21b8094, 0x10, 0, 4 );

  printf( "resetting CS0 space to 64M and enabling 64M CS1 space.\n" );
  write_kernel_memory( 0x20e0004, 
		       (read_kernel_memory(0x20e0004, 0, 4) & 0xFFFFFFC0) |
		       0x1B, 0, 4);

  printf( "done.\n" );
}

void gpio_test() {
  write_kernel_memory( 0x08040012, 0xFF, 0, 2 );
  while(1) {
    write_kernel_memory( 0x08040010, 0xFF, 0, 2 );
  }
}

void ddr3_setadr(unsigned int port, unsigned int adr) {
  unsigned short offset = 0;
  if( port == 2)
    offset = 0;
  else if( port == 3)
    offset = 0x10;
  else {
    printf( "only ports 2 and 3 supported.\n" );
    return;
  }
  
  write_kernel_memory( FPGA_W_DDR3_P2_LADR + offset, (adr & 0xFFFF), 0, 2 );
  write_kernel_memory( FPGA_W_DDR3_P2_HADR + offset, (adr >> 16) & 0xFFFF, 0, 2 );
}

void ddr3_writedata(unsigned int data) {
  write_kernel_memory( FPGA_W_DDR3_P2_LDAT, (data & 0xFFFF), 0, 2 );
  write_kernel_memory( FPGA_W_DDR3_P2_HDAT, (data >> 16) & 0xFFFF, 0, 2 );
}

void ddr3_status() {
  unsigned short rv;
  unsigned int data;

  rv = read_kernel_memory(FPGA_R_DDR3_CAL, 0, 2);
  printf( "DDR3 pll " );
  if( rv & 0x1 )   printf( "lock" ); else  printf( "unlock" );
  printf( " cal " );
  if( rv & 0x2 )   printf( "done" ); else printf( "not done" );
  printf( "\n" );

  rv = read_kernel_memory(FPGA_R_DDR3_P2_STAT, 0, 2);
  printf( "P2 wr count: %d ", (rv >> 8) & 0x7F );
  if( rv & 0x20 )    printf( "cmd_empty " );
  if( rv & 0x10 )    printf( "cmd_full " );
  if( rv & 0x8 )    printf( "q_full " );
  if( rv & 0x4 )    printf( "q_empty " );
  if( rv & 0x2 )    printf( "q_underrun " );
  if( rv & 0x1 )    printf( "q_error " );
  printf( "\n" );

  rv = read_kernel_memory(FPGA_R_DDR3_P3_STAT, 0, 2);
  printf( "P3 rd count: %d, ", (rv >> 8) & 0x7F );
  if( rv & 0x20 )    printf( "cmd_empty " );
  if( rv & 0x10 )    printf( "cmd_full " );
  if( rv & 0x8 )    printf( "q_full " );
  if( rv & 0x4 )    printf( "q_empty " );
  if( rv & 0x2 )    printf( "q_underrun " );
  if( rv & 0x1 )    printf( "q_error " );

  rv = read_kernel_memory(FPGA_R_DDR3_P3_LDAT, 0, 2);
  data = (unsigned int) rv;
  rv = read_kernel_memory(FPGA_R_DDR3_P3_HDAT, 0, 2);
  data |= (rv << 16);
  printf( " read: %08x\n", data );

}

void ddr3_writecmd() {
  ddr3_status();
  printf( "* write data to FIFO\n" );
  write_kernel_memory( FPGA_W_DDR3_P2_WEN, 0x10, 0, 2 );
  write_kernel_memory( FPGA_W_DDR3_P2_WEN, 0x00, 0, 2 );
  ddr3_status();
  printf( "* issue write command\n" );
  write_kernel_memory( FPGA_W_DDR3_P2_CMD, 0x008, 0, 2 );
  write_kernel_memory( FPGA_W_DDR3_P2_CMD, 0x000, 0, 2 );
  ddr3_status();
}

void ddr3_readcmd() {
  ddr3_status();
  printf( "* issue read command\n" );
  write_kernel_memory( FPGA_W_DDR3_P3_CMD, 0x001, 0, 2 );
  write_kernel_memory( FPGA_W_DDR3_P3_CMD, 0x009, 0, 2 );
  write_kernel_memory( FPGA_W_DDR3_P3_CMD, 0x001, 0, 2 );
  ddr3_status();
  printf( "* advance read queue\n" );
  write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x10, 0, 2 );
  write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x00, 0, 2 );
  ddr3_status();
  
}

#define DDR3_SIZE (1024 * 1024 * 1)  // in words (4 bytes per word)
#define DDR3_FIFODEPTH 64

#define PULSE_GATE_MASK  0x1000

void ddr3_test() {
  unsigned int *testdat;
  unsigned int readback[DDR3_FIFODEPTH];
  int i;
  int burstaddr = 0;
  unsigned int data;
  int iters = 0;
  int offset;
  unsigned int rv;
  unsigned int arg = 0;

  srand(time(NULL)); // seed the random generator

  testdat = calloc(DDR3_SIZE, sizeof(unsigned int));
  if( testdat == NULL ) {
    printf( "Can't allocate test array.\n" );
    return;
  }

  while(1) {
    // dummy writes to clear any previous data in the queue -- caution, writes to "wherever"!
    while( !(read_kernel_memory(FPGA_R_DDR3_P2_STAT, 0, 2) & 4) ) {
      write_kernel_memory( FPGA_W_DDR3_P2_CMD, 0x008 | PULSE_GATE_MASK, 0, 2 );
      write_kernel_memory( FPGA_W_DDR3_P2_CMD, 0x000 | PULSE_GATE_MASK, 0, 2 );
    }
    // dummy reads to clear any previous data in the queue
    while( !(read_kernel_memory(FPGA_R_DDR3_P3_STAT, 0, 2) & 4) ) {
      write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x010, 0, 2 );
      write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x000, 0, 2 );
    }

    putchar('+'); fflush(stdout);
    for( i = 0; i < DDR3_SIZE; i++ ) {
      testdat[i] = (unsigned int) rand();
    }
    
    offset = 0;
    burstaddr = 0;
    write_kernel_memory( FPGA_W_DDR3_P2_LADR + offset, ((burstaddr * 4) & 0xFFFF), 0, 2 );
    write_kernel_memory( FPGA_W_DDR3_P2_HADR + offset, ((burstaddr * 4) >> 16) & 0xFFFF, 0, 2 );

    putchar('!'); fflush(stdout);
    while( burstaddr < DDR3_SIZE ) {
      while( !(read_kernel_memory(FPGA_R_DDR3_P2_STAT, 0, 2) & 4) ) {
	putchar('-'); fflush(stdout);  // wait for write queue to be empty
      }
      for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
	write_kernel_memory( FPGA_W_DDR3_P2_LDAT, (testdat[burstaddr + i] & 0xFFFF), 0, 2 );
	write_kernel_memory( FPGA_W_DDR3_P2_HDAT, (testdat[burstaddr + i] >> 16) & 0xFFFF, 0, 2 );
	//	write_kernel_memory( FPGA_W_DDR3_P2_WEN, 0x10, 0, 2 );
	//	write_kernel_memory( FPGA_W_DDR3_P2_WEN, 0x00, 0, 2 );
      }
      if( (read_kernel_memory(FPGA_R_DDR3_P2_STAT, 0, 2) >> 8) != DDR3_FIFODEPTH ) {
	printf( "z%d\n", (read_kernel_memory(FPGA_R_DDR3_P2_STAT, 0, 2) >> 8) );
	putchar('z'); fflush(stdout);
      }
      arg = ((DDR3_FIFODEPTH - 1) << 4);
      write_kernel_memory( FPGA_W_DDR3_P2_CMD, arg | PULSE_GATE_MASK, 0, 2 );
      arg |= 8;
      write_kernel_memory( FPGA_W_DDR3_P2_CMD, arg | PULSE_GATE_MASK, 0, 2 );
      write_kernel_memory( FPGA_W_DDR3_P2_CMD, 0x000 | PULSE_GATE_MASK, 0, 2 );
      burstaddr += DDR3_FIFODEPTH;
      write_kernel_memory( FPGA_W_DDR3_P2_LADR + offset, ((burstaddr * 4) & 0xFFFF), 0, 2 );
      write_kernel_memory( FPGA_W_DDR3_P2_HADR + offset, ((burstaddr * 4) >> 16) & 0xFFFF, 0, 2 );
    }
    
    offset = 0x10; // accessing port 3 (read port)
    burstaddr = 0;
    write_kernel_memory( FPGA_W_DDR3_P2_LADR + offset, ((burstaddr * 4) & 0xFFFF), 0, 2 );
    write_kernel_memory( FPGA_W_DDR3_P2_HADR + offset, ((burstaddr * 4) >> 16) & 0xFFFF, 0, 2 );

    putchar('.'); fflush(stdout);

    while( burstaddr < DDR3_SIZE ) {
#if 1
      arg = ((DDR3_FIFODEPTH - 1) << 4) | 1;
      write_kernel_memory( FPGA_W_DDR3_P3_CMD, arg | PULSE_GATE_MASK, 0, 2 );
      arg |= 0x8;
      write_kernel_memory( FPGA_W_DDR3_P3_CMD, arg | PULSE_GATE_MASK, 0, 2 );
      arg &= ~0x8;
      write_kernel_memory( FPGA_W_DDR3_P3_CMD, arg | PULSE_GATE_MASK, 0, 2 );
      for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
	while( (read_kernel_memory(FPGA_R_DDR3_P3_STAT, 0, 2) & 4) ) {
	  putchar('i'); fflush(stdout);// wait for queue to become full before reading
	}
	rv = read_kernel_memory(FPGA_R_DDR3_P3_LDAT, 0, 2);
	data = ((unsigned int) rv) & 0xFFFF;
	rv = read_kernel_memory(FPGA_R_DDR3_P3_HDAT, 0, 2);
	data |= (rv << 16);
	readback[i] = data;
	//	write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x10, 0, 2 );
	//	write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x00, 0, 2 );
      }
#else
      for( i = 0; i < DDR3_FIFODEPTH; i++ ) { 
	write_kernel_memory( FPGA_W_DDR3_P2_LADR + offset, 
			     (((burstaddr + i) * 4) & 0xFFFF), 0, 2 );
	write_kernel_memory( FPGA_W_DDR3_P2_HADR + offset, 
			     (((burstaddr + i) * 4) >> 16) & 0xFFFF, 0, 2 );
	write_kernel_memory( FPGA_W_DDR3_P3_CMD, 1 | PULSE_GATE_MASK, 0, 2 ); // single beat reads
	write_kernel_memory( FPGA_W_DDR3_P3_CMD, 9 | PULSE_GATE_MASK, 0, 2 );
	write_kernel_memory( FPGA_W_DDR3_P3_CMD, 1 | PULSE_GATE_MASK, 0, 2 );
	while( ((read_kernel_memory(FPGA_R_DDR3_P3_STAT, 0, 2) >> 8) == 0) ) {
	  putchar('i'); fflush(stdout);// wait for queue to become full before reading
	}
	rv = read_kernel_memory(FPGA_R_DDR3_P3_LDAT, 0, 2);
	data = ((unsigned int) rv) & 0xFFFF;
	rv = read_kernel_memory(FPGA_R_DDR3_P3_HDAT, 0, 2);
	data |= (rv << 16);
	readback[i] = data;
	//	write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x10, 0, 2 );
	//	write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x00, 0, 2 );
      }
#endif
      while( !(read_kernel_memory(FPGA_R_DDR3_P3_STAT, 0, 2) & 0x4) ) {
	putchar('x'); fflush(stdout); // error, should be empty now
	write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x10, 0, 2 );
	write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x00, 0, 2 );
      }
      for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
	if( testdat[burstaddr + i] != readback[i] ) {
	  printf( "\n%08x: %08x(w) %08x(r)", burstaddr + i, testdat[burstaddr + i], readback[i] );
	}
      }
      burstaddr += DDR3_FIFODEPTH;
      write_kernel_memory( FPGA_W_DDR3_P2_LADR + offset, ((burstaddr * 4) & 0xFFFF), 0, 2 );
      write_kernel_memory( FPGA_W_DDR3_P2_HADR + offset, ((burstaddr * 4) >> 16) & 0xFFFF, 0, 2 );
    }

    if( !(iters % 16) ) {
      printf( "\n%d iterations\n", iters );
    }
    iters++;
  }
}

void ddr3_test_opt() {
  unsigned int *testdat;
  unsigned int readback[DDR3_FIFODEPTH];
  int i;
  int burstaddr = 0;
  unsigned int data;
  int iters = 0;
  int offset;
  unsigned int rv;
  unsigned int arg = 0;
  volatile unsigned short *cs0;

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_32;

  srand(time(NULL)); // seed the random generator

  testdat = calloc(DDR3_SIZE, sizeof(unsigned int));
  if( testdat == NULL ) {
    printf( "Can't allocate test array.\n" );
    return;
  }

  while(1) {
    // dummy writes to clear any previous data in the queue -- caution, writes to "wherever"!
    while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] & 4) ) {
      cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  0x008 | PULSE_GATE_MASK;
      cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  0x000 | PULSE_GATE_MASK;
    }
    // dummy reads to clear any previous data in the queue
    while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 4) ) {
      cs0[FPGA_MAP(FPGA_W_DDR3_P3_REN)] =  0x010;
      cs0[FPGA_MAP(FPGA_W_DDR3_P3_REN)] =  0x000;
    }

    putchar('+'); fflush(stdout);
    for( i = 0; i < DDR3_SIZE; i++ ) {
      testdat[i] = (unsigned int) rand();
    }
    
    offset = 0;
    burstaddr = 0;
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_LADR + offset)] = ((burstaddr * 4) & 0xFFFF);
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_HADR + offset)] = ((burstaddr * 4) >> 16) & 0xFFFF;

    putchar('!'); fflush(stdout);
    while( burstaddr < DDR3_SIZE ) {
      while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] & 4) ) {
	putchar('-'); fflush(stdout);  // wait for write queue to be empty
      }
      for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
	cs0[FPGA_MAP(FPGA_W_DDR3_P2_LDAT)] = (testdat[burstaddr + i] & 0xFFFF);
	cs0[FPGA_MAP(FPGA_W_DDR3_P2_HDAT)] = (testdat[burstaddr + i] >> 16) & 0xFFFF;
      }
      if( (cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] >> 8) != DDR3_FIFODEPTH ) {
	printf( "z%d\n", cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] >> 8 );
	putchar('z'); fflush(stdout);
      }
      arg = ((DDR3_FIFODEPTH - 1) << 4);
      cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  arg | PULSE_GATE_MASK;
      arg |= 8;
      cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  arg | PULSE_GATE_MASK;
      cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] = 0x000 | PULSE_GATE_MASK;
      burstaddr += DDR3_FIFODEPTH;
      cs0[FPGA_MAP(FPGA_W_DDR3_P2_LADR + offset)] = ((burstaddr * 4) & 0xFFFF);
      cs0[FPGA_MAP(FPGA_W_DDR3_P2_HADR + offset)] = ((burstaddr * 4) >> 16) & 0xFFFF;
    }
    
    offset = 0x10; // accessing port 3 (read port)
    burstaddr = 0;
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_LADR + offset)] = ((burstaddr * 4) & 0xFFFF);
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_HADR + offset)] = ((burstaddr * 4) >> 16) & 0xFFFF;

    putchar('.'); fflush(stdout);

    while( burstaddr < DDR3_SIZE ) {
      arg = ((DDR3_FIFODEPTH - 1) << 4) | 1;
      cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] =  arg | PULSE_GATE_MASK;
      arg |= 0x8;
      cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] =  arg | PULSE_GATE_MASK;
      arg &= ~0x8;
      cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] = arg | PULSE_GATE_MASK;
      for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
	while( (cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 4) ) {
	  putchar('i'); fflush(stdout);// wait for queue to become full before reading
	}
	rv = cs0[FPGA_MAP(FPGA_R_DDR3_P3_LDAT)];
	data = ((unsigned int) rv) & 0xFFFF;
	rv = cs0[FPGA_MAP(FPGA_R_DDR3_P3_HDAT)];
	data |= (rv << 16);
	readback[i] = data;
      }
      while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 0x4) ) {
	putchar('x'); fflush(stdout); // error, should be empty now
	cs0[FPGA_MAP( FPGA_W_DDR3_P3_REN )] = 0x10;
	cs0[FPGA_MAP( FPGA_W_DDR3_P3_REN )] = 0x00;
      }
      for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
	if( testdat[burstaddr + i] != readback[i] ) {
	  printf( "\n%08x: %08x(w) %08x(r)", burstaddr + i, testdat[burstaddr + i], readback[i] );
	}
      }
      burstaddr += DDR3_FIFODEPTH;
      cs0[FPGA_MAP( FPGA_W_DDR3_P2_LADR + offset )] = ((burstaddr * 4) & 0xFFFF);
      cs0[FPGA_MAP( FPGA_W_DDR3_P2_HADR + offset )] = ((burstaddr * 4) >> 16) & 0xFFFF;
    }

    if( !(iters % 16) ) {
      printf( "\n%d iterations\n", iters );
    }
    iters++;
  }
}

void rom_unknown() {
  unsigned short rv;
  unsigned short count, stop;
  int i;

  rv = read_kernel_memory(FPGA_R_NAND_UK_STAT, 0, 2);
  count = rv & 0xFFF;
  printf( "count: %d ", count );
  if( rv & 0x1000 ) 
    printf( "empty " );
  if( rv & 0x2000 )
    printf( "over " );
  if( rv & 0x4000 )
    printf( "full " );
  printf( "\n" );
  
  stop = count;
  for( i = 0; i < stop; i++ ) {
    rv = read_kernel_memory(FPGA_R_NAND_UK_DATA, 0, 2);
    count = read_kernel_memory(FPGA_R_NAND_UK_STAT, 0, 2) & 0xFFF;
    printf( "%4d: %02x  %4d\n", i, rv & 0xff, count );
  }

  write_kernel_memory(FPGA_W_NAND_UK_CTL, 0x2, 0, 2 );
  write_kernel_memory(FPGA_W_NAND_UK_CTL, 0x0, 0, 2 );

}

void rom_command() {
  unsigned short rv;
  unsigned short count, stop;
  int i;

  rv = read_kernel_memory(FPGA_R_NAND_CMD_STAT, 0, 2);
  count = rv & 0xFFF;
  printf( "count: %d ", count );
  if( rv & 0x1000 ) 
    printf( "empty " );
  if( rv & 0x2000 )
    printf( "over " );
  if( rv & 0x4000 )
    printf( "full " );
  printf( "\n" );

  stop = count;
  for( i = 0; i < stop; i++ ) {
    rv = read_kernel_memory(FPGA_R_NAND_CMD_DATA, 0, 2);
    count = read_kernel_memory(FPGA_R_NAND_CMD_STAT, 0, 2) & 0xFFF;
    printf( "%4d: %02x  %4d\n", i, rv & 0xff, count );
  }

  write_kernel_memory(FPGA_W_NAND_UK_CTL, 0x4, 0, 2 );
  write_kernel_memory(FPGA_W_NAND_UK_CTL, 0x0, 0, 2 );

}

void rom_address() {
  unsigned short rv;
  unsigned short count, stop;
  unsigned long address;
  int i;

  rv = read_kernel_memory(FPGA_R_NAND_ADR_STAT, 0, 2);
  count = rv & 0x3FFF;
  printf( "count: %d ", count );
  if( rv & 0x4000 ) 
    printf( "empty " );
  if( rv & 0x8000 )
    printf( "full " );
  printf( "\n" );

  stop = count;
  for( i = 0; i < stop; i++ ) {
    rv = read_kernel_memory(FPGA_R_NAND_ADR_LOW, 0, 2);
    address = rv & 0xFFFF;
    rv = read_kernel_memory(FPGA_R_NAND_ADR_HI, 0, 2);
    address |= (rv & 0xFFFF) << 16;
    
    count = read_kernel_memory(FPGA_R_NAND_ADR_STAT, 0, 2) & 0x3FFF;
    printf( "%5d: %08lx  %c\n", i, (address & 0x7FF) | (address & 0xFFFFF000 >> 1), 
	    address & 0x800 ? '*' : ' ' );  // print * if hitting OOB area explicitly
  }

  write_kernel_memory(FPGA_W_NAND_UK_CTL, 0x10, 0, 2 ); // reset the fifo
  write_kernel_memory(FPGA_W_NAND_UK_CTL, 0x0, 0, 2 );

}

void rom_address2() {
  unsigned short rv;
  unsigned short count, stop;
  unsigned long address;
  int i;

  rv = read_kernel_memory(FPGA_R_NAND_ADR_STAT, 0, 2);
  count = rv & 0x3FFF;
  printf( "count: %d ", count );
  if( rv & 0x4000 ) 
    printf( "empty " );
  if( rv & 0x8000 )
    printf( "full " );
  printf( "\n" );

  stop = count;
  for( i = 0; i < stop; i++ ) {
    rv = read_kernel_memory(FPGA_R_NAND_ADR_LOW, 0, 2);
    address = rv & 0xFFFF;
    rv = read_kernel_memory(FPGA_R_NAND_ADR_HI, 0, 2);
    address |= (rv & 0xFFFF) << 16;
    
    count = read_kernel_memory(FPGA_R_NAND_ADR_STAT, 0, 2) & 0x3FFF;
    printf( "%5d: %08lx  %c\n", i, (address & 0x7FF) | (address & 0xFFFFF000 >> 1), 
	    address & 0x800 ? '*' : ' ' );  // print * if hitting OOB area explicitly
  }

  write_kernel_memory(FPGA_W_NAND_UK_CTL, 0x11, 0, 2 ); // reset the fifo, and set both edges
  write_kernel_memory(FPGA_W_NAND_UK_CTL, 0x1, 0, 2 );

}

#define ROM_SIZE  32768    // size in 16-bit words

void rom_dump() {
  int i;
  volatile unsigned short *cs0;

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08000000);
  cs0 = (volatile unsigned short *)mem_32;
  
  for( i = 0; i < ROM_SIZE; i++ ) {
    if( (i % 16) == 0 ) {
      printf( "\n%04x: ", i*2 );
    }
    printf( "%04hx ", cs0[i] );
  }
  printf( "\n" );
}


void rom_upload(int infile) {
  int bytes, i;
  unsigned short data[ROM_SIZE];
  volatile unsigned short *cs0;
  int errors = 0;

  for( i = 0; i < ROM_SIZE; i++ ) 
    data[i] = 0;

  bytes = read(infile, data, ROM_SIZE * sizeof(unsigned short));
  if( bytes != ROM_SIZE * sizeof(unsigned short) ) {
    printf( "Note: read only %d shorts from input file\n", bytes );
  }
  
  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08000000);
  cs0 = (volatile unsigned short *)mem_32;

  // write it in
  for( i = 0; i < ROM_SIZE; i++ ) {
    cs0[i] = data[i];
  }
  // check it
  for( i = 0, errors = 0; i < ROM_SIZE; i++ ) {
    if( cs0[i] != data[i] )
      errors++;
  }
  if( errors )
    printf( "Found %d errors on readback check\n", errors );
  else
    printf( "ROM uploaded successfully\n" );
}

void rom_upload2(int infile) {  // upload with ECC holes
  int bytes, i;
  unsigned short data[ROM_SIZE];
  volatile unsigned short *cs0;

  printf( "uploading a ROM with holes for ECC\n" );
  for( i = 0; i < ROM_SIZE; i++ ) 
    data[i] = 0;

  bytes = read(infile, data, ROM_SIZE * sizeof(unsigned short));
  if( bytes != ROM_SIZE * sizeof(unsigned short) ) {
    printf( "Note: read only %d shorts from input file\n", bytes );
  }
  
  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08000000);
  cs0 = (volatile unsigned short *)mem_32;

  // clear memory
  for( i = 0; i < ROM_SIZE; i++ ) {
    cs0[i] = 0xFFFF;
  }

  for( i = 0; i + ((i / 1024) * 32) < ROM_SIZE; i++ ) {
    cs0[i + ((i / 1024) * 32)] = data[i];
  }

}

void rom_verify(int infile) {
  int bytes, i;
  unsigned short data[ROM_SIZE];
  volatile unsigned short *cs0;
  int errors = 0;

  for( i = 0; i < ROM_SIZE; i++ ) 
    data[i] = 0;

  bytes = read(infile, data, ROM_SIZE * sizeof(unsigned short));
  if( bytes != ROM_SIZE  * sizeof(unsigned short)) {
    printf( "Note: read only %d shorts from input file\n", bytes );
  }
  
  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08000000);
  cs0 = (volatile unsigned short *)mem_32;

  // check it
  for( i = 0, errors = 0; i < ROM_SIZE; i++ ) {
    if( cs0[i] != data[i] )
      errors++;
  }
  if( errors )
    printf( "Found %d errors on readback check\n", errors );
  else
    printf( "No errors found on readback compare\n" );
}

#define LOGENTRY_LEN 8
#define FIFOWIDTH 4

#define CONFIG_SD 1

static void trace_printline(uint8_t data, uint8_t ctrl, uint32_t nsec, uint32_t sec) {
  static int i = 0;
  printf( "Packet %6d:   ", i );
  if( ctrl & 0x1 )
    printf( " ALE" );
  else 
    printf( "    " );
  if(ctrl & 0x2)
    printf( " CLE" );
  else
    printf( "    " );
  if((ctrl & 0x4) == 0)
    printf( "  WE" );
  else
    printf( "    " );
  if((ctrl & 0x8) == 0)
    printf( "  RE" );
  else
    printf( "    " );
  if((ctrl & 0x10) == 0)
    printf( "  CS" );
  else 
    printf( "    " );

  printf( " %02x  .  %u.%u\n", data, sec, nsec );

  i++;
}

void log_dump(int ofd, unsigned int records, int verbose) {
  unsigned int readback[DDR3_FIFODEPTH];
  int i;
  int burstaddr = 0;
  unsigned int data;
  int offset;
  unsigned int rv;
  unsigned int arg = 0;
  volatile unsigned short *cs0;
  unsigned char d;
  unsigned char unk[2];
  unsigned char ctrl;
  unsigned int sec, nsec;
  unsigned char buf[12];
  unsigned int log_start;

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_32;

  offset = 0x10; // accessing port 3 (read port)
#if CONFIG_SD
  printf( "Note: configured for 'SD' style tap boards & FPGA config\n" );
  burstaddr = 0x0F000000 / 4; // log starts at 16MB for non-sandisk version
  log_start = 0x0F000000 / 4;
#else
  printf( "Note: configured for 'Sandisk' style tap boards & FPGA config\n" );
  burstaddr = 0x0; // sandisk version
  log_start = 0x0;
#endif
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_LADR + offset)] = ((burstaddr * 4) & 0xFFFF);
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_HADR + offset)] = ((burstaddr * 4) >> 16) & 0xFFFF;


  if( records > 0 )
    records = records - 1; // I think there's a fencepost error somewhere...
  printf( "dumping %u records", records );
  if( verbose ) printf( "\n" );

  while( burstaddr < (records * LOGENTRY_LEN / FIFOWIDTH + log_start) ) {
    arg = ((DDR3_FIFODEPTH - 1) << 4) | 1;
    cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] =  arg | PULSE_GATE_MASK;
    arg |= 0x8;
    cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] =  arg | PULSE_GATE_MASK;
    arg &= ~0x8;
    cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] = arg | PULSE_GATE_MASK;
    for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
      while( (cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 4) ) {
	putchar('i'); fflush(stdout);// wait for queue to become full before reading
      }
      rv = cs0[FPGA_MAP(FPGA_R_DDR3_P3_LDAT)];
      data = ((unsigned int) rv) & 0xFFFF;
      rv = cs0[FPGA_MAP(FPGA_R_DDR3_P3_HDAT)];
      data |= (rv << 16);
      readback[i] = data;
    }
    while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 0x4) ) {
      putchar('x'); fflush(stdout); // error, should be empty now
      cs0[FPGA_MAP( FPGA_W_DDR3_P3_REN )] = 0x10;
      cs0[FPGA_MAP( FPGA_W_DDR3_P3_REN )] = 0x00;
    }
    for( i = 0; i < DDR3_FIFODEPTH; i += 2 ) {
      d = readback[i] & 0xFF;
      ctrl = (readback[i] >> 8) & 0x1F; // control is already lined up by FPGA mapping
      unk[0] = ((readback[i] >> 13) & 0xFF);
      unk[1] = ((readback[i] >> 21) & 0x3);
      // now prepare nsec, sec values:
      // bits 31-23 are LSB of nsec
      // 1111 1111 1_111 1111 . 1111 1111 1111 1111
      nsec = ((readback[i+1] & 0x7FFFFF) << 9) | ((readback[i] >> 23) & 0x1FF);
      sec = (readback[i+1] >> 23) & 0x1FF;
      buf[0] = d;
      buf[1] = ctrl;
      buf[2] = unk[0];
      buf[3] = unk[1];
      //      nsec = ntohl(nsec);
      //      sec = ntohl(sec);
      memcpy( buf+4, &nsec, 4 );
      memcpy( buf+8, &sec, 4 );
      //      write(ofd, &d, 1);
      //      write(ofd, &ctrl, 1);
      //      write(ofd, unk, 2);
      //      write(ofd, &nsec, 4);
      //      write(ofd, &sec, 4);
      if( (burstaddr + i + 1) < (records * LOGENTRY_LEN / FIFOWIDTH) )
	write( ofd, buf, 12 );

      if( verbose )
	trace_printline(d, ctrl, nsec, sec);
    }
    if( !verbose ) {
      if( (burstaddr % (1024 * 128)) == 0 ) {
	printf( "." );
	fflush(stdout);
      }
    }

    burstaddr += DDR3_FIFODEPTH;
    cs0[FPGA_MAP( FPGA_W_DDR3_P2_LADR + offset )] = ((burstaddr * 4) & 0xFFFF);
    cs0[FPGA_MAP( FPGA_W_DDR3_P2_HADR + offset )] = ((burstaddr * 4) >> 16) & 0xFFFF;
  }
  printf( "\n" );
  printf( "Note to self: this log needs to be parsed with https://github.com/xobs/novena-tbraw\n" );
}

void dump_ddr3(unsigned int address, unsigned int len) {
  unsigned int readback[DDR3_FIFODEPTH];
  int i;
  int burstaddr = 0;
  unsigned int data;
  int offset;
  unsigned int rv;
  unsigned int arg = 0;
  volatile unsigned short *cs0;

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_32;

  cs0[F(FPGA_W_DDR3_P3_CMD)] &= 0x7FFF;  // clear burst mode

  offset = 0x10; // accessing port 3 (read port)
  burstaddr = address / 4;
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_LADR + offset)] = ((burstaddr * 4) & 0xFFFF);
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_HADR + offset)] = ((burstaddr * 4) >> 16) & 0xFFFF;

  while( burstaddr < (address + len) / 4 ) {
    arg = ((DDR3_FIFODEPTH - 1) << 4) | 1;
    cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] =  arg | PULSE_GATE_MASK;
    arg |= 0x8;
    cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] =  arg | PULSE_GATE_MASK;
    arg &= ~0x8;
    cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] = arg | PULSE_GATE_MASK;
    for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
      while( (cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 4) ) {
	putchar('i'); fflush(stdout);// wait for queue to become full before reading
      }
      rv = cs0[FPGA_MAP(FPGA_R_DDR3_P3_LDAT)];
      data = ((unsigned int) rv) & 0xFFFF;
      rv = cs0[FPGA_MAP(FPGA_R_DDR3_P3_HDAT)];
      data |= (rv << 16);
      readback[i] = data;
    }
    while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 0x4) ) {
      putchar('x'); fflush(stdout); // error, should be empty now
      cs0[FPGA_MAP( FPGA_W_DDR3_P3_REN )] = 0x10;
      cs0[FPGA_MAP( FPGA_W_DDR3_P3_REN )] = 0x00;
    }
    for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
      if( (i % 8) == 0 )
	printf( "\n%08x: ", (burstaddr + i) * 4 );
      printf( "%08x ", readback[i] );
    }
    burstaddr += DDR3_FIFODEPTH;
    cs0[FPGA_MAP( FPGA_W_DDR3_P2_LADR + offset )] = ((burstaddr * 4) & 0xFFFF);
    cs0[FPGA_MAP( FPGA_W_DDR3_P2_HADR + offset )] = ((burstaddr * 4) >> 16) & 0xFFFF;
  }
  printf( "\n" );

}

#define DDR3_BCOUNT 16LL
void dump_ddr3_burst(unsigned int address, unsigned int len) {
  unsigned long long readback[DDR3_BCOUNT];
  int i;
  int burstaddr = 0;
  unsigned long long data;
  volatile unsigned long long *cs1;
  volatile unsigned short *cs0;

  setup_fpga_cs1();

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_32;
  cs0[F(FPGA_W_DDR3_P3_CMD)] = 0x8000;  // set burst mode for ddr3 access

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x0C040000);
  cs1 = (volatile unsigned long long *)mem_32;

  cs1[F1(FPGA_WB_DDR3_RD_CTL)] = 0x8000000000000000LL; // reset the interface
  cs1[F1(FPGA_WB_DDR3_RD_CTL)] = 0x0000000000000000LL;

  usleep(1000);

  printf( "flags: " );
  //  data = cs1[F1(FPGA_RB_DDR3_RD_STAT)];
  memcpy(&(data),(void *) &(cs1[F1(FPGA_RB_DDR3_RD_STAT)]), 8);

  if( data & 0x1000 ) 
    printf( "cmd_err " );
  if( data & 0x0800 )
    printf( "cmd_empty " );
  if( data & 0x0400 )
    printf( "cmd_full " );
  if( data & 0x0200 )
    printf( "data_empty " );
  if( data & 0x0100 )
    printf( "data_full " );
  printf( "fifocount: %d, readcount: %d\n", (int) data & 0x7F, (int) (data >> 16) & 0xFF );


  burstaddr = address;

  while( burstaddr < address + len ) {
    cs1[F1(FPGA_WB_DDR3_RD_CTL)] = (burstaddr & 0xFFFFFFFF) | (DDR3_BCOUNT << 32);
    
    for( i = 0; i < DDR3_BCOUNT; i++ ) {
      //      memcpy(&(readback[i]),(void *) &(cs1[F1(FPGA_RB_DDR3_RD_DATA)]), 8);
      readback[i] = cs1[F1(FPGA_RB_DDR3_RD_DATA)]; 
    }

    for( i = 0; i < DDR3_BCOUNT; i++ ) {
      if( (i % 4) == 0 )
	printf( "\n%08x: ", burstaddr + i * 8 );
      printf( "%016llx ", readback[i] );
    }

    burstaddr += (DDR3_BCOUNT * 8);
  }
  printf( "\n" );
  printf( "flags: " );
  //  data = cs1[F1(FPGA_RB_DDR3_RD_STAT)];
  memcpy(&(data),(void *) &(cs1[F1(FPGA_RB_DDR3_RD_STAT)]), 8);

  if( data & 0x1000 ) 
    printf( "cmd_err " );
  if( data & 0x0800 )
    printf( "cmd_empty " );
  if( data & 0x0400 )
    printf( "cmd_full " );
  if( data & 0x0200 )
    printf( "data_empty " );
  if( data & 0x0100 )
    printf( "data_full " );
  printf( "fifocount: %d, readcount: %d\n", (int) data & 0x7F, (int) (data >> 16) & 0xFF );

  if( ((data >> 16) & 0xFF) != DDR3_BCOUNT ) {
    printf( "Compiler generated non-64bit access to 64bit-only region!\n" );
  } else {
    printf( "Compiler correctly generated 64-bit accesses\n" );
  }

}


void ddr3_bverify(unsigned int ifd) {
  unsigned long long *buf;
  unsigned long long readback[DDR3_BCOUNT];
  int i;
  int burstaddr = 0;
  unsigned long long data;
  unsigned int actual;
  volatile unsigned long long *cs1;
  volatile unsigned short *cs0;

  setup_fpga_cs1();

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_32;
  cs0[F(FPGA_W_DDR3_P3_CMD)] = 0x8000;  // set burst mode for ddr3 access

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x0C040000);
  cs1 = (volatile unsigned long long *)mem_32;

  cs1[F1(FPGA_WB_DDR3_RD_CTL)] = 0x8000000000000000LL; // reset the interface
  cs1[F1(FPGA_WB_DDR3_RD_CTL)] = 0x0000000000000000LL;

  usleep(1000);

  burstaddr = 0;

  buf = (unsigned long long *) calloc(256 * 1024 * 1024 / 4, 4);
  if( buf == NULL ) {
    printf( "Unable to allocate 256MB shadow area. Yell at bunnie for making crappy code.\n" );
    return;
  }
  actual = read(ifd, buf, 256 * 1024 * 1024);

  while( burstaddr < actual ) {
    if( (burstaddr % (1024 * 1024 * 4)) == 0 ) {
      printf( "." );
      fflush(stdout);
    }
    cs1[F1(FPGA_WB_DDR3_RD_CTL)] = (burstaddr & 0xFFFFFFFF) | (DDR3_BCOUNT << 32);

    // this memcpy is necessary because:
    // we aren't doing variable-latency reads
    // the read request initiated from above actually takes a shy bit more time than
    // the natural wait state of the i.MX6. When doing a large (256MB) memory dump
    // maybe 20-30 accesses will fail. Probably unlucky with respect to when the
    // refresh cycle has to happen.
    //
    // Just a couple bclk delays are needed to guarantee timing, this dummy read below
    // is conservative but definitely meets timing requirement
    memcpy(&(data),(void *) &(cs1[F1(FPGA_RB_DDR3_RD_STAT)]), 8); // dummy read to allow for access
    
    for( i = 0; i < DDR3_BCOUNT; i++ ) {
      readback[i] = cs1[F1(FPGA_RB_DDR3_RD_DATA)]; 
    }
    
    for( i = 0; i < DDR3_BCOUNT; i++ ) {
      if(buf[(burstaddr / 8) + i] != readback[i]) {
	printf( "\n%08x: ", (burstaddr + i) * 4);
	printf( "+%016llx -%016llx", buf[(burstaddr / 8) + i], readback[i] );
      }
    }

    burstaddr += (DDR3_BCOUNT * 8);
  }
  printf( "\n" );

  //  data = cs1[F1(FPGA_RB_DDR3_RD_STAT)];
  memcpy(&(data),(void *) &(cs1[F1(FPGA_RB_DDR3_RD_STAT)]), 8);

  if( ((data >> 16) & 0xFF) != DDR3_BCOUNT ) {
    printf( "Compiler generated non-64bit access to 64bit-only region!\n" );
  } else {
    printf( "Compiler correctly generated 64-bit accesses\n" );
  }

}


void log_trace(int ofd, unsigned int records) {
  unsigned int readback[DDR3_FIFODEPTH];
  int i;
  int burstaddr = 0;
  unsigned int data;
  int offset;
  unsigned int rv;
  unsigned int arg = 0;
  volatile unsigned short *cs0;
  unsigned char d;
  unsigned char unk[2];
  unsigned char ctrl;
  unsigned int sec, nsec;
  unsigned char buf[12];
  unsigned int log_start;

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_32;

  offset = 0x10; // accessing port 3 (read port)
#if CONFIG_SD
  printf( "Note: configured for 'SD' style tap boards & FPGA config\n" );
  burstaddr = 0x0F000000; // log starts at 16MB for non-sandisk version
  log_start = 0x0F000000;
#else
  printf( "Note: configured for 'Sandisk' style tap boards & FPGA config\n" );
  burstaddr = 0x0; // sandisk version
  log_start = 0x0;
#endif
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_LADR + offset)] = ((burstaddr * 4) & 0xFFFF);
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_HADR + offset)] = ((burstaddr * 4) >> 16) & 0xFFFF;

  records = records - 1; // I think there's a fencepost error somewhere...
  printf( "dumping %d records", records );

  while( burstaddr < (records * LOGENTRY_LEN / FIFOWIDTH + log_start) ) {
    arg = ((DDR3_FIFODEPTH - 1) << 4) | 1;
    cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] =  arg | PULSE_GATE_MASK;
    arg |= 0x8;
    cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] =  arg | PULSE_GATE_MASK;
    arg &= ~0x8;
    cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] = arg | PULSE_GATE_MASK;
    for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
      while( (cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 4) ) {
	putchar('i'); fflush(stdout);// wait for queue to become full before reading
      }
      rv = cs0[FPGA_MAP(FPGA_R_DDR3_P3_LDAT)];
      data = ((unsigned int) rv) & 0xFFFF;
      rv = cs0[FPGA_MAP(FPGA_R_DDR3_P3_HDAT)];
      data |= (rv << 16);
      readback[i] = data;
    }
    while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 0x4) ) {
      putchar('x'); fflush(stdout); // error, should be empty now
      cs0[FPGA_MAP( FPGA_W_DDR3_P3_REN )] = 0x10;
      cs0[FPGA_MAP( FPGA_W_DDR3_P3_REN )] = 0x00;
    }
    for( i = 0; i < DDR3_FIFODEPTH; i += 2 ) {
      d = readback[i+1] & 0xFF;
      ctrl = (readback[i+1] >> 8) & 0x1F; // control is already lined up by FPGA mapping
      unk[0] = ((readback[i+1] >> 13) & 0xFF);
      unk[1] = ((readback[i+1] >> 21) & 0x3);
      // now prepare nsec, sec values:
      // bits 31-23 are LSB of nsec
      // 1111 1111 1_111 1111 . 1111 1111 1111 1111
      nsec = ((readback[i] & 0x7FFFFF) << 9) | ((readback[i+1] >> 23) & 0x1FF);
      sec = (readback[i] >> 23) & 0x1FF;
      buf[0] = d;
      buf[1] = ctrl;
      buf[2] = unk[0];
      buf[3] = unk[1];
      //      nsec = ntohl(nsec);
      //      sec = ntohl(sec);
      memcpy( buf+4, &nsec, 4 );
      memcpy( buf+8, &sec, 4 );
      //      write(ofd, &d, 1);
      //      write(ofd, &ctrl, 1);
      //      write(ofd, unk, 2);
      //      write(ofd, &nsec, 4);
      //      write(ofd, &sec, 4);
      if( (burstaddr + i + 1) < (records * LOGENTRY_LEN / FIFOWIDTH) )
	write( ofd, buf, 12 );
    }
    if( (burstaddr % (1024 * 128)) == 0 ) {
	printf( "." );
	fflush(stdout);
    }

    burstaddr += DDR3_FIFODEPTH;
    cs0[FPGA_MAP( FPGA_W_DDR3_P2_LADR + offset )] = ((burstaddr * 4) & 0xFFFF);
    cs0[FPGA_MAP( FPGA_W_DDR3_P2_HADR + offset )] = ((burstaddr * 4) >> 16) & 0xFFFF;
  }
  printf( "\n" );
  printf( "Note to self: this log needs to be parsed with https://github.com/xobs/novena-tbraw\n" );
}

void get_time(unsigned int *s, unsigned int *ns) {
  volatile unsigned short *cs0;

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_32;

  *ns = cs0[FPGA_MAP(FPGA_R_LOG_TIME_NSL)] & 0xFFFF;
  *ns |= cs0[FPGA_MAP(FPGA_R_LOG_TIME_NSH)] << 16;
  *s = cs0[FPGA_MAP(FPGA_R_LOG_TIME_SL)] & 0xFFFF;
  *s |= cs0[FPGA_MAP(FPGA_R_LOG_TIME_SH)] << 16;

}

#define TESTLEN 4096
void make_testdat(unsigned char *testdat) {
  int i;
#if 1  
  for( i = 0; i < TESTLEN; i++ ) {
    //    testdat[i] = (i >> 3);  // 00 00 00 00 01 01 01 01 02 02 02 02 ....
    if( i % 2 )
      testdat[i] = 0xAA;
    else
      testdat[i] = 0x55;
  }
#else
  for( i = 0; i < TESTLEN; i++ ) {
    testdat[i] = 0xDD;
  }
#endif

}

void print_sector(unsigned char *testdat) {
  int i;
  for( i = 0; i < 512; i++ ) {
    if( (i % 16) == 0 )
      printf( "\n%4x: ", i );
    printf("%02x ", testdat[i]);
  }
  printf( "\n" );
}

void log_test() {
  unsigned int s;
  unsigned int ns;
  double timeval;
  struct sd_state *state;
  unsigned int log_entries;
  unsigned short log_stat;
  volatile unsigned short *cs0;
  unsigned char testdat[TESTLEN];

  setup_fpga();
  //  setup_fpga_cs1();

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_32;


  cs0[FPGA_MAP(FPGA_W_LOG_CMD)] =  0x1;  // reset the log
  cs0[FPGA_MAP(FPGA_W_LOG_CMD)] = 0x0;
  usleep(10000);

  cs0[FPGA_MAP(FPGA_W_LOG_CMD)] = 0x2; // run the log
  

  //////////// 
  pkt_send_hello(NULL);
  state = sd_init(MISO_PIN, MOSI_PIN, CLK_PIN, CS_PIN, POWER_PIN);
  if (!state)
    return;
  pkt_send_reset((struct sd *)state);
  sd_reset(state, 2);

  printf("CSD:\n");
  send_cmdX(state, 9, 0, 0, 0, 0, 32);
  //  pkt_send_command((struct sd *)state, struct sd_cmd *cmd, uint8_t start_stop);
  printf("\n");

  printf("CID:\n");
  send_cmdX(state, 10, 0, 0, 0, 0, 32);
  //  pkt_send_command((struct sd *)state, struct sd_cmd *cmd, uint8_t start_stop);
  printf("\n");
  sleep(1);
  /////////////

  
  ns = cs0[FPGA_MAP(FPGA_R_LOG_TIME_NSL)] & 0xFFFF;
  ns |= cs0[FPGA_MAP(FPGA_R_LOG_TIME_NSH)] << 16;
  s = cs0[FPGA_MAP(FPGA_R_LOG_TIME_SL)] & 0xFFFF;
  s |= cs0[FPGA_MAP(FPGA_R_LOG_TIME_SH)] << 16;

  timeval = (double) s;
  timeval = timeval + ((double) ns / 1000000000);

  printf( "time according to the FPGA: %lf seconds\n", timeval );

  log_entries = cs0[FPGA_MAP(FPGA_R_LOG_ENTRY_L)] & 0xFFFF;
  log_entries |= cs0[FPGA_MAP(FPGA_R_LOG_ENTRY_H)] << 16;
  log_stat = cs0[FPGA_MAP(FPGA_R_LOG_STAT)];

  printf( "log entries: %u\n", log_entries );
  if( log_stat & 0x1 )
    printf( "Note: data fifo overflowed\n" );
  if( log_stat & 0x2 )
    printf( "Note: command fifo overflowed\n" );
  if( log_stat & 0x4 )
    printf( "Note: extended command fifo underflowed\n" );
  if( log_stat & 0x8 )
    printf( "Note: extended command fifo overflowed\n" );
  printf( "Max command fifo depth: %d\n", (log_stat >> 4) & 0x1F );
  printf( "Max data fifo depth: %d\n", (log_stat >> 9) & 0x7F );

  log_stat = cs0[FPGA_MAP(FPGA_R_LOG_DEBUG)];
  if( log_stat & 0x1 )
    printf( "Note: data fifo currently shows full\n" );
  else
    printf( "Note: data fifo currently shows not full\n" );

  ////////////
#if 0
  printf( "writing data\n" );
  make_testdat(testdat);
  // write some data in
  sd_write_block (state, 10000,		/* Start sector number (LBA) */
		  testdat,	/* Pointer to the data to be written */
		  TESTLEN / 512			/* Sector count (1..128) */
		  );
#endif
#if 1
  memset(testdat, 0, TESTLEN);
  printf( "reading data\n" );
  sd_read_block (
		 state, 10000,	/* Start sector number (LBA) */
		 testdat,		/* Pointer to the data buffer to store read data */
		 1		/* Sector count (1..128) */
		 );

  printf( "Read back sector dump:\n" );
  print_sector(testdat);
#endif

  cs0[FPGA_MAP(FPGA_W_LOG_CMD)] = 0x0;  // stop the log

  ns = cs0[FPGA_MAP(FPGA_R_LOG_TIME_NSL)] & 0xFFFF;
  ns |= cs0[FPGA_MAP(FPGA_R_LOG_TIME_NSH)] << 16;
  s = cs0[FPGA_MAP(FPGA_R_LOG_TIME_SL)] & 0xFFFF;
  s |= cs0[FPGA_MAP(FPGA_R_LOG_TIME_SH)] << 16;

  timeval = (double) s;
  timeval = timeval + ((double) ns / 1000000000);

  printf( "time according to the FPGA: %lf seconds\n", timeval );

  log_entries = cs0[FPGA_MAP(FPGA_R_LOG_ENTRY_L)] & 0xFFFF;
  log_entries |= cs0[FPGA_MAP(FPGA_R_LOG_ENTRY_H)] << 16;
  log_stat = cs0[FPGA_MAP(FPGA_R_LOG_STAT)];

  printf( "log entries: %u\n", log_entries );
  if( log_stat & 0x1 )
    printf( "Note: data fifo overflowed\n" );
  if( log_stat & 0x2 )
    printf( "Note: command fifo overflowed\n" );
  if( log_stat & 0x4 )
    printf( "Note: extended command fifo underflowed\n" );
  if( log_stat & 0x8 )
    printf( "Note: extended command fifo overflowed\n" );
  printf( "Max command fifo depth: %d\n", (log_stat >> 4) & 0x1F );
  printf( "Max data fifo depth: %d\n", (log_stat >> 9) & 0x7F );

  log_stat = cs0[FPGA_MAP(FPGA_R_LOG_DEBUG)];
  if( log_stat & 0x1 )
    printf( "Note: data fifo currently shows full\n" );
  else
    printf( "Note: data fifo currently shows not full\n" );

}

void g_cs() {
  ((volatile unsigned short *)mem_32)[F(FPGA_W_GPIOB_DOUT)] &= ~B_CS;
}

void g_uncs() {
  ((volatile unsigned short *)mem_32)[F(FPGA_W_GPIOB_DOUT)] |= B_CS;
}

void g_cmd(unsigned char cmd) {
  volatile unsigned short *cs0;
  cs0 = (volatile unsigned short *)mem_32;

  cs0[F(FPGA_W_GPIOB_DIR)] = 0x1FFF; 
  //  printf( "dir: %04x\n", cs0[F(FPGA_W_GPIOB_DIR)] );
  cs0[F(FPGA_W_GPIOB_DOUT)] &= 0xFF00;
  //  printf( "dat: %04x\n", cs0[F(FPGA_W_GPIOB_DOUT)] );
  cs0[F(FPGA_W_GPIOB_DOUT)] |= ((cmd & 0xFF) | B_CLE);
  //  printf( "dat: %04x\n", cs0[F(FPGA_W_GPIOB_DOUT)] );
  cs0[F(FPGA_W_GPIOB_DOUT)] &= ~B_WE;
  //  printf( "dat: %04x\n", cs0[F(FPGA_W_GPIOB_DOUT)] );

  cs0[F(FPGA_W_GPIOB_DOUT)] |= B_WE;
  //  printf( "dat: %04x\n", cs0[F(FPGA_W_GPIOB_DOUT)] );
  cs0[F(FPGA_W_GPIOB_DOUT)] &= ~B_CLE;
  //  printf( "dat: %04x\n", cs0[F(FPGA_W_GPIOB_DOUT)] );
  cs0[F(FPGA_W_GPIOB_DIR)] = 0x1F00; 
  //  printf( "dir: %04x\n", cs0[F(FPGA_W_GPIOB_DIR)] );
}

void g_adr(unsigned char adr) {
  volatile unsigned short *cs0;
  cs0 = (volatile unsigned short *)mem_32;

  cs0[F(FPGA_W_GPIOB_DIR)] = 0x1FFF; 
  //  printf( "dir: %04x\n", cs0[F(FPGA_W_GPIOB_DIR)] );
  cs0[F(FPGA_W_GPIOB_DOUT)] &= 0xFF00;
  //  printf( "dat: %04x\n", cs0[F(FPGA_W_GPIOB_DOUT)] );
  cs0[F(FPGA_W_GPIOB_DOUT)] |= ((adr & 0xFF) | B_ALE);
  //  printf( "dat: %04x\n", cs0[F(FPGA_W_GPIOB_DOUT)] );
  cs0[F(FPGA_W_GPIOB_DOUT)] &= ~B_WE;
  //  printf( "dat: %04x\n", cs0[F(FPGA_W_GPIOB_DOUT)] );

  cs0[F(FPGA_W_GPIOB_DOUT)] |= B_WE;
  //  printf( "dat: %04x\n", cs0[F(FPGA_W_GPIOB_DOUT)] );
  cs0[F(FPGA_W_GPIOB_DOUT)] &= ~B_ALE;
  //  printf( "dat: %04x\n", cs0[F(FPGA_W_GPIOB_DOUT)] );
  cs0[F(FPGA_W_GPIOB_DIR)] = 0x1F00; 
  //  printf( "dir: %04x\n", cs0[F(FPGA_W_GPIOB_DIR)] );
}

void g_dat(unsigned char dat) {
  volatile unsigned short *cs0;
  cs0 = (volatile unsigned short *)mem_32;

  cs0[F(FPGA_W_GPIOB_DOUT)] &= 0xFF00;
  //  printf( "dat: %04x\n", cs0[F(FPGA_W_GPIOB_DOUT)] );
  cs0[F(FPGA_W_GPIOB_DOUT)] |= (dat & 0xFF);
  //  printf( "dat: %04x\n", cs0[F(FPGA_W_GPIOB_DOUT)] );

  cs0[F(FPGA_W_GPIOB_DIR)] = 0x1FFF; 
  //  printf( "dir: %04x\n", cs0[F(FPGA_W_GPIOB_DIR)] );
  cs0[F(FPGA_W_GPIOB_DOUT)] &= ~B_WE;
  //  printf( "dat: %04x\n", cs0[F(FPGA_W_GPIOB_DOUT)] );

  cs0[F(FPGA_W_GPIOB_DOUT)] |= B_WE;
  //  printf( "dat: %04x\n", cs0[F(FPGA_W_GPIOB_DOUT)] );
  cs0[F(FPGA_W_GPIOB_DIR)] = 0x1F00; 
  //  printf( "dir: %04x\n", cs0[F(FPGA_W_GPIOB_DIR)] );
}

unsigned char g_rd() {
  unsigned char retval;
  volatile unsigned short *cs0;
  cs0 = (volatile unsigned short *)mem_32;

  //  printf( "dir: %04x\n", cs0[F(FPGA_W_GPIOB_DIR)] );
  // assume: data pins are in input mode already
  cs0[F(FPGA_W_GPIOB_DOUT)] &= ~B_RE;
  //  printf( "dat: %04x\n", cs0[F(FPGA_W_GPIOB_DOUT)] );

  retval = cs0[F(FPGA_R_GPIOB_DIN)] & 0xFF;
  //  printf( "din: %04x\n", cs0[F(FPGA_R_GPIOB_DIN)] );

  cs0[F(FPGA_W_GPIOB_DOUT)] = B_WE | B_RE; // reset to normal condition
  //  printf( "dat: %04x\n", cs0[F(FPGA_W_GPIOB_DOUT)] );

  return retval;
}

void g_getid() {
  char data[16];
  int i;

  memset(data, 0, 16); // clear the data array to start with

  g_cs();
  g_cmd(0x90);
  g_adr(0x00);
  for( i = 0; i < 8; i++ ) {
    data[i] = g_rd();
  }
  g_uncs();

  printf( "ID code: " );
  for( i = 0; i < 8; i++ ) {
    printf( "%02x ", data[i] );
  }
  printf( "\n" );
}

void funkychicken() {
  volatile unsigned short *cs0;
  cs0 = (volatile unsigned short *)mem_32;

  cs0[F(FPGA_W_GPIOB_DOUT)] &= 0xFF00;
  cs0[F(FPGA_W_GPIOB_DOUT)] |= (0x15);
  cs0[F(FPGA_W_GPIOB_DIR)] = 0x1FFF; 

  cs0[F(FPGA_W_GPIOB_DOUT)] |=  B_ALE;
  cs0[F(FPGA_W_GPIOB_DOUT)] &= ~B_RE;
  cs0[F(FPGA_W_GPIOB_DOUT)] |= B_RE;
  cs0[F(FPGA_W_GPIOB_DOUT)] &= ~B_RE;
  cs0[F(FPGA_W_GPIOB_DOUT)] |= B_RE;

  cs0[F(FPGA_W_GPIOB_DOUT)] &= 0xFF00;
  cs0[F(FPGA_W_GPIOB_DOUT)] |= (0x14);

  cs0[F(FPGA_W_GPIOB_DOUT)] &= ~B_RE;
  cs0[F(FPGA_W_GPIOB_DOUT)] |= B_RE;

  cs0[F(FPGA_W_GPIOB_DOUT)] &= 0xFF00;
  cs0[F(FPGA_W_GPIOB_DOUT)] |= (0x15);

  cs0[F(FPGA_W_GPIOB_DOUT)] &= ~B_RE;
  cs0[F(FPGA_W_GPIOB_DOUT)] |= B_RE;

  cs0[F(FPGA_W_GPIOB_DOUT)] &= 0xFF00;
  cs0[F(FPGA_W_GPIOB_DOUT)] |= (0x14);

  cs0[F(FPGA_W_GPIOB_DOUT)] &= ~B_RE;
  cs0[F(FPGA_W_GPIOB_DOUT)] |= B_RE;

  cs0[F(FPGA_W_GPIOB_DOUT)] &= ~B_ALE;
  cs0[F(FPGA_W_GPIOB_DIR)] = 0x1F00; 

  g_uncs();
  g_adr(0xbd);
  g_cs();
}

void bang_read(unsigned int row, unsigned int col, unsigned char *buf, unsigned int len) {
  int i;

  g_cs();
  g_cmd(0x65);
  g_adr(row & 0xFF);
  g_adr((row >> 8) & 0xFF);
  g_adr((row >> 16) & 0xFF);
  g_uncs();

  g_cs();
  g_cmd(0xa2);
  g_cmd(0x00);

  g_adr(col &  0xFF);
  g_adr((col >> 8) & 0xFF);
  g_adr((row & 0xFF));
  g_adr((row >> 8) & 0xFF);
  g_adr((row >> 16) & 0xFF);
  g_uncs();

  g_cs();
  usleep(80);
  g_cmd(0x30);
  usleep(80);
  g_cmd(0x05);

  g_adr(col &  0xFF);
  g_adr((col >> 8) & 0xFF);
  g_adr((row & 0xFF));
  g_adr((row >> 8) & 0xFF);
  g_adr((row >> 16) & 0xFF);

  g_cmd(0xe0);
  for( i = 0; i < len; i++ ) {
    buf[i] = g_rd();
  }
  g_uncs();

}

#define MAXDEPTH 0x2400
#define COLDEPTH 0x23D0
#define MAXROWS  0x80000

void bangtest(int ofd) {
  volatile unsigned short *cs0;
  unsigned char data[MAXDEPTH];
  int i;
  struct sd_state *state;
  
  memset(data, 0, MAXDEPTH); // clear the data array to start with

  setup_fpga();
  //  setup_fpga_cs1();

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_32;

  cs0[F(FPGA_W_GPIOB_DIR)] = 0x0000; // turn off all overrides
  cs0[F(FPGA_W_MCUDRV)] = 0x0; // this disengages the MCU from the card
  usleep(10000); // wait for the MCU to shut down before driving pins

#if 1
  state = (struct sd_state *) sd_init(MISO_PIN, MOSI_PIN, CLK_PIN, CS_PIN, POWER_PIN);
  if (!state)
    return;
  sd_reset(state, 2);

  printf("CSD:\n");
  send_cmdX(state, 9, 0, 0, 0, 0, 32);
  printf("\n");

  printf("CID:\n");
  send_cmdX(state, 10, 0, 0, 0, 0, 32);
  printf("\n");
  sleep(1);
#endif

  cs0[F(FPGA_W_NAND_PWR_CTL)] = 0x1; // make sure the NAND is powered on
  cs0[F(FPGA_W_MCUDRV)] = 0x2; // this disengages the MCU from the card
  usleep(10000); // wait for the MCU to shut down before driving pins

  cs0[F(FPGA_W_GPIOB_DOUT)] = B_WE | B_RE | B_CS;
  cs0[F(FPGA_W_GPIOB_DIR)] = 0x1F00; // setup the basic driving pins

#if 0

  funkychicken();

  g_getid();

  g_cs();
  g_cmd(0x5C);
  g_cmd(0xC5);
  g_uncs();

  g_cs();
  g_cmd(0x55);
  g_adr(0x0);
  g_dat(0x1);
  g_uncs();

  g_cs();
  g_cmd(0x55);
  g_adr(0x20);
  g_dat(0xc0);
  g_uncs();
  
  g_cs();
  g_cmd(0x55);
  g_adr(0x20);
  g_dat(0xc0);
  g_uncs();

  g_cs();
  g_cmd(0x65);
  g_adr(0x0);
  g_adr(0x0);
  g_adr(0x0);
  g_uncs();

  g_cs();
  g_cmd(0xFD);
  g_uncs();
  usleep(1500);

  g_cs();
  g_cmd(0x70);
  usleep(2);
  printf( "s1 - %02x\n", g_rd() );
  g_uncs();
  
  g_getid();

  g_cs();
  g_cmd(CMD_CHARGE1);
  g_adr(0x0);
  g_adr(0x04);
  g_adr(0x14);
  g_uncs();

  g_cs();
  g_cmd(CMD_CACHE1);
  g_uncs();

  g_cs();
  g_cmd(CMD_CHARGE1);
  g_adr(0x0);
  g_adr(0x04);
  g_adr(0x14);
  g_uncs();

  g_cs();
  g_cmd(CMD_COLSET);
  g_adr(0x11);
  g_adr(0xe8);
  g_adr(0x0);
  g_adr(0x04);
  g_adr(0x14);
  g_cmd(0xE0);

  for( i = 0; i < 2300; i++ ) {
    data[i] = g_rd();
  }
  g_uncs();
#endif

  g_getid();

  printf( "Dumping NAND" );
  for( i = 0; i < 0x100000; i++ ) {
    if( (i % 0x100) == 0 ) {
      printf( "." );
      fflush(stdout);
    }
    memset(data, 0x42, MAXDEPTH); // clear the data array to 0x42 so i can identify mis-reads
    bang_read( i, 0, data, (unsigned int) MAXDEPTH ); // row col format
    write(ofd, data, COLDEPTH);
  }
  printf( "done\n" );

  for( i = 0; i < MAXDEPTH; i++ ) {
    if( i % 16 == 0 )
      printf( "\n %04x: ", i );
    printf( "%02x ", data[i] );
  }
  printf( "\n" );

  g_getid();

  cs0[F(FPGA_W_MCUDRV)] = 0x0; // this disengages the MCU from the card
  cs0[F(FPGA_W_GPIOB_DIR)] = 0x0000; // turn off all overrides
}

void ddr3load(int ifd, int verify) {
  volatile unsigned short *cs0;
  unsigned int *buf;
  unsigned int actual;
  int burstaddr = 0;
  int i;
  unsigned int arg = 0;
  int offset;
  unsigned int rv;
  unsigned int data;
  unsigned int readback[DDR3_FIFODEPTH];

  setup_fpga();
  //  setup_fpga_cs1();

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_32;

  cs0[F(FPGA_W_DDR3_P3_CMD)] &= 0x7FFF;  // clear burst mode

  buf = calloc(256 * 1024 * 1024 / 4, 4);
  if( buf == NULL ) {
    printf( "Unable to allocate 256MB shadow area. Yell at bunnie for making crappy code.\n" );
    return;
  }
  actual = read(ifd, buf, 256 * 1024 * 1024);
  actual = actual / 4;
  if( verify == 2 ) {
    // "quick load" flag set
    verify = 0;
    actual = 1024 * 1024 * 2; // just load 2 meg of data
  }
  printf( "Writing %x 32-bit words\n", actual );
  if( (actual % 64) != 0 )
    printf( "Warning: number of bytes being written is not divisible by 64\n" );

  if( !verify ) {
  // dummy writes to clear any previous data in the queue -- caution, writes to "wherever"!
  while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] & 4) ) {
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  0x008 | PULSE_GATE_MASK;
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  0x000 | PULSE_GATE_MASK;
  }
  // dummy reads to clear any previous data in the queue
  while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 4) ) {
    cs0[FPGA_MAP(FPGA_W_DDR3_P3_REN)] =  0x010;
    cs0[FPGA_MAP(FPGA_W_DDR3_P3_REN)] =  0x000;
  }

  putchar('+'); fflush(stdout);
    
  offset = 0;
  burstaddr = 0;
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_LADR + offset)] = ((burstaddr * 4) & 0xFFFF);
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_HADR + offset)] = ((burstaddr * 4) >> 16) & 0xFFFF;

  ////////////////////  // dummy loop to clear bad config state
  while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] & 4) ) {
    putchar('-'); fflush(stdout);  // wait for write queue to be empty
  }
  for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_LDAT)] = (buf[burstaddr + i] & 0xFFFF);
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_HDAT)] = (buf[burstaddr + i] >> 16) & 0xFFFF;
  }
  if( (cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] >> 8) != DDR3_FIFODEPTH ) {
  }
  arg = ((DDR3_FIFODEPTH - 1) << 4);
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  arg | PULSE_GATE_MASK;
  arg |= 8;
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  arg | PULSE_GATE_MASK;
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] = 0x000 | PULSE_GATE_MASK;
  burstaddr += DDR3_FIFODEPTH;
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_LADR + offset)] = ((burstaddr * 4) & 0xFFFF);
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_HADR + offset)] = ((burstaddr * 4) >> 16) & 0xFFFF;

  // dummy writes to clear any previous data in the queue -- caution, writes to "wherever"!
  while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] & 4) ) {
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  0x008 | PULSE_GATE_MASK;
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  0x000 | PULSE_GATE_MASK;
  }
  // dummy reads to clear any previous data in the queue
  while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 4) ) {
    cs0[FPGA_MAP(FPGA_W_DDR3_P3_REN)] =  0x010;
    cs0[FPGA_MAP(FPGA_W_DDR3_P3_REN)] =  0x000;
  }

  offset = 0;
  burstaddr = 0;
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_LADR + offset)] = ((burstaddr * 4) & 0xFFFF);
  cs0[FPGA_MAP(FPGA_W_DDR3_P2_HADR + offset)] = ((burstaddr * 4) >> 16) & 0xFFFF;
  /////////////////////////// end dummy code

  putchar('!'); fflush(stdout);
  while( burstaddr < actual ) {
    if( (burstaddr % (1024 * 1024)) == 0 ) {
      printf( "." );
      fflush(stdout);
    }
    while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] & 4) ) {
      putchar('-'); fflush(stdout);  // wait for write queue to be empty
    }
    for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
      cs0[FPGA_MAP(FPGA_W_DDR3_P2_LDAT)] = (buf[burstaddr + i] & 0xFFFF);
      cs0[FPGA_MAP(FPGA_W_DDR3_P2_HDAT)] = (buf[burstaddr + i] >> 16) & 0xFFFF;
    }
    if( (cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] >> 8) != DDR3_FIFODEPTH ) {
      printf( "z%d\n", cs0[FPGA_MAP(FPGA_R_DDR3_P2_STAT)] >> 8 );
      putchar('z'); fflush(stdout);
    }
    arg = ((DDR3_FIFODEPTH - 1) << 4);
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  arg | PULSE_GATE_MASK;
    arg |= 8;
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] =  arg | PULSE_GATE_MASK;
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_CMD)] = 0x000 | PULSE_GATE_MASK;
    burstaddr += DDR3_FIFODEPTH;
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_LADR + offset)] = ((burstaddr * 4) & 0xFFFF);
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_HADR + offset)] = ((burstaddr * 4) >> 16) & 0xFFFF;
  }
  } else {
    printf( "verifying...\n" );
    offset = 0x10; // accessing port 3 (read port)
    burstaddr = 0;
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_LADR + offset)] = ((burstaddr * 4) & 0xFFFF);
    cs0[FPGA_MAP(FPGA_W_DDR3_P2_HADR + offset)] = ((burstaddr * 4) >> 16) & 0xFFFF;
    
    while( burstaddr < actual ) {
      if( (burstaddr % (1024 * 1024)) == 0 ) {
	printf( "." );
	fflush(stdout);
      }
      arg = ((DDR3_FIFODEPTH - 1) << 4) | 1;
      cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] =  arg | PULSE_GATE_MASK;
      arg |= 0x8;
      cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] =  arg | PULSE_GATE_MASK;
      arg &= ~0x8;
      cs0[FPGA_MAP(FPGA_W_DDR3_P3_CMD)] = arg | PULSE_GATE_MASK;
      for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
	while( (cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 4) ) {
	  putchar('i'); fflush(stdout);// wait for queue to become full before reading
	}
	rv = cs0[FPGA_MAP(FPGA_R_DDR3_P3_LDAT)];
	data = ((unsigned int) rv) & 0xFFFF;
	rv = cs0[FPGA_MAP(FPGA_R_DDR3_P3_HDAT)];
	data |= (rv << 16);
	readback[i] = data;
      }
      while( !(cs0[FPGA_MAP(FPGA_R_DDR3_P3_STAT)] & 0x4) ) {
	putchar('x'); fflush(stdout); // error, should be empty now
	cs0[FPGA_MAP( FPGA_W_DDR3_P3_REN )] = 0x10;
	cs0[FPGA_MAP( FPGA_W_DDR3_P3_REN )] = 0x00;
      }
      for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
	if(buf[burstaddr + i] != readback[i]) {
	  printf( "\n%08x: ", (burstaddr + i) * 4);
	  printf( "+%08x -%08x", buf[burstaddr + i], readback[i] );
	}
      }
      burstaddr += DDR3_FIFODEPTH;
      cs0[FPGA_MAP( FPGA_W_DDR3_P2_LADR + offset )] = ((burstaddr * 4) & 0xFFFF);
      cs0[FPGA_MAP( FPGA_W_DDR3_P2_HADR + offset )] = ((burstaddr * 4) >> 16) & 0xFFFF;
    }

  }
  printf( "\n" );

}

void sdwrite(unsigned int sector) {
  struct sd_state *state;
  unsigned char testdat[TESTLEN];
  int ifd;

  ifd = open("/dev/null", O_CREAT | O_WRONLY | O_TRUNC, 0644 );

  init_packet_log(ifd);

  state = (struct sd_state *) sd_init(MISO_PIN, MOSI_PIN, CLK_PIN, CS_PIN, POWER_PIN);
  if (!state)
    return;

  printf( "writing data\n" );
  make_testdat(testdat);

  sd_write_block (state, sector,		/* Start sector number (LBA) */
		  testdat,	/* Pointer to the data to be written */
		  //		  TESTLEN / 512			/* Sector count (1..128) */
		  1
		  );
}

void sdread(unsigned int sector, int do_reset) {
  struct sd_state *state;
  unsigned char testdat[TESTLEN];
  int ifd;
  int i;

  for( i = 0; i < TESTLEN; i++ ) {
    testdat[i] = 0;
  }

  ifd = open("/dev/null", O_CREAT | O_WRONLY | O_TRUNC, 0644 );

  init_packet_log(ifd);

  state = (struct sd_state *) sd_init(MISO_PIN, MOSI_PIN, CLK_PIN, CS_PIN, POWER_PIN);
  if (!state)
    return;

  if( do_reset ) {
    sd_reset(state, 2);
    printf("CID:\n");
    send_cmdX(state, 10, 0, 0, 0, 0, 32);
    printf("\n");
  }
  printf( "reading data\n" );
  sd_read_block (
		 state, sector,	/* Start sector number (LBA) */
		 testdat,		/* Pointer to the data buffer to store read data */
		 1		/* Sector count (1..128) */
		 );

  printf( "Read back sector dump:\n" );
  print_sector(testdat);
}


void sd_test1(unsigned int sector, int ifd) {
  struct sd_state *state;
  unsigned char testdat[TESTLEN];
  unsigned int s;
  unsigned int ns;
  double timeval;
  unsigned int log_entries;
  unsigned short log_stat;
  volatile unsigned short *cs0;

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_32;


  init_packet_log(ifd);

  state = (struct sd_state *) sd_init(MISO_PIN, MOSI_PIN, CLK_PIN, CS_PIN, POWER_PIN);
  if (!state)
    return;
#if 1

  cs0[FPGA_MAP(FPGA_W_LOG_CMD)] =  0x1;  // reset the log
  cs0[FPGA_MAP(FPGA_W_LOG_CMD)] = 0x0;
  usleep(10000);

  printf( "writing data\n" );
  make_testdat(testdat);
  // write some data in
  printf( "log run state: %d\n",  cs0[FPGA_MAP(FPGA_W_LOG_CMD)] );
  cs0[FPGA_MAP(FPGA_W_LOG_CMD)] = 0x2; // run the log
  printf( "log run state: %d\n",  cs0[FPGA_MAP(FPGA_W_LOG_CMD)] );

  sd_write_block (state, sector,		/* Start sector number (LBA) */
		  testdat,	/* Pointer to the data to be written */
		  //		  TESTLEN / 512			/* Sector count (1..128) */
		  1
		  );
#endif
  sleep(1);
  printf( "log run state: %d\n",  cs0[FPGA_MAP(FPGA_W_LOG_CMD)] );
  cs0[FPGA_MAP(FPGA_W_LOG_CMD)] = 0x0; // stop the log
  printf( "log run state: %d\n",  cs0[FPGA_MAP(FPGA_W_LOG_CMD)] );

  printf( "reading data\n" );
  sd_read_block (
		 state, sector,	/* Start sector number (LBA) */
		 testdat,		/* Pointer to the data buffer to store read data */
		 1		/* Sector count (1..128) */
		 );

  printf( "Read back sector dump:\n" );
  print_sector(testdat);

#if 0
  sd_reset(state, 2);

  printf( "reading data\n" );
  sd_read_block (
		 state, sector,	/* Start sector number (LBA) */
		 testdat,		/* Pointer to the data buffer to store read data */
		 1		/* Sector count (1..128) */
		 );

  printf( "Read back sector dump:\n" );
  print_sector(testdat);
#endif

  ns = cs0[FPGA_MAP(FPGA_R_LOG_TIME_NSL)] & 0xFFFF;
  ns |= cs0[FPGA_MAP(FPGA_R_LOG_TIME_NSH)] << 16;
  s = cs0[FPGA_MAP(FPGA_R_LOG_TIME_SL)] & 0xFFFF;
  s |= cs0[FPGA_MAP(FPGA_R_LOG_TIME_SH)] << 16;

  timeval = (double) s;
  timeval = timeval + ((double) ns / 1000000000);

  printf( "time according to the FPGA: %lf seconds\n", timeval );

  log_entries = cs0[FPGA_MAP(FPGA_R_LOG_ENTRY_L)] & 0xFFFF;
  log_entries |= cs0[FPGA_MAP(FPGA_R_LOG_ENTRY_H)] << 16;
  log_stat = cs0[FPGA_MAP(FPGA_R_LOG_STAT)];

  printf( "log entries: %u\n", log_entries );
  if( log_stat & 0x1 )
    printf( "Note: data fifo overflowed\n" );
  if( log_stat & 0x2 )
    printf( "Note: command fifo overflowed\n" );
  if( log_stat & 0x4 )
    printf( "Note: extended command fifo underflowed\n" );
  if( log_stat & 0x8 )
    printf( "Note: extended command fifo overflowed\n" );
  printf( "Max command fifo depth: %d\n", (log_stat >> 4) & 0x1F );
  printf( "Max data fifo depth: %d\n", (log_stat >> 9) & 0x7F );

  log_stat = cs0[FPGA_MAP(FPGA_R_LOG_DEBUG)];
  if( log_stat & 0x1 )
    printf( "Note: data fifo currently shows full\n" );
  else
    printf( "Note: data fifo currently shows not full\n" );
  
  log_dump(ifd, log_entries, 1);
}

void romreset() {
  volatile unsigned short *cs0;

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_32;

  cs0[F(FPGA_W_NAND_UK_CTL)] = 0x0036;
  usleep(1);
  cs0[F(FPGA_W_NAND_UK_CTL)] = 0x0000;
}

int testcs1() {
  unsigned long long i;
  unsigned long long retval;
  volatile unsigned long long *cs1;
  unsigned long long testbuf[16];
  unsigned long long origbuf[16];

  setup_fpga_cs1();

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return 0;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x0C040000);
  cs1 = (unsigned long long *)mem_32;

  for( i = 0; i < 2; i++ ) {
    testbuf[i] = i | (i + 64) << 16 | (i + 8) << 32 | (i + 16) << 48 ;
  }
  testbuf[0] = 0xdeadbeeffeedfaceLL;
  //  testbuf[0] = 0x0LL;
  testbuf[1] = 0x5555aaaa33339999LL;

  retval = 0;

  //  memcpy( (void *) cs1, testbuf, 2*8);
  origbuf[0] = testbuf[0];
  origbuf[1] = testbuf[1];
  cs1[0] = testbuf[0];
  cs1[1] = testbuf[1];

  for( i = 0; i < 2; i++ ) {
    testbuf[i] = 0;
  }

  memcpy(testbuf,(void *) cs1, 8);
  memcpy(&(testbuf[1]),(void *)cs1 + 8, 8);
  
  for( i = 0; i < 2; i++ ) {
    printf( "%lld: %016llx\n", i, origbuf[i] );
    printf( "%lld: %016llx\n", i, testbuf[i] );
  }

  //  cs1[0] = 0xdeadbeeffeedfaceLL;
  //  cs1[1] = 0x12456789abcdef01LL;
  //  cs1[2] = 0xf0f0f0f0f0f0f0f0LL;
  //  cs1[3] = 0x12345555aaaa9876LL;

  return retval;
}

void logsd() {
  unsigned int s;
  unsigned int ns;
  unsigned int log_entries;
  volatile unsigned short *cs0;

  setup_fpga();
  //  setup_fpga_cs1();

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_32;


  cs0[FPGA_MAP(FPGA_W_LOG_CMD)] =  0x1;  // reset the log
  cs0[FPGA_MAP(FPGA_W_LOG_CMD)] = 0x0;
  usleep(10000);

  cs0[FPGA_MAP(FPGA_W_LOG_CMD)] = 0x2; // run the log

  sleep(5);
  
  cs0[FPGA_MAP(FPGA_W_LOG_CMD)] = 0x0; // stop the log
  
  log_entries = cs0[F(FPGA_R_LOG_ENTRY_L)];
  log_entries |= (cs0[F(FPGA_R_LOG_ENTRY_H)] << 16);
  
  printf( "found %d entries\n", log_entries );
  
  printf( "status is %x\n", cs0[F(FPGA_R_LOG_STAT)] );

  ns = cs0[FPGA_MAP(FPGA_R_LOG_TIME_NSL)] & 0xFFFF;
  ns |= cs0[FPGA_MAP(FPGA_R_LOG_TIME_NSH)] << 16;
  s = cs0[FPGA_MAP(FPGA_R_LOG_TIME_SL)] & 0xFFFF;
  s |= cs0[FPGA_MAP(FPGA_R_LOG_TIME_SH)] << 16;
  printf( "time is: %us.%uns\n", s, ns );
  
}

void logdepth() {
  unsigned int s;
  unsigned int ns;
  unsigned int log_entries;
  volatile unsigned short *cs0;

  setup_fpga();
  //  setup_fpga_cs1();

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_32;

  log_entries = cs0[F(FPGA_R_LOG_ENTRY_L)];
  log_entries |= (cs0[F(FPGA_R_LOG_ENTRY_H)] << 16);
  
  printf( "found %d entries\n", log_entries );

  printf( "status is 0x%x, runstate is 0x%x\n", cs0[F(FPGA_R_LOG_STAT)], 
	  cs0[FPGA_MAP(FPGA_W_LOG_CMD)] );

  ns = cs0[FPGA_MAP(FPGA_R_LOG_TIME_NSL)] & 0xFFFF;
  ns |= cs0[FPGA_MAP(FPGA_R_LOG_TIME_NSH)] << 16;
  s = cs0[FPGA_MAP(FPGA_R_LOG_TIME_SL)] & 0xFFFF;
  s |= cs0[FPGA_MAP(FPGA_R_LOG_TIME_SH)] << 16;
  printf( "time is: %us.%uns\n", s, ns );
  
}

void ddr3_logdump(unsigned int ofd, unsigned int entries) {
  unsigned long long *buf;
  unsigned long long readback[DDR3_BCOUNT];
  int i;
  int burstaddr = 0;
  unsigned long long data;

  volatile unsigned long long *cs1;
  volatile unsigned short *cs0;

  setup_fpga();
  setup_fpga_cs1();

  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_32;
  cs0[F(FPGA_W_DDR3_P3_CMD)] = 0x8000;  // set burst mode for ddr3 access

  if( entries == 0 ) {
    entries = cs0[F(FPGA_R_LOG_ENTRY_L)];
    entries |= (cs0[F(FPGA_R_LOG_ENTRY_H)] << 16);
  }


  if(mem_32)
    munmap(mem_32, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x0C040000);
  cs1 = (volatile unsigned long long *)mem_32;

  cs1[F1(FPGA_WB_DDR3_RD_CTL)] = 0x8000000000000000LL; // reset the interface
  cs1[F1(FPGA_WB_DDR3_RD_CTL)] = 0x0000000000000000LL;

  usleep(1000);

  burstaddr = 0;

  buf = (unsigned long long *) calloc(256 * 1024 * 1024 / 4, 4);
  if( buf == NULL ) {
    printf( "Unable to allocate 256MB shadow area. Yell at bunnie for making crappy code.\n" );
    return;
  }

  while( burstaddr < entries ) {
    if( (burstaddr % (1024 * 1024 * 4)) == 0 ) {
      printf( "." );
      fflush(stdout);
    }
    cs1[F1(FPGA_WB_DDR3_RD_CTL)] = (burstaddr & 0xFFFFFFFF) | (DDR3_BCOUNT << 32);

    // this memcpy is necessary because:
    // we aren't doing variable-latency reads
    // the read request initiated from above actually takes a shy bit more time than
    // the natural wait state of the i.MX6. When doing a large (256MB) memory dump
    // maybe 20-30 accesses will fail. Probably unlucky with respect to when the
    // refresh cycle has to happen.
    //
    // Just a couple bclk delays are needed to guarantee timing, this dummy read below
    // is conservative but definitely meets timing requirement
    memcpy(&(data),(void *) &(cs1[F1(FPGA_RB_DDR3_RD_STAT)]), 8); // dummy read to allow for access
    
    for( i = 0; i < DDR3_BCOUNT; i++ ) {
      readback[i] = cs1[F1(FPGA_RB_DDR3_RD_DATA)]; 
    }
    
    for( i = 0; i < DDR3_BCOUNT; i++ ) {
      buf[(burstaddr / 8) + i] = readback[i];
    }

    burstaddr += (DDR3_BCOUNT * 8);
  }
  printf( "\n" );

  //  data = cs1[F1(FPGA_RB_DDR3_RD_STAT)];
  memcpy(&(data),(void *) &(cs1[F1(FPGA_RB_DDR3_RD_STAT)]), 8);

  if( ((data >> 16) & 0xFF) != DDR3_BCOUNT ) {
    printf( "Compiler generated non-64bit access to 64bit-only region!\n" );
  } else {
    printf( "Compiler correctly generated 64-bit accesses\n" );
  }

  write( ofd, buf, entries );
}

int analysis1() {
  unsigned int *cs1_mem;
  int memfd;
  volatile unsigned long long *cs1;
  unsigned long long testWord;
  int i2cfd;
  char i2cbuf[256]; // meh too big but meh
  int i2c_read;
  int address = 0x1E; // device address of the FPGA on I2C bus 2 (hardware interface I2C3)
  int i;
  unsigned int record;
  
  i2cfd = open("/dev/i2c-2", O_RDWR);
  if( i2cfd < 0 ) {
    perror("Unable to open /dev/i2c-2\n");
    i2cfd = 0;
    return 0;
  }
  if( ioctl( i2cfd, I2C_SLAVE, address) < 0 ) {
    perror("Unable to set I2C slave device 0x1E\n");
    return 0;
  }
  
  setup_fpga();
  setup_fpga_cs1();
  
  memfd = open("/dev/mem", O_RDWR );
  if( memfd < 0 ) {
    perror("Unable to open /dev/mem a second time\n");
    memfd = 0;
    return 0;
  }
  
  cs1_mem = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x0C040000 );
  cs1 = (unsigned long long *)cs1_mem;

  // run a quick test to make sure the interface is working
  printf( "interface test: (should see 0xaa the 0x55)\n" );
  i2cbuf[0] = 0x0; i2cbuf[1] = 0xAA;
  if( write(i2cfd, i2cbuf, 2) != 2 ) {
    perror("i2c write failed\n");
  }

  i2c_read = i2c_smbus_read_byte_data(i2cfd, 0x40);
  printf( "read back %02x\n", i2c_read & 0xFF );

  i2cbuf[0] = 0x0; i2cbuf[1] = 0x55;
  if( write(i2cfd, i2cbuf, 2) != 2 ) {
    perror("i2c write failed\n");
  }

  i2c_read = i2c_smbus_read_byte_data(i2cfd, 0x40);
  printf( "read back %02x\n", i2c_read & 0xFF );
  fflush(stdout); // make sure these tests are flushed to the screen

  /// now we are all ready. First, reset the log...
  i2cbuf[0] = 0x2; i2cbuf[1] = 0x2; // reset the log
  write(i2cfd, i2cbuf, 2);
  
  i2cbuf[0] = 0x2; i2cbuf[1] = 0x1; // set the log to run
  write(i2cfd, i2cbuf, 2);

  // kick off a test poke
  testWord = 0xfacebabebeefbad5LL;
  *cs1 = testWord;
  testWord = 0x5a5aa5a5ceceececLL;
  *cs1 = testWord;

  // now read back from the log
  for( i = 0; i < 1024; i++ ) { // the log is 1024 entries long
    record = 0;
    i2c_read = i2c_smbus_read_byte_data(i2cfd,0x44);
    record = i2c_read & 0xFF;
    i2c_read = i2c_smbus_read_byte_data(i2cfd,0x45);
    record |= (i2c_read & 0xFF) << 8;
    i2c_read = i2c_smbus_read_byte_data(i2cfd,0x46);
    record |= (i2c_read & 0xFF) << 16;

    if( i > 480 && i < 600 ) {
      printf( "%04d: %08x ", i, record );
      if( record & 0x200000 )
	printf( " ADV" );
      else
	printf( "    " );
      if( record & 0x100000 )
	printf( " RD " );
      else
	printf( " WR " );
      
      if( record & 0x080000 ) 
	printf( "     " );
      else
	printf( " CS1 " );
    
      printf( " %01x ", (record >> 16) & 0x7 );
      printf( " %04x\n", record & 0xFFFF );
    }
  }

  return 0;
}

int main(int argc, char **argv) {
  unsigned int a1, a2;
  int infile = -1; 

  char *prog = argv[0];
  argv++;
  argc--;

  if(!argc) {
    print_usage(prog);
    return 1;
  }

  while(argc > 0) {
    if(!strcmp(*argv, "-h")) {
      argc--;
      argv++;
      print_usage(prog);
    } 
    else if(!strcmp(*argv, "-s")) {
      argc--;
      argv++;
      setup_fpga();
      //      setup_fpga_cs1();
    }
    else if(!strcmp(*argv, "-t")) {
      argc--;
      argv++;
      test_fpga();
    }
    else if(!strcmp(*argv, "-v")) {
      argc--;
      argv++;
      printf( "FPGA version code: %04hx.%04hx\n", 
	      read_kernel_memory(FPGA_R_DDR3_V_MINOR, 0, 2),
	      read_kernel_memory(FPGA_R_DDR3_V_MAJOR, 0, 2) );
    }
    else if(!strcmp(*argv, "-ds")) { // ddr3 status
      argc--;
      argv++;
      ddr3_status();
    }
    else if(!strcmp(*argv, "-da")) { // ddr3 address
      argc--;
      argv++;
      if( argc != 2 ) {
	printf( "usage: -da <port> <address>\n" );
	return 1;
      }
      a1 = strtoul(*argv, NULL, 10);
      argc--;
      argv++;
      a2 = strtoul(*argv, NULL, 16);
      argc--;
      argv++;
      ddr3_setadr(a1, a2);
    }
    else if(!strcmp(*argv, "-dd")) { // ddr3 write data
      argc--;
      argv++;
      if( argc != 1 ) {
	printf( "usage: -dd <data>\n" );
	return 1;
      }
      a1 = strtoul(*argv, NULL, 16);
      argc--;
      argv++;
      ddr3_writedata(a1);
    }
    else if(!strcmp(*argv, "-dw")) { // ddr3 issue write command
      argc--;
      argv++;
      ddr3_writecmd();
    }
    else if(!strcmp(*argv, "-dr")) { // ddr3 issue read command
      argc--;
      argv++;
      ddr3_readcmd();
    }
    else if(!strcmp(*argv, "-dt")) { // ddr3 stress test
      argc--;
      argv++;
      ddr3_test();
    }
    else if(!strcmp(*argv, "-dt2")) { // ddr3 stress test, optimized
      argc--;
      argv++;
      ddr3_test_opt();
    }
    else if(!strcmp(*argv, "-tg")) { // test gio
      argc--;
      argv++;
      gpio_test();
    }
    else if(!strcmp(*argv, "-rduk")) { // dump rom unknown commands
      argc--;
      argv++;
      rom_unknown();
    }
    else if(!strcmp(*argv, "-rdcmd")) { // dump rom unknown commands
      argc--;
      argv++;
      rom_command();
    }
    else if(!strcmp(*argv, "-rdadr")) { // dump rom address requests
      argc--;
      argv++;
      rom_address();
    }
    else if(!strcmp(*argv, "-rdadr2")) { // dump rom address requests (both edges)
      argc--;
      argv++;
      rom_address2();
    }
    else if(!strcmp(*argv, "-rd")) { // dump ROM contents
      argc--;
      argv++;
      rom_dump();
    }
    else if(!strcmp(*argv, "-rul")) { // upload a ROM image
      argc--;
      argv++;
      infile = open(*argv, O_RDONLY );
      if( infile == -1 ) {
	printf("Unable to open %s\n", *argv );
	return 1;
      }
      argc--;
      argv++;
      rom_upload(infile);
      close(infile);
    }
    else if(!strcmp(*argv, "-rcheck")) { // chek a ROM image
      argc--;
      argv++;
      infile = open(*argv, O_RDONLY );
      if( infile == -1 ) {
	printf("Unable to open %s\n", *argv );
	return 1;
      }
      argc--;
      argv++;
      rom_verify(infile);
      close(infile);
    }
    else if(!strcmp(*argv, "-rul2")) { // upload a ROM image with ECC holes
      argc--;
      argv++;
      infile = open(*argv, O_RDONLY );
      if( infile == -1 ) {
	printf("Unable to open %s\n", *argv );
	return 1;
      }
      argc--;
      argv++;
      rom_upload2(infile);
      close(infile);
    }
    else if(!strcmp(*argv, "-logtest")) { // test log functions
      argc--;
      argv++;
      if( argc != 1 ) {
	printf( "usage -logtest <logfile.tbraw>\n" );
	return 1;
      }
      infile = open(*argv, O_CREAT | O_WRONLY | O_TRUNC, 0644 );
      if( infile == -1 ) {
	printf("Unable to open %s\n", *argv );
	return 1;
      }
      argc--;
      argv++;
      init_packet_log(infile);
      log_test();
      close(infile);
    }
    else if(!strcmp(*argv, "-logdump")) { // raw dump log to file
      argc--;
      argv++;
      if( argc != 2 ) {
	printf( "usage -logdump <numrecs> <logdump.bin>\n" );
	return 1;
      }
      a1 = strtoul(*argv, NULL, 10);
      argc--;
      argv++;
      infile = open(*argv, O_CREAT | O_WRONLY | O_TRUNC, 0644 );
      if( infile == -1 ) {
	printf("Unable to open %s\n", *argv );
	return 1;
      }
      argc--;
      argv++;
      log_dump(infile, a1, 0);
      close(infile);
    }
    else if(!strcmp(*argv, "-ddr3dump")) { // dump DDR3 to screen
      argc--;
      argv++;
      if( argc != 2 ) {
	printf( "usage -ddr3dump <address> <count>\n" );
	return 1;
      }
      a1 = strtoul(*argv, NULL, 16);
      argc--;
      argv++;
      a2 = strtoul(*argv, NULL, 16);
      argc--;
      argv++;
      dump_ddr3(a1, a2);
    }
    else if(!strcmp(*argv, "-ddr3bdump")) { // dump DDR3 to screen using burst-mode
      argc--;
      argv++;
      if( argc != 2 ) {
	printf( "usage -ddr3bdump <address> <count>\n" );
	return 1;
      }
      a1 = strtoul(*argv, NULL, 16);
      argc--;
      argv++;
      a2 = strtoul(*argv, NULL, 16);
      argc--;
      argv++;
      dump_ddr3_burst(a1, a2);
    }
    else if(!strcmp(*argv, "-bangtest")) { // test bitbang driver routines for NAND
      argc--;
      argv++;
      if( argc != 1 ) {
	printf( "usage -bangtest <dump.bin>\n" );
	return 1;
      }
      infile = open(*argv, O_CREAT | O_WRONLY | O_TRUNC, 0644 );
      if( infile == -1 ) {
	printf("Unable to open %s\n", *argv );
	return 1;
      }
      argc--;
      argv++;
      bangtest(infile);
    }
    else if(!strcmp(*argv, "-ddr3load")) { // load DDR3 with values from file (for romulation)
      argc--;
      argv++;
      if( argc != 1 ) {
	printf( "usage -ddr3load <file>\n" );
	return 1;
      }
      infile = open(*argv, O_RDONLY );
      if( infile == -1 ) {
	printf("Unable to open %s\n", *argv );
	return 1;
      }
      argc--;
      argv++;
      ddr3load(infile, 0);
    } else if(!strcmp(*argv, "-ddr3qload")) { // load DDR3 with values from file (just first 64k)
      argc--;
      argv++;
      if( argc != 1 ) {
	printf( "usage -ddr3qload <file>\n" );
	return 1;
      }
      infile = open(*argv, O_RDONLY );
      if( infile == -1 ) {
	printf("Unable to open %s\n", *argv );
	return 1;
      }
      argc--;
      argv++;
      ddr3load(infile, 2);
    } else if(!strcmp(*argv, "-ddr3verify")) { // verify romulator image integrity
      argc--;
      argv++;
      if( argc != 1 ) {
	printf( "usage -ddr3verify <file>\n" );
	return 1;
      }
      infile = open(*argv, O_RDONLY );
      if( infile == -1 ) {
	printf("Unable to open %s\n", *argv );
	return 1;
      }
      argc--;
      argv++;
      ddr3load(infile, 1);
    } else if(!strcmp(*argv, "-ddr3bverify")) { // verify romulator image integrity
      argc--;
      argv++;
      if( argc != 1 ) {
	printf( "usage -ddr3bverify <file>\n" );
	return 1;
      }
      infile = open(*argv, O_RDONLY );
      if( infile == -1 ) {
	printf("Unable to open %s\n", *argv );
	return 1;
      }
      argc--;
      argv++;
      ddr3_bverify(infile);
    } else if(!strcmp(*argv, "-romreset")) {
      argc--;
      argv++;
      romreset(infile);
    }
    else if(!strcmp(*argv, "-sdtest1")) {  // test write/read of SD data to card (romulator verification)
      argc--;
      argv++;
      if( argc != 2 ) {
	printf( "usage -sdtest1 <sector> <logfile.bin>\n" );
	return 1;
      }
      a1 = strtoul(*argv, NULL, 16);
      argc--;
      argv++;
      infile = open(*argv, O_CREAT | O_WRONLY | O_TRUNC, 0644 );
      if( infile == -1 ) {
	printf("Unable to open %s\n", *argv );
	return 1;
      }
      argc--;
      argv++;
      sd_test1(a1, infile);
    }
    else if(!strcmp(*argv, "-sdwrite")) {  // write fixed test data to sector
      argc--;
      argv++;
      if( argc != 1 ) {
	printf( "usage -sdwrite <sector>\n" );
	return 1;
      }
      a1 = strtoul(*argv, NULL, 16);
      argc--;
      argv++;
      sdwrite(a1);
    }
    else if(!strcmp(*argv, "-sdread")) {  // dump a sector
      argc--;
      argv++;
      if( argc != 1 ) {
	printf( "usage -sdread <sector>\n" );
	return 1;
      }
      a1 = strtoul(*argv, NULL, 16);
      argc--;
      argv++;
      sdread(a1, 1);
    }
    else if(!strcmp(*argv, "-sdread_nores")) {  // dump a sector
      argc--;
      argv++;
      if( argc != 1 ) {
	printf( "usage -sdread_nores <sector>\n" );
	return 1;
      }
      a1 = strtoul(*argv, NULL, 16);
      argc--;
      argv++;
      sdread(a1, 0);
    }
    else if(!strcmp(*argv, "-testcs1")) {
      argc--;
      argv++;
      testcs1();
    }
    else if(!strcmp(*argv, "-logsd")) {
      argc--;
      argv++;
      logsd();
    }
    else if(!strcmp(*argv, "-logdepth")) {
      argc--;
      argv++;
      logdepth();
    }
    else if(!strcmp(*argv, "-logbdump")) {
      argc--;
      argv++;
      if( argc == 0 ) {
	printf( "usage -logbdump <filename> <entries>\n" );
	return 1;
      }
      infile = open(*argv, O_CREAT | O_WRONLY | O_TRUNC, 0644 );
      if( infile == -1 ) {
	printf("Unable to open %s\n", *argv );
	return 1;
      }
      argc--;
      argv++;
      if( argc == 2 ) {
	a1 = strtoul(*argv, NULL, 10);
	argc--;
	argv++;
      } else {
	a1 = 0;
      }

      ddr3_logdump(infile, a1);
      close(infile);
    } else if(!strcmp(*argv, "-a1")) { // ever-so-helpful name for "analysis 1 -- trying to figure out if bursts to CS1 work or not"
      argc--;
      argv++;
      analysis1();
    } else {
      print_usage(prog);
      return 1;
    }
  }

  return 0;
}
