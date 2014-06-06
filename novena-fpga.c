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

/** Definitions for Novena EIM interface */
#define CS_PIN    GPIO_IS_EIM | 3
#define MISO_PIN  GPIO_IS_EIM | 0
#define CLK_PIN   GPIO_IS_EIM | 4
#define MOSI_PIN  GPIO_IS_EIM | 5
#define POWER_PIN 17 //GPIO1_IO17

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

#define FPGA_W_ROMULATE_CTL 0x08040010

#define FPGA_R_TEST0        0x08041000
#define FPGA_R_TEST1        0x08041002

#define FPGA_R_ROMULATE_STAT 0x08041100
#define FPGA_R_ROMULATE_CNT  0x08041102

#define FPGA_R_ROMADR_STAT 0x08041104
#define FPGA_R_ROMADR_CNT  0x08041106
#define FPGA_R_ROMADR_DL   0x08041108
#define FPGA_R_ROMADR_DH   0x0804110A

#define FPGA_R_ROMOUT_STAT 0x0804110C
#define FPGA_R_ROMOUT_CNT  0x0804110E

#define FPGA_R_DDR3_V_MINOR 0x08041FFC
#define FPGA_R_DDR3_V_MAJOR 0x08041FFE

// burst access registers (in CS1 bank -- only 64-bit access allowed)
#define FPGA_WB_LOOP0       0x0C040000
#define FPGA_WB_LOOP1       0x0C040008

#define FPGA_RB_LOOP0       0x0C041000
#define FPGA_RB_LOOP1       0x0C041008

#define FPGA_RB_DDR3_RD_DATA 0x0C041100
#define FPGA_RB_DDR3_RD_STAT 0x0C041108


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

void romulate(int onoff) {
  volatile unsigned short *cs0;

  if(mem_16)
    munmap(mem_16, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_16 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_16;

  if( onoff )
    cs0[F(FPGA_W_ROMULATE_CTL)] |= 0x1;
  else
    cs0[F(FPGA_W_ROMULATE_CTL)] &= 0xFFFE;

}

void rom_uk(int mode) {
  volatile unsigned short *cs0;
  int i, tot;

  if(mem_16)
    munmap(mem_16, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_16 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_16;

  if( mode == 0 ) {  // status update
    printf( "ROMulator UK FIFO reports %d entries, full: %d, over: %d, empty: %d\n", 
	    cs0[F(FPGA_R_ROMULATE_CNT)] & 0x3FF, 
	    cs0[F(FPGA_R_ROMULATE_STAT)] & 0x200 ? 1 : 0,
	    cs0[F(FPGA_R_ROMULATE_STAT)] & 0x400 ? 1 : 0,
	    cs0[F(FPGA_R_ROMULATE_STAT)] & 0x100 ? 1 : 0 );
    printf( "ROMulator control register: %04x\n", cs0[F(FPGA_W_ROMULATE_CTL)] );
  } else if( mode == 1 ) { // reset count
    printf( "resetting ROMulator UK FIFO\n" );
    cs0[F(FPGA_W_ROMULATE_CTL)] |= 0x4;
    printf( "ROMulator control register: %04x\n", cs0[F(FPGA_W_ROMULATE_CTL)] );
    cs0[F(FPGA_W_ROMULATE_CTL)] &= ~0x4;
    printf( "ROMulator control register: %04x\n", cs0[F(FPGA_W_ROMULATE_CTL)] );
  } else if( mode == 2 ) { // read out
    tot = cs0[F(FPGA_R_ROMULATE_CNT)] & 0x3FF; 
    i = 0;
    printf( "Reading out %d entries\n", tot );
    while( tot > 0 ) {
      cs0[F(FPGA_W_ROMULATE_CTL)] |= 0x2; // pulse read
      cs0[F(FPGA_W_ROMULATE_CTL)] &= ~0x2;
      printf( "%d: %02x\n", i, cs0[F(FPGA_R_ROMULATE_STAT)] & 0xFF );
      i++;
      tot--;
    }
  }

}

void rom_adr(int mode) {
  volatile unsigned short *cs0;
  int i, tot;

  if(mem_16)
    munmap(mem_16, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_16 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_16;

  if( mode == 0 ) {  // status update
    printf( "ROMulator ADR FIFO reports %d entries, full: %d, over: %d, empty: %d\n", 
	    cs0[F(FPGA_R_ROMADR_CNT)] & 0x3FF, 
	    cs0[F(FPGA_R_ROMADR_STAT)] & 0x200 ? 1 : 0,
	    cs0[F(FPGA_R_ROMADR_STAT)] & 0x400 ? 1 : 0,
	    cs0[F(FPGA_R_ROMADR_STAT)] & 0x100 ? 1 : 0 );
    printf( "ROMulator control register: %04x\n", cs0[F(FPGA_W_ROMULATE_CTL)] );
  } else if( mode == 1 ) { // reset count
    printf( "resetting ROMulator ADR FIFO\n" );
    cs0[F(FPGA_W_ROMULATE_CTL)] |= 0x10;
    printf( "ROMulator control register: %04x\n", cs0[F(FPGA_W_ROMULATE_CTL)] );
    cs0[F(FPGA_W_ROMULATE_CTL)] &= ~0x10;
    printf( "ROMulator control register: %04x\n", cs0[F(FPGA_W_ROMULATE_CTL)] );
  } else if( mode == 2 ) { // read out
    tot = cs0[F(FPGA_R_ROMADR_CNT)] & 0x3FF; 
    i = 1;
    printf( "Reading out %d entries\n", i );
    while( tot > 0 ) {
      cs0[F(FPGA_W_ROMULATE_CTL)] |= 0x8; // pulse read
      cs0[F(FPGA_W_ROMULATE_CTL)] &= ~0x8;
      printf( "%d: %06x\n", i, ((cs0[F(FPGA_R_ROMADR_DH)] & 0xFF) << 16) | 
	      (cs0[F(FPGA_R_ROMADR_DL)] & 0xFFFF));
      i++;
      tot--;
    }
  }

}


void rom_out(int mode) {
  volatile unsigned short *cs0;
  int i, tot;

  if(mem_16)
    munmap(mem_16, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_16 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08040000);
  cs0 = (volatile unsigned short *)mem_16;

  if( mode == 0 ) {  // status update
    printf( "ROMulator UK FIFO reports %d entries, full: %d, over: %d, empty: %d\n", 
	    cs0[F(FPGA_R_ROMOUT_CNT)] & 0x3FF, 
	    cs0[F(FPGA_R_ROMOUT_STAT)] & 0x200 ? 1 : 0,
	    cs0[F(FPGA_R_ROMOUT_STAT)] & 0x400 ? 1 : 0,
	    cs0[F(FPGA_R_ROMOUT_STAT)] & 0x100 ? 1 : 0 );
    printf( "ROMulator control register: %04x\n", cs0[F(FPGA_W_ROMULATE_CTL)] );
  } else if( mode == 1 ) { // reset count
    printf( "resetting ROMulator out FIFO\n" );
    cs0[F(FPGA_W_ROMULATE_CTL)] |= 0x40;
    printf( "ROMulator control register: %04x\n", cs0[F(FPGA_W_ROMULATE_CTL)] );
    cs0[F(FPGA_W_ROMULATE_CTL)] &= ~0x40;
    printf( "ROMulator control register: %04x\n", cs0[F(FPGA_W_ROMULATE_CTL)] );
  } else if( mode == 2 ) { // read out
    tot = cs0[F(FPGA_R_ROMOUT_CNT)] & 0x3FF; 
    i = 0;
    printf( "Reading out %d entries\n", tot );
    while( tot > 0 ) {
      cs0[F(FPGA_W_ROMULATE_CTL)] |= 0x20; // pulse read
      cs0[F(FPGA_W_ROMULATE_CTL)] &= ~0x20;
      printf( "%d: %02x\n", i, cs0[F(FPGA_R_ROMOUT_STAT)] & 0xFF );
      i++;
      tot--;
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

  mem_32 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08010000);
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
  
  if(mem_16)
    munmap(mem_16, 0xFFFF);
  if(fd)
    close(fd);

  fd = open("/dev/mem", O_RDWR);
  if( fd < 0 ) {
    perror("Unable to open /dev/mem");
    fd = 0;
    return;
  }

  mem_16 = mmap(0, 0xffff, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x08010000);
  cs0 = (volatile unsigned short *)mem_16;

  // write it in
  for( i = 0; i < ROM_SIZE; i++ ) {
    cs0[i] = data[i];
  }
  // check it
  for( i = 0, errors = 0; i < ROM_SIZE; i++ ) {
    if( cs0[i] != data[i] ) {
      errors++;
      printf( "%04x: %04x , %04x\n", i, cs0[i], data[i] );
    }
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


int main(int argc, char **argv) {
  unsigned int a1, a2;
  int infile = -1; 

  char *prog = argv[0];
  argv++;
  argc--;

  setup_fpga();

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

    else if(!strcmp(*argv, "-uk")) {
      argc--;
      argv++;
      rom_uk(2);
    }
    else if(!strcmp(*argv, "-uk_reset")) {
      argc--;
      argv++;
      rom_uk(1);
    }
    else if(!strcmp(*argv, "-uk_stat")) {
      argc--;
      argv++;
      rom_uk(0);
    }

    else if(!strcmp(*argv, "-adr")) {
      argc--;
      argv++;
      rom_adr(2);
    }
    else if(!strcmp(*argv, "-adr_reset")) {
      argc--;
      argv++;
      rom_adr(1);
    }
    else if(!strcmp(*argv, "-adr_stat")) {
      argc--;
      argv++;
      rom_adr(0);
    }

    else if(!strcmp(*argv, "-out")) {
      argc--;
      argv++;
      rom_out(2);
    }
    else if(!strcmp(*argv, "-out_reset")) {
      argc--;
      argv++;
      rom_out(1);
    }
    else if(!strcmp(*argv, "-out_stat")) {
      argc--;
      argv++;
      rom_out(0);
    }

    else if(!strcmp(*argv, "-testcs1")) {
      argc--;
      argv++;
      testcs1();
    }
    else if(!strcmp(*argv, "-romulate")) {
      argc--;
      argv++;
      romulate(1);
    }
    else if(!strcmp(*argv, "-bypass")) {
      argc--;
      argv++;
      romulate(0);
    }
    else {
      print_usage(prog);
      return 1;
    }
  }

  return 0;
}
