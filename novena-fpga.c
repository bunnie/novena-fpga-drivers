#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>

struct reg_info {
  char *name;
  int offset;
  int size;
  char *description;
};

static int fd = 0;
static int   *mem_32 = 0;
static short *mem_16 = 0;
static char  *mem_8  = 0;
static int *prev_mem_range = 0;

#define FPGA_W_TEST0       0x08040000
#define FPGA_W_TEST1       0x08040002
#define FPGA_W_GPIOA_DOUT  0x08040010
#define FPGA_W_GPIOA_DIR   0x08040012

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

#define FPGA_R_TEST0        0x08041000
#define FPGA_R_TEST1        0x08041002
#define FPGA_R_DDR3_CAL     0x08041004
#define FPGA_R_GPIOA_DIN    0x08041010

#define FPGA_R_DDR3_P2_STAT 0x08041020
#define FPGA_R_DDR3_P3_STAT 0x08041030
#define FPGA_R_DDR3_P3_LDAT 0x08041032
#define FPGA_R_DDR3_P3_HDAT 0x08041034

#define FPGA_R_DDR3_V_MINOR 0x08041FFC
#define FPGA_R_DDR3_V_MAJOR 0x08041FFE

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
  int result;
  volatile unsigned short *cs0;
  volatile unsigned short dummy;
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


static inline int swab(int arg) {
  return ((arg&0xff)<<24) | ((arg&0xff00)<<8) | ((arg&0xff0000)>>8) | ((arg&0xff000000)>>24);
}

void setup_fpga() {
  int i;
  printf( "setting up EIM pads and configuring timing\n" );
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

  // EIM_CS0GCR1   
  // 0101 0  001 1   001    0   001 11  00  0  000  1    0   1   1   1   0   0   1
  // PSZ  WP GBC AUS CSREC  SP  DSZ BCS BCD WC BL   CREP CRE RFL WFL MUM SRD SWR CSEN
  //
  // PSZ = 0101  256 words page size
  // WP = 0      (not protected)
  // GBC = 001   min 1 cycles between chip select changes
  // AUS = 0     address shifted according to port size
  // CSREC = 001 min 1 cycles between CS, OE, WE signals
  // SP = 0      no supervisor protect (user mode access allowed)
  // DSZ = 001   16-bit port resides on DATA[15:0]
  // BCS = 11    3 clock delay for burst generation
  // BCD = 00    divide EIM clock by 1 for burst clock
  // WC = 0      specify write bust according to BL
  // BL = 000    4 words wrap burst length
  // CREP = 1    non-PSRAM, set to 1
  // CRE = 0     CRE is disabled
  // RFL = 1     fixed latency reads (don't monitor WAIT)
  // WFL = 1     fixed latency writes (don't monitor WAIT)
  // MUM = 1     multiplexed mode enabled
  // SRD = 0     no synch reads
  // SWR = 0     no synch writes
  // CSEN = 1    chip select is enabled

  // 0101 0111 1111    0001 1100  0000  1011   1   0   0   1
  // 0x5  7    F        1   C     0     B    9

  // 0101 0001 1001    0001 1100  0000  1011   1001
  // 5     1    9       1    c     0     B      9

  write_kernel_memory( 0x21b8000, 0x5191C0B9, 0, 4 );

  // EIM_CS0GCR2   
  //  MUX16_BYP_GRANT = 1
  //  ADH = 1 (1 cycles)
  //  0x1001
  write_kernel_memory( 0x21b8004, 0x1001, 0, 4 );


  // EIM_CS0RCR1   
  // 00 000101 0 000   0   000   0 000 0 000 0 000 0 000
  //    RWSC     RADVA RAL RADVN   OEA   OEN   RCSA  RCSN
  // RWSC 000101    5 cycles for reads to happen
  //
  // 0000 0111 0000   0011   0000 0000 0000 0000
  //  0    7     0     3      0  0    0    0
  // 0000 0101 0000   0000   0 000 0 000 0 000 0 000
//  write_kernel_memory( 0x21b8008, 0x05000000, 0, 4 );
  write_kernel_memory( 0x21b8008, 0x0A024000, 0, 4 );
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

  // EIM_WCR
  // BCM = 1   free-run BCLK
  // GBCD = 0  don't divide the burst clock
  write_kernel_memory( 0x21b8090, 0x1, 0, 4 );

  // EIM_WIAR 
  // ACLK_EN = 1
  write_kernel_memory( 0x21b8094, 0x10, 0, 4 );

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

#define DDR3_SIZE (1024 * 1024 * 4)  // in words (4 bytes per word)
#define DDR3_FIFODEPTH 64

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
      write_kernel_memory( FPGA_W_DDR3_P2_CMD, 0x008, 0, 2 );
      write_kernel_memory( FPGA_W_DDR3_P2_CMD, 0x000, 0, 2 );
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
	write_kernel_memory( FPGA_W_DDR3_P2_WEN, 0x10, 0, 2 );
	write_kernel_memory( FPGA_W_DDR3_P2_WEN, 0x00, 0, 2 );
      }
      if( (read_kernel_memory(FPGA_R_DDR3_P2_STAT, 0, 2) >> 8) != DDR3_FIFODEPTH ) {
	printf( "z%d\n", (read_kernel_memory(FPGA_R_DDR3_P2_STAT, 0, 2) >> 8) );
	putchar('z'); fflush(stdout);
      }
      arg = ((DDR3_FIFODEPTH - 1) << 4);
      write_kernel_memory( FPGA_W_DDR3_P2_CMD, arg, 0, 2 );
      arg |= 8;
      write_kernel_memory( FPGA_W_DDR3_P2_CMD, arg, 0, 2 );
      write_kernel_memory( FPGA_W_DDR3_P2_CMD, 0x000, 0, 2 );
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
      write_kernel_memory( FPGA_W_DDR3_P3_CMD, arg, 0, 2 );
      arg |= 0x8;
      write_kernel_memory( FPGA_W_DDR3_P3_CMD, arg, 0, 2 );
      arg &= ~0x8;
      write_kernel_memory( FPGA_W_DDR3_P3_CMD, arg, 0, 2 );
      for( i = 0; i < DDR3_FIFODEPTH; i++ ) {
	while( (read_kernel_memory(FPGA_R_DDR3_P3_STAT, 0, 2) & 4) ) {
	  putchar('i'); fflush(stdout);// wait for queue to become full before reading
	}
	rv = read_kernel_memory(FPGA_R_DDR3_P3_HDAT, 0, 2);
	data = (rv << 16);
	rv = read_kernel_memory(FPGA_R_DDR3_P3_LDAT, 0, 2);
	data |= ((unsigned int) rv) & 0xFFFF;
	readback[i] = data;
	write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x10, 0, 2 );
	write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x00, 0, 2 );
      }
#else
      for( i = 0; i < DDR3_FIFODEPTH; i++ ) { 
	write_kernel_memory( FPGA_W_DDR3_P2_LADR + offset, 
			     (((burstaddr + i) * 4) & 0xFFFF), 0, 2 );
	write_kernel_memory( FPGA_W_DDR3_P2_HADR + offset, 
			     (((burstaddr + i) * 4) >> 16) & 0xFFFF, 0, 2 );
	write_kernel_memory( FPGA_W_DDR3_P3_CMD, 1, 0, 2 ); // single beat reads
	write_kernel_memory( FPGA_W_DDR3_P3_CMD, 9, 0, 2 );
	write_kernel_memory( FPGA_W_DDR3_P3_CMD, 1, 0, 2 );
	while( ((read_kernel_memory(FPGA_R_DDR3_P3_STAT, 0, 2) >> 8) == 0) ) {
	  putchar('i'); fflush(stdout);// wait for queue to become full before reading
	}
	rv = read_kernel_memory(FPGA_R_DDR3_P3_HDAT, 0, 2);
	data = (rv << 16);
	rv = read_kernel_memory(FPGA_R_DDR3_P3_LDAT, 0, 2);
	data |= ((unsigned int) rv) & 0xFFFF;
	readback[i] = data;
	write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x10, 0, 2 );
	write_kernel_memory( FPGA_W_DDR3_P3_REN, 0x00, 0, 2 );
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

int main(int argc, char **argv) {
  int          dump_registers = 0;
  unsigned int read_offset    = 0;
  unsigned int write_offset   = 0;
  unsigned int write_value    = 0;
  int          virtualized    = 0;
  int          ch;
  int fd;
  unsigned int a1, a2;

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
    else if(!strcmp(*argv, "-tg")) { // test gio
      argc--;
      argv++;
      gpio_test();
    }
    else {
      print_usage(prog);
      return 1;
    }
  }

  return 0;
}
