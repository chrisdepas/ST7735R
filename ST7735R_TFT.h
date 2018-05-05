// A high performance TFT drawing driver library for Arduino Uno / Atmega 328P to interface
// with the Arduino.cc provided ST7735R TFT display.
// Assumes that wiring is done according to default instructions for Uno at http://www.arduino.cc/en/Guide/TFTtoBoards
// Datasheet available at http://www.adafruit.com/datasheets/ST7735R_V0.2.pdf)
// Only supports hardware SPI mode with 8MHz SPI clock.
// Written by Jukka Jylänki.
// Fork/Modifications by Chris De Pasquale
// This code is released to public domain.
#pragma once
#include <SPI.h>

//*************************************************************************************************
// For highest performance, we directly access memory-mapped port registers to make
// I/O pins bit flips one clock-cycle operations. Usually digitalWrite() or
// portOutputRegister -> digitalPinToPort -> digitalPinToBitMask is used.
// This requires that we configure pins at compile time, and also tell the compiler
// which port address and bit in that address we are accessing. An excellent diagram
// to figuring out which PORT+BIT a given I/O PIN maps to on Arduino Uno can be found
// here: http://pighixxx.com/unov3pdf.pdf
// If you are using something else than Uno, find a similar diagram to your board.
//
//
// Chip Select pin. If this is low, the TFT chip is active.
// LCD CS pin at http://www.arduino.cc/en/Guide/TFTtoBoards
// CSX pin on the MCU interface at page 13 of http://www.adafruit.com/datasheets/ST7735R_V0.2.pdf
//
// Configure Chip Select Pin
#ifndef ST7735R_CS_PIN
    #define ST7735R_CS_PIN 10
#endif
// Configure Chip Select Port
#ifndef ST7735R_CS_PORT
    #define ST7735R_CS_PORT PORTB
#endif
// Configure Chip Select Bit
#ifndef ST7735R_CS_BIT
    #define ST7735R_CS_BIT (1 << PB2)
#endif
//
//
// Display Data/Command Mode Select pin.
// If this is high, we are sending a data byte, and if this is low, we are sending a command byte.
// D/C pin at http://www.arduino.cc/en/Guide/TFTtoBoards
// "D/CX (SCL)" pin on the MCU interface at page 13 of http://www.adafruit.com/datasheets/ST7735R_V0.2.pdf
//
// Configure Data/Command mode Pin
#ifndef ST7735R_RS_PIN
    #define ST7735R_RS_PIN 9
#endif
// Configure Data/Command mode Port
#ifndef ST7735R_RS_PORT
#define ST7735R_RS_PORT PORTB
#endif
// Configure Data/Command mode Bit
#ifndef ST7735R_RS_BIT
    #define ST7735R_RS_BIT (1 << PB1)
#endif
//
//
// Reset pin.
// If this is taken low, performs a hard reset on the chip. This is optional to connect.
// Configure Reset Pin
#ifndef ST7735R_RST_PIN
    #define ST7735R_RST_PIN 8
#endif
//
//
// Screen dimensions, in pixels
// Configure Screen width
#define ST7735R_WIDTH 160
// Configure Screen height
#define ST7735R_HEIGHT 128
//
// End configuration
//*************************************************************************************************

// Helper - Screen min and max
#define ST7735R_MIN_X 0
#define ST7735R_MIN_Y 0
#define ST7735R_MAX_X ST7735R_WIDTH - 1
#define ST7735R_MAX_Y ST7735R_HEIGHT - 1


// void DisplayDrawText(int x, int y, const char *text, uint8_t r, uint8_t g, uint8_t b, uint8_t bgR, uint8_t bgG, uint8_t bgB);
// Renders text using the Monaco font. To convert fonts to C headers for inclusion, use ttffont_to_cppheader.py

// Activate the TFT for communication. This is expected to compile down to a single
// assembly instruction like "cbi  0x05, 1" (Clear Bit in I/O register) so should take
// only one clock cycle.
#define BEGIN_TFT() (ST7735R_CS_PORT &= ~ST7735R_CS_BIT)

// The TFT has two separate modes of interpreting the received data. One toggles
// between these modes by flipping a bit. The usual transmission flow proceeds
// by first activating the "command mode", then sending a byte that represents
// the command, after which communication is transitioned to "data mode" to
// push the data bytes related to that command. Before calling either of these,
// the SPI send line must be free (no pending WRITE_SPI_NOWAIT messages on the
// line, but WAIT_SPI has been called). Both of these should compile down to a single
// instruction like "cbi  0x05, 1" (Clear Bit in I/O register) and "sbi 0x05, 1"
// (Set Bit in I/O register) and should take only one clock cycle each.
#define CHOOSE_COMMAND_MODE (ST7735R_RS_PORT &= ~ST7735R_RS_BIT)
#define CHOOSE_DATA_MODE (ST7735R_RS_PORT |= ST7735R_RS_BIT)

// Deactivates the TFT for communication. Before calling this, the SPI send line
// must be free (no pending WRITE_SPI_NOWAIT messages on the line, but WAIT_SPI
// has been called). One assembly instruction sbi, like above.
#define END_TFT() (ST7735R_CS_PORT |= ST7735R_CS_BIT)

// Pushes a byte to the SPI line, but does not stall to wait until
// the communication has finished. Before calling this function, the line must
// be free for transmission, so after calling this, one must call WAIT_SPI
// at some point to ensure that the line is free to send the next byte.
// This should be a single instruction like "out  0x2e, r21".
#define WRITE_SPI_NOWAIT(b) (SPDR = (b))

// Waits until the SPI line is ready (other end has received the byte we sent
// last). After calling this, it is safe to write to SPI again.
// This while loop is expected to compile down to the following (most efficient?)
// form:
//    in r0, 0x2d (read the input port from the I/O address of the SPSR register)
//    sbrs r0, 7  (Skip executing the next instruction if bit 7 is high (sbrs = Skip if Bit in Register Set)
//    rjmp .-6    (Relative Jump back to the 'in' command if bit 7 was low)
// The 'in' instruction takes 1 clock cycle, 'rjmp' takes two, and 'sbrs' takes
// 1 - 3, depending on the result of the test (predicts zero, i.e. no skip)
// Therefore one iteration of WAIT_SPI takes a total of four cycles in the case
// when the iteration is not done but we jump back.
#define WAIT_SPI do { while (!(SPSR & 0x80)) ; } while(0)

// Does one cycle of no operation on the CPU.
#define NOP asm volatile("nop")

// The function WRITE_SPI_SYNC() writes a byte to SPI, and immediately
// (synchronously) waits that the transmission has safely finished. This is
// like WRITE_SPI_NOWAIT, but safe (and slower), since it ensures that the line
// is free afterwards. The reason that a NOP is inserted is to exploit "phase"
// with respect to the time it takes for the ST7735R to process a SPI byte (in
// relation to the 16MHz Atmega 328P at least). Since WAIT_SPI will always take
// a multiple of 4 clock cycles (as one test is 4 clocks), it means that the
// SPSR register is polled only once every 4 clocks, so depending on when the
// SPI operation finishes and which part the loop is at, there can be a delay
// of 0-3 clocks until we see the SPSR be free. Therefore we can carefully
// "prime" the phase of the loop to ensure that we should always hit the 0
// wasted clocks, which by experimental profiling happens when we insert exactly
// one NOP before entering the loop. Profiling also indicates that a SPI write
// operation takes 33 clocks almost always, so WRITE_SPI_SYNC() is therefore
// considerably much slower than WRITE_SPI_NOWAIT() which only takes one clock.
// The general strategy with all draw operations is to interleave the asynchronous
// WRITE_SPI_NOWAIT() operations with as much of other processing as possible,
// so that we can hide these 33 clock cycle busy loops and do meaningful work
// in that time.
// To experimentally confirm the above, try benchmarking the time that draw
// operations take when inserting 0-5 nops here. The results that were received
// at the time of writing, when drawing a full screen of random pixels:
//    0 nops: 553.8 msecs
//    1 nop:  526.8 msecs
//    2 nops: 535.8 msecs
//    3 nops: 545.0 msecs
//    4 nops: 553.8 msecs
//    5 nops: 526.8 msecs
// and as expected, the performance varies in a multiple of four nops, with the
// phase of 4*k+1 nops being the optimally primed pause.
#define WRITE_SPI_SYNC(b) \
  do { \
  WRITE_SPI_NOWAIT(b); \
  NOP; \
  WAIT_SPI; \
  } while(0)

// Transitions to command mode, sends the given command, and transitions back to
// data mode. The SPI line should be free before calling this. After this function
// finishes, the SPI line is free for data transfer.
#define SEND_COMMAND(cmd) \
  do { \
    CHOOSE_COMMAND_MODE; \
    WRITE_SPI_SYNC(cmd); \
    CHOOSE_DATA_MODE; \
  } while(0)

// For high performance operation, we can't afford to do very small transactions inside
// the individual operations. Therefore if you are using transactions, wrap all draw
// operations inside a block of form
//
// DisplayBeginTransaction();
//    draw commands;
// DisplayEndTransaction();
//

#if defined(SPI_HAS_TRANSACTION)
    #define DisplayBeginTransaction() SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0))
    #define DisplayEndTransaction() SPI.endTransaction()
#else
    #define DisplayBeginTransaction() ((void)0)
    #define DisplayEndTransaction() ((void)0)
#endif

// The MCU command interface implements the following instructions. These
// are called with the SEND_COMMAND(cmd) instruction.
#define ST7735R_NOP     0x00 // No operation
#define ST7735R_SWRESET 0x01 // Sofware reset
#define ST7735R_RDDID   0x04 // Read Display ID (four reads follows)
#define ST7735R_RDDST   0x09 // Read Display Status (five reads follows)
#define ST7735R_RDDPM   0x0A // Read Display Power (two reads follows)
#define ST7735R_RDDMADCTL 0x0B // Read Display (two reads follows)
#define ST7735R_RDDCOLMOD 0x0C // Read Display Pixel (two reads follows)
#define ST7735R_RDDIM   0x0D // Read Display Image (two reads follows)
#define ST7735R_RDDSM   0x0E // Read Display Signal (two reads follows)
#define ST7735R_SLPIN   0x10 // Sleep in & booster off. Default after reset: "Sleep In"
#define ST7735R_SLPOUT  0x11 // Sleep out & booster on
#define ST7735R_PTLON   0x12 // Partial mode on. Default after reset: off.
#define ST7735R_NORON   0x13 // Partial off (normal)
#define ST7735R_INVOFF  0x20 // Display inversion off. Default after reset: off.
#define ST7735R_INVON   0x21 // Display inversion on
#define ST7735R_GAMSET  0x26 // Gamma curve select (one data byte follows). Default after reset: GC0
  #define GAMSET_GC0    0x01 // Gamma curve 1 (2.2x if GS=1, 1.0x otherwise)
  #define GAMSET_GC1    0x02 // Gamma curve 2 (1.8x if GS=1, 2.5x otherwise)
  #define GAMSET_GC2    0x04 // Gamma curve 3 (2.5x if GS=1, 2.2x otherwise)
  #define GAMSET_GC3    0x08 // Gamma curve 4 (1.0x if GS=1, 1.8x otherwise)
#define ST7735R_DISPOFF 0x28 // Display off. Default after reset: off.
#define ST7735R_DISPON  0x29 // Display on
#define ST7735R_CASET   0x2A // Column address set (four data bytes follow)
#define ST7735R_RASET   0x2B // Row address set (four data bytes follow)
#define ST7735R_RAMWR   0x2C // Memory write (A custom N amount of data bytes follows)
#define ST7735R_RGBSET  0x2D // LUT for 4k, 65k, 262k color (N data bytes follows)
#define ST7735R_RAMRD   0x2E // Memory read
#define ST7735R_PTLAR   0x30 // Partial start/end address set (four data bytes follow)
#define ST7735R_TEOFF   0x34 // Tearing effect line off
#define ST7735R_TEON    0x35 // Tearing effect mode set & on (one data byte follows)
#define ST7735R_MADCTL  0x36 // Memory data access control (one data byte follows)
  #define MADCTL_MH  0x04    // Horizontal refresh order. 0: left to right, 1: right to left.
  #define MADCTL_RGB 0x08    // RGB/BGR order. 0: RGB color filter panel, 1: BGR color filter panel.
  #define MADCTL_ML  0x10    // Vertical refresh order. 0: top to bottom, 1: bottom to top
  #define MADCTL_MV  0x20    // Row/Column exchange.
  #define MADCTL_MX  0x40    // Column address order.
  #define MADCTL_MY  0x80    // Row address order.
#define ST7735R_IDMOFF  0x38 // Idle mode off
#define ST7735R_IDMON   0x39 // Idle mode on
#define ST7735R_COLMOD  0x3A // Interface pixel format (one data byte follows)
  #define COLMOD_12BPP  0x03
  #define COLMOD_16BPP  0x05
  #define COLMOD_18BPP  0x06
  #define COLMOD_WRITE_16BPP 0x55 // Datasheet states: "The command 3Ah should be set at 55h when writing 16-bit/pixel data into frame memory". Not sure how this differs from COLMOD_16BPP though.
#define ST7735R_RDID1   0xDA // Read ID1 (two reads follows)
#define ST7735R_RDID2   0xDB // Read ID2 (two reads follows)
#define ST7735R_RDID2   0xDC // Read ID3 (two reads follows)

extern PROGMEM const unsigned char ST7735R_Init_Sequence[];
void DisplaySendCommandList(const uint8_t *addr);
void DisplayBegin();
void DisplayLine(int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b);
void DisplayDrawMonoSprite(int x, int y, const uint8_t *addr, int width, int height,
                            uint8_t lo, uint8_t hi, uint8_t bgLo, uint8_t bgHi);
void DisplayDrawText(int x, int y, const char *text, uint8_t r, uint8_t g, uint8_t b,
                      uint8_t bgR, uint8_t bgG, uint8_t bgB);
void DisplayDraw565(int x, int y, uint32_t imageDataBlock);

// Adapted from Arduino.cc TFT library:
#define DELAY 0x80
PROGMEM const unsigned char ST7735R_Init_Sequence[] =
{
  9,
  ST7735R_SWRESET, DELAY, 10,
  ST7735R_SLPOUT, DELAY, 100,
  ST7735R_GAMSET, 1, GAMSET_GC2,
  ST7735R_COLMOD, 1 | DELAY, COLMOD_WRITE_16BPP, 20,
  ST7735R_MADCTL, 1, MADCTL_MY | MADCTL_MV | MADCTL_RGB,
  ST7735R_CASET, 4, 0, 0, 0, ST7735R_WIDTH-1,
  ST7735R_RASET, 4, 0, 0, 0, ST7735R_HEIGHT-1,
  ST7735R_NORON, DELAY, 2,
  ST7735R_DISPON, DELAY, 100
};

// Adapted from Arduino.cc TFT library:
void DisplaySendCommandList(const uint8_t *addr)
{
  uint8_t numCommands = pgm_read_byte(addr++);
  while(numCommands--)
  {
    SEND_COMMAND(pgm_read_byte(addr++));
    uint8_t numArgs = pgm_read_byte(addr++);
    uint8_t ms = numArgs & DELAY;
    numArgs &= ~DELAY;
    while(numArgs--) WRITE_SPI_SYNC(pgm_read_byte(addr++));
    if (ms) delay((uint16_t)pgm_read_byte(addr++)*5);
  }
}

// Initializes the ST7735R display. Call this at startup.
void DisplayBegin()
{
  pinMode(ST7735R_CS_PIN, OUTPUT);
  pinMode(ST7735R_RS_PIN, OUTPUT);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  pinMode(ST7735R_RST_PIN, OUTPUT);
  digitalWrite(ST7735R_RST_PIN, HIGH);
  delay(500);
  digitalWrite(ST7735R_RST_PIN, LOW);
  delay(500);
  digitalWrite(ST7735R_RST_PIN, HIGH);
  delay(500);

  DisplayBeginTransaction();
  BEGIN_TFT();
  DisplaySendCommandList(ST7735R_Init_Sequence);
  END_TFT();
  DisplayEndTransaction();
}


// Begins drawing a single rectangular array of pixels one scanline at a time.
// The end coordinaes are inclusive, i.e. the range [x0, x1] X [y0, y1] is drawn.
// Use as follows:
//   DisplayBeginRect(topLeftX, topLeftY, bottomRightX, bottomRightY);
//   for(int y = topLeftY; y <= bottomRightY; ++y)
//      for(int x = topLeftX; x <= bottomRightX; ++x)
//         DisplayPushPixel(r, g, b);
//   DisplayEndDraw();
#define DisplayBeginRect(x0, y0, x1, y1) \
  do { \
    /*BEGIN_TFT();*/ \
    SEND_COMMAND(ST7735R_CASET); /* Column address set */ \
    WRITE_SPI_SYNC(0); /* XSTART[15:8] */ \
    WRITE_SPI_SYNC(x0); /* XSTART[7:0] */ \
    WRITE_SPI_SYNC(0); /* XEND[15:8] */ \
    WRITE_SPI_SYNC(x1); /* XEND[7:0] */ \
    SEND_COMMAND(ST7735R_RASET); \
    WRITE_SPI_SYNC(0); /* YSTART[15:8] */ \
    WRITE_SPI_SYNC(y0); /* YSTART[7:0] */ \
    /* hack: The YEND coordinate is always parked to the bottom of the screen. */ \
    /*WRITE_SPI_SYNC(0);*/ /* YEND[15:8] */ \
    /*WRITE_SPI_SYNC(y1);*/ /* YEND[7:0] */ \
    SEND_COMMAND(ST7735R_RAMWR); /* RAM Write */ \
  } while(0)

// Begins drawing a batch of individual pixels.
// Use as follows:
//   DisplayBeginPixels();
//   for(int i = 0; i < severalTimes; ++i)
//      DisplayPixel(x, y, r, g, b);
//   DisplayEndDraw();
#define DisplayBeginPixels() DisplayBeginRect(0, 0, ST7735R_WIDTH-1, ST7735R_HEIGHT-1)

#define DisplayEndDraw() WAIT_SPI

// Computes the high 8 bits of a 24-bit RGB triplet converted to a R5G6B5 quantity.
#define HI8_RGB24(r,g,b) (((uint8_t)(r) & 0xF8) | ((uint8_t)(g) >> 5))
// Computes the low 8 bits of a 24-bit RGB triplet converted to a R5G6B5 quantity.
#define LO8_RGB24(r,g,b) ((((uint8_t)(g) << 3) & 0xE0) | ((uint8_t)(b) >> 3))

// Plots a single pixel on the screen. Call this function only in between
// a DisplayBeginPixels() and DisplayEndDraw() block.
// x: Horizontal coordinate in the range [0, 159].
// y: Vertical coordinate [0, 127].
// r, g, b: Color components in the range [0, 255] (24-bit color).
#define DisplayPixel(x, y, r, g, b) \
  do { \
    WAIT_SPI; \
    SEND_COMMAND(ST7735R_CASET); \
    WRITE_SPI_SYNC(0); /*XSTART[15:8]*/ \
    WRITE_SPI_SYNC((uint8_t)(x)); /*XSTART[7:0]*/ \
    /* hack: the data sheet states that CASET instruction should be     */ \
    /* followed by four data bytes, two for XSTART and two for XEND,    */ \
    /* practice shows that we can omit sending the two XEND bytes here, */ \
    /* which speeds up performance. We "park" the XEND and YEND to      */ \
    /* bottom right corner of the screen, and only ever adjust top-left */ \
    /* coordinates. It is uncertain if this is allowed by the spec, but */ \
    /* works ok on my tests. If you run into problems, uncomment the    */ \
    /* following two lines.                                             */ \
    /* WRITE_SPI_SYNC(0);*/ /*XEND[15:8]*/ \
    /* WRITE_SPI_SYNC(ST7735R_WIDTH-1);*/ /*XEND[7:0]*/ \
    SEND_COMMAND(ST7735R_RASET); \
    WRITE_SPI_SYNC(0); \
    WRITE_SPI_NOWAIT((uint8_t)(y)); \
    /* hack: same as above. If you run into problems, uncomment below: */ \
    /* NOP; WAIT_SPI; WRITE_SPI_SYNC(0);*/ /*YEND[15:8]*/ \
    /* WRITE_SPI_NOWAIT(ST7735R_HEIGHT-1);*/ /*YEND[7:0]*/ \
    uint8_t hi = HI8_RGB24(r,g,b); \
    WAIT_SPI; \
    SEND_COMMAND(ST7735R_RAMWR); \
    WRITE_SPI_NOWAIT(hi); \
    uint8_t lo = LO8_RGB24(r,g,b); \
    WAIT_SPI; \
    WRITE_SPI_NOWAIT(lo); \
  } while(0)

// Specifies the next pixel in a rectangular grid of pixels. Call this function
// only in between a DisplayBeginRect() and DisplayEndDraw() block.
// r,g,b: Color of the pixel, each value in the range [0, 255], i.e. 24-bit colors.
#define DisplayPushPixel(r,g,b) \
  do { \
    uint8_t hi = HI8_RGB24(r,g,b); \
    WAIT_SPI; \
    WRITE_SPI_NOWAIT(hi); \
    uint8_t lo = LO8_RGB24(r,g,b); \
    WAIT_SPI; \
    WRITE_SPI_NOWAIT(lo); \
  } while(0)

// Specifies the next pixel in a rectangular grid of pixels, in two 8-bit halves of the
// 16-bit RGB565 color. Call this function only in between a DisplayBeginRect()
// and DisplayEndDraw() block.
// This function can be used as a replacement for DisplayPushPixel() wherever
// that function is accepted. This function is a tiny bit faster in the case when
// doing single color fills, in which case one can precompute the color up front.
// i.e. the two code snippets are equivalent:
//
//    DisplayPushPixel(r,g,b);
//
//  vs
//
//    uint8_t lo = LO8_RGB24(r,g,b);
//    uint8_t hi = HI8_RGB24(r,g,b);
//    DisplayPushPixelU16(lo, hi);
#define DisplayPushPixelU16(lo, hi) \
  do { \
    NOP; \
    WAIT_SPI; \
    WRITE_SPI_SYNC(hi); \
    WRITE_SPI_NOWAIT(lo); \
  } while(0)

#define DisplayHLineU16(x0, x1, y, lo, hi) \
  do { \
    DisplayBeginRect(x0, y, x1, y); \
    for(uint8_t i = x0; i <= x1; ++i) \
      DisplayPushPixelU16(lo, hi); \
    WAIT_SPI; \
  } while(0)

#define DisplayHLine(x0, x1, y, r, g, b) \
  do { \
    uint8_t lo = LO8_RGB24(r,g,b); \
    uint8_t hi = HI8_RGB24(r,g,b); \
    DisplayHLineU16(x0, x1, y, lo, hi); \
  } while(0)

#define DisplayVLineU16(x, y0, y1, lo, hi) \
  do { \
    DisplayBeginRect(x, y0, x, y1); \
    for(uint8_t i = (y0); i <= (y1); ++i) \
      DisplayPushPixelU16(lo, hi); \
    WAIT_SPI; \
  } while(0)

#define DisplayVLine(x, y0, y1, r, g, b) \
  do { \
    uint8_t lo = LO8_RGB24(r,g,b); \
    uint8_t hi = HI8_RGB24(r,g,b); \
    DisplayVLineU16(x, y0, y1, lo, hi); \
  } while(0)

#define DisplayFillRect(x0, y0, x1, y1, r, g, b) \
  do { \
    uint8_t lo = LO8_RGB24(r,g,b); \
    uint8_t hi = HI8_RGB24(r,g,b); \
    DisplayBeginRect((x0), (y0), (x1), (y1)); \
    for(uint16_t i = ((y1)-(y0)+1)*((x1)-(x0)+1); i > 0; --i) { \
      /* N.B. unrolling the loop does not help here, since the loop control \
         is already done parallel to waiting for SPI operation to complete. */ \
        DisplayPushPixelU16(lo, hi); \
    } \
    WAIT_SPI; \
  } while(0)

#define DisplayClear(r, g, b) DisplayFillRect(0, 0, ST7735R_WIDTH-1, ST7735R_HEIGHT-1, r, g, b)

void __attribute__((always_inline)) swap_int(int &a, int &b)
{
  int tmp = a;
  a = b;
  b = tmp;
}

void DisplayLine(int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b)
{
    int dx = x1 - x0;
    int dy = y1 - y0;
    int adx = abs(dx);
    int ady = abs(dy);

    uint8_t lo = LO8_RGB24(r,g,b);
    uint8_t hi = HI8_RGB24(r,g,b);

    if (ady < adx) // "Mostly horizontal": Bresenham's algorithm in the variant where x travels faster than y.
    {
        if (x1 < x0)
        {
            swap_int(x0, x1);
            swap_int(y0, y1);
            dx = -dx;
            dy = -dy;
        }
        int ysign = (dy >= 0) ? 1 : -1;

        // General strategy here is the same as with the other graphics functions:
        // interleave SPI commands with computation as much as possible, and only
        // send the minimal number of SPI CASET/RASET commands that are needed
        // needed to traverse the cursor across the line.
        DisplayBeginRect(x0, y0, ST7735R_WIDTH-1, ST7735R_HEIGHT-1);
        int diff = dy >> 1;
        while(x0 <= x1)
        {
            WAIT_SPI;
            WRITE_SPI_NOWAIT(hi);
            diff -= ady;
            ++x0;
            WAIT_SPI;
            WRITE_SPI_NOWAIT(lo);

            if (diff < 0)
            {
                y0 += ysign;
                diff += dx;
                WAIT_SPI;
                // The line is "mostly horizontal", so CASET and RASET end coordinates
                // are parked to bottom right of the screen, and only the start cursor
                // needs to be specified. This exploits the advantage that when the
                // y coordinate of two adjacent pixels doesn't change, we don't need
                // to change CASET/RASET commands either.
                SEND_COMMAND(ST7735R_CASET);
                WRITE_SPI_SYNC(0);
                WRITE_SPI_SYNC(x0);
                SEND_COMMAND(ST7735R_RASET);
                WRITE_SPI_SYNC(0);
                WRITE_SPI_SYNC(y0);
                SEND_COMMAND(ST7735R_RAMWR);
            }
        }
    }
    else // "Mostly vertical": Bresenham's algorithm in the variant where y travels faster than x.
    {
        if (y1 < y0)
        {
        swap_int(x0, x1);
        swap_int(y0, y1);
        dx = -dx;
        dy = -dy;
        }
        int xsign = (dx >= 0) ? 1 : -1;

        DisplayBeginRect(x0, y0, x0, ST7735R_HEIGHT-1);

        int diff = dx >> 1;
        while(y0 <= y1)
        {
            WAIT_SPI;
            WRITE_SPI_NOWAIT(hi);
            diff -= adx;
            ++y0;
            WAIT_SPI;
            WRITE_SPI_NOWAIT(lo);

            if (diff < 0)
            {
                x0 += xsign;
                diff += dy;
                WAIT_SPI;
                // The line is "mostly vertical", so have the CASET be a single pixel
                // window, so that the cursor traverses downwards naturally. YEND is
                // parked to bottom end of the screen. Only need to send CASET/RASET
                // commands again when the x coordinate changes. Overall the
                // "mostly vertical" path is two SPI commands slower per x coord
                // delta compared to the "mostly horizontal" path above.
                SEND_COMMAND(ST7735R_CASET);
                WRITE_SPI_SYNC(0);
                WRITE_SPI_SYNC(x0);
                WRITE_SPI_SYNC(0);
                WRITE_SPI_SYNC(x0);
                SEND_COMMAND(ST7735R_RASET);
                WRITE_SPI_SYNC(0);
                WRITE_SPI_SYNC(y0);
                SEND_COMMAND(ST7735R_RAMWR);
            }
        }
    }
    WAIT_SPI;
}

#define CURSORX(x) \
  SEND_COMMAND(ST7735R_CASET); \
  WRITE_SPI_SYNC(0); \
  WRITE_SPI_SYNC((x)); \
  SEND_COMMAND(ST7735R_RAMWR);

#define CURSORY(y) \
  SEND_COMMAND(ST7735R_RASET); \
  WRITE_SPI_SYNC(0); \
  WRITE_SPI_SYNC((y)); \
  SEND_COMMAND(ST7735R_RAMWR);

#define CURSOR(x, y) \
  SEND_COMMAND(ST7735R_CASET); \
  WRITE_SPI_SYNC(0); \
  WRITE_SPI_SYNC((x)); \
  SEND_COMMAND(ST7735R_RASET); \
  WRITE_SPI_SYNC(0); \
  WRITE_SPI_SYNC((y)); \
  SEND_COMMAND(ST7735R_RAMWR);

#define PUSH_PIXEL(hi, lo) \
  WRITE_SPI_SYNC(hi); \
  WRITE_SPI_SYNC(lo);

void DisplayCircle(int x, int y, uint8_t radius, uint8_t red, uint8_t green, uint8_t blue)
{
    // TODO: x-y clipping.
    uint8_t lo = LO8_RGB24(red, green, blue);
    uint8_t hi = HI8_RGB24(red, green, blue);

    int tx = 0;
    int ty = radius;
    int error = (5 - (radius << 2)) >> 2;

    //  DisplayPixel(x, y - ty, r, g, b);
    DisplayBeginRect(x, y - ty, ST7735R_WIDTH-1, ST7735R_HEIGHT-1);
    PUSH_PIXEL(hi, lo);
    //  DisplayPixel(x, y + ty, r, g, b);
    CURSORY(y + ty);
    PUSH_PIXEL(hi, lo);
    //  DisplayPixel(x - ty, y, r, g, b);
    CURSOR(x - ty, y);
    PUSH_PIXEL(hi, lo);
    //  DisplayPixel(x + ty, y, r, g, b);
    CURSORX(x + ty);
    PUSH_PIXEL(hi, lo);

    while(tx < ty)
    {
        ++tx;
        if (error < 0)
        error += (tx << 1) + 1;
        else
        {
            --ty;
            error += ((tx - ty) << 1) + 1;
        }

        WAIT_SPI;
    //  DisplayPixel(x - tx, y - ty, r, g, b);
        CURSOR(x - tx, y - ty);
        PUSH_PIXEL(hi, lo);
    //  DisplayPixel(x + tx, y - ty, r, g, b);
        CURSORX(x + tx);
        PUSH_PIXEL(hi, lo);
    //  DisplayPixel(x + tx, y + ty, r, g, b);
        CURSORY(y + ty);
        PUSH_PIXEL(hi, lo);
    //  DisplayPixel(x - tx, y + ty, r, g, b);
        CURSORX(x - tx);
        PUSH_PIXEL(hi, lo);

    //  DisplayPixel(x - ty, y - tx, r, g, b);
        CURSOR(x - ty, y - tx);
        PUSH_PIXEL(hi, lo);
    //  DisplayPixel(x + ty, y - tx, r, g, b);
        CURSORX(x + ty);
        PUSH_PIXEL(hi, lo);
    //  DisplayPixel(x + ty, y + tx, r, g, b);
        CURSORY(y + tx);
        PUSH_PIXEL(hi, lo);
    //  DisplayPixel(x - ty, y + tx, r, g, b);
        CURSORX(x - ty);
        DisplayPushPixelU16(lo, hi);
    }
    WAIT_SPI;
}

void DisplayFilledCircle(int x, int y, uint8_t radius, uint8_t red, uint8_t green, uint8_t blue)
{
    // TODO: x-y clipping.
    uint8_t lo = LO8_RGB24(red, green, blue);
    uint8_t hi = HI8_RGB24(red, green, blue);

    int tx = 0;
    int ty = radius;
    int error = (5 - (radius << 2)) >> 2;

    DisplayBeginRect(x - ty, y, ST7735R_WIDTH-1, ST7735R_HEIGHT-1);

    int nty = (ty << 1) + 1;
    for(int i = 0; i < nty; ++i)
    DisplayPushPixelU16(lo, hi);

    while(tx < ty)
    {
        ++tx;
        if (error < 0) {
            error += (tx << 1) + 1;
        } else {
            --ty;
            error += ((tx - ty) << 1) + 1;
        }
        WAIT_SPI;
        CURSOR(x - tx, y - ty);
        int ntx = (tx << 1) + 1;
        for(int i = 0; i < ntx; ++i) {
            DisplayPushPixelU16(lo, hi);
        }
        WAIT_SPI;
        CURSORY(y + ty);
        for(int i = 0; i < ntx; ++i) {
            DisplayPushPixelU16(lo, hi);
        }
        WAIT_SPI;
        int nty = (ty << 1) + 1;
        CURSOR(x - ty, y - tx);
        for(int i = 0; i < nty; ++i) {
            DisplayPushPixelU16(lo, hi);
        }
        WAIT_SPI;
        CURSORY(y + tx);
        for(int i = 0; i < nty; ++i) {
            DisplayPushPixelU16(lo, hi);
        }
    }
    WAIT_SPI;
}

void DisplayDrawMonoSprite(int x, int y, const uint8_t *addr, int width, int height,
                            uint8_t lo, uint8_t hi, uint8_t bgLo, uint8_t bgHi)
{
    DisplayBeginRect(x, y, x + width-1, ST7735R_HEIGHT-1);
    int nBytes = (width*height) >> 3; // Width * height must be divisible by 8!
    while(nBytes-- > 0)
    {
        uint8_t byte = pgm_read_byte(addr++);
        for(uint8_t bit = 1; bit; bit <<= 1)
        {
            if ((byte & bit) != 0) DisplayPushPixelU16(lo, hi);
            else DisplayPushPixelU16(bgLo, bgHi);
        }
    }
    DisplayEndDraw();
}

/* Define default font 'Monaco' */
static const uint8_t monaco_font[] PROGMEM = {
    /*spc*/ 0x00,0x00,0x00,0x00,0x00,
    /* ! */ 0x21,0x84,0x10,0x40,0x00,
    /* " */ 0xa5,0x00,0x00,0x00,0x00,
    /* # */ 0x8c,0x7d,0xf5,0x8d,0x01,
    /* $ */ 0xc4,0x97,0x62,0x38,0x7d,
    /* % */ 0x71,0x26,0xc2,0x74,0x02,
    /* & */ 0xa2,0x14,0xd1,0x92,0x03,
    /* ' */ 0x21,0x00,0x00,0x00,0x00,
    /* ( */ 0x44,0x84,0x10,0x82,0x20,
    /* ) */ 0x41,0x10,0x42,0x88,0x08,
    /* * */ 0xe2,0x1c,0x01,0x00,0x00,
    /* + */ 0x84,0x7c,0x42,0x00,0x00,
    /* , */ 0x21,0x04,0x00,0x00,0x00,
    /* - */ 0x07,0x00,0x00,0x00,0x00,
    /* . */ 0x01,0x00,0x00,0x00,0x00,
    /* / */ 0x08,0x11,0x21,0x02,0x00,
    /* 0 */ 0xa6,0xb5,0xb5,0x96,0x01,
    /* 1 */ 0x62,0x08,0x21,0xc4,0x01,
    /* 2 */ 0x07,0x21,0x22,0xc2,0x03,
    /* 3 */ 0x07,0x21,0x83,0xd0,0x01,
    /* 4 */ 0x88,0xa9,0xf4,0x11,0x02,
    /* 5 */ 0x2f,0x1c,0x84,0xd0,0x01,
    /* 6 */ 0x26,0x94,0xb5,0x96,0x01,
    /* 7 */ 0x0f,0x21,0x62,0x84,0x00,
    /* 8 */ 0x2e,0x25,0x93,0xd2,0x01,
    /* 9 */ 0x26,0xa5,0xe4,0x90,0x01,
    /* : */ 0x01,0x00,0x10,0x00,0x00,
    /* ; */ 0x01,0x00,0x10,0x42,0x00,
    /* < */ 0x90,0x0d,0x06,0x01,0x00,
    /* = */ 0x0f,0x3c,0x00,0x00,0x00,
    /* > */ 0xc1,0x60,0x13,0x00,0x00,
    /* ? */ 0x87,0x10,0x21,0x80,0x00,
    /* @ */ 0x2e,0xf6,0xbd,0x9b,0x01,
    /* A */ 0xc6,0x18,0xf3,0x5e,0x02,
    /* B */ 0xa3,0x94,0x51,0xca,0x00,
    /* C */ 0x2e,0x84,0x10,0x82,0x03,
    /* D */ 0x27,0xa5,0x94,0xd2,0x01,
    /* E */ 0x2f,0x84,0x17,0xc2,0x03,
    /* F */ 0x2f,0x84,0x17,0x42,0x00,
    /* G */ 0x2e,0x84,0x96,0x92,0x03,
    /* H */ 0x29,0xa5,0x97,0x52,0x02,
    /* I */ 0x47,0x08,0x21,0xc4,0x01,
    /* J */ 0x0e,0x21,0x84,0xd0,0x01,
    /* K */ 0x29,0x95,0x51,0x4a,0x02,
    /* L */ 0x21,0x84,0x10,0xc2,0x01,
    /* M */ 0x7b,0xef,0x7d,0x6b,0x04,
    /* N */ 0x69,0xad,0xd6,0x5a,0x02,
    /* O */ 0x26,0xa5,0x94,0x92,0x01,
    /* P */ 0x27,0xa5,0x74,0x42,0x00,
    /* Q */ 0x26,0xa5,0x94,0x92,0x21,
    /* R */ 0xa3,0x94,0x31,0x4a,0x01,
    /* S */ 0x27,0x04,0x41,0xc8,0x01,
    /* T */ 0x9f,0x10,0x42,0x08,0x01,
    /* U */ 0x29,0xa5,0x94,0x92,0x01,
    /* V */ 0xa5,0x94,0x52,0x84,0x00,
    /* W */ 0xb5,0xde,0xad,0x94,0x02,
    /* X */ 0x4a,0x11,0x42,0x94,0x02,
    /* Y */ 0xa5,0x14,0x21,0x84,0x00,
    /* Z */ 0x0f,0x11,0x21,0xc2,0x03,
    /* [ */ 0x27,0x84,0x10,0x42,0x38,
    /* \ */ 0x21,0x08,0x41,0x10,0x00,
    /* ] */ 0x87,0x10,0x42,0x08,0x39,
    /* ^ */ 0xc2,0x98,0x04,0x00,0x00,
    /* _ */ 0x0f,0x00,0x00,0x00,0x00,
    /* ` */ 0x02,0x00,0x00,0x00,0x00,
    /* a */ 0x3e,0xc6,0xe8,0x01,0x00,
    /* b */ 0x21,0xbc,0x18,0xe3,0x03,
    /* c */ 0x2e,0x84,0xe0,0x00,0x00,
    /* d */ 0x08,0xb9,0x94,0x92,0x03,
    /* e */ 0x26,0xbd,0xe0,0x00,0x00,
    /* f */ 0x5c,0x3c,0x21,0x84,0x00,
    /* g */ 0x2e,0xa5,0xe4,0xd0,0x01,
    /* h */ 0x21,0x9c,0x94,0x52,0x02,
    /* i */ 0x02,0x0c,0x21,0x84,0x01,
    /* j */ 0x04,0x1c,0x42,0x08,0x21,
    /* k */ 0x21,0x94,0x31,0x4a,0x02,
    /* l */ 0x43,0x08,0x21,0x84,0x01,
    /* m */ 0xbf,0xd6,0x5a,0x01,0x00,
    /* n */ 0x27,0xa5,0x94,0x00,0x00,
    /* o */ 0x26,0xa5,0x64,0x00,0x00,
    /* p */ 0x2f,0xc6,0xf8,0x42,0x00,
    /* q */ 0x2e,0xa5,0xe4,0x10,0x02,
    /* r */ 0xa7,0x84,0x10,0x00,0x00,
    /* s */ 0x2e,0xb9,0x74,0x00,0x00,
    /* t */ 0x42,0x3c,0x21,0x18,0x00,
    /* u */ 0x29,0xa5,0xe4,0x00,0x00,
    /* v */ 0xa5,0x94,0x23,0x00,0x00,
    /* w */ 0xf5,0xde,0xa7,0x00,0x00,
    /* x */ 0xca,0x10,0xa5,0x00,0x00,
    /* y */ 0x4a,0x29,0x46,0xc8,0x00,
    /* z */ 0x8f,0x88,0xf0,0x00,0x00,
    /* { */ 0x46,0x08,0x11,0x84,0x30,
    /* | */ 0x21,0x84,0x10,0x02,0x00,
    /* } */ 0x43,0x08,0x41,0x84,0x18,
    /* ~ */ 0x1f,0x00,0x00,0x00,0x00,
};

static const uint8_t monaco_height_adjust[] PROGMEM = {
   6, -1, -1, -1, -2, -1, -1, -1, -1, -1, -1, 1, 5, 3, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
   -1, 1, 1, 1, 2, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 6, 0, 1, -1, 1, -1, 1, -1, 1, -1, -1, -1,
   -1, -1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, -1, -1, -1, 4
};

void DisplayDrawText(int x, int y, /* Position */
                      const char *text, /* Text to draw */
                      uint8_t r, uint8_t g, uint8_t b, /* Color */
                      uint8_t bgR, uint8_t bgG, uint8_t bgB) /* Background color */
{
    uint8_t ch = *text;
    uint8_t lo = LO8_RGB24(r,g,b);
    uint8_t hi = HI8_RGB24(r,g,b);
    uint8_t bgLo = LO8_RGB24(bgR,bgG,bgB);
    uint8_t bgHi = HI8_RGB24(bgR,bgG,bgB);
    while(ch)
    {
        int height_adjust = pgm_read_byte(monaco_height_adjust + ch-32);
        DisplayDrawMonoSprite(x, y + height_adjust, monaco_font + (ch-32) * 5, 5, 8, lo, hi, bgLo, bgHi);
        ++text;
        ch = *text;
        x += 6;
    }
}