// tinyHVSP
//
// Stand-alone AVR High-Voltage Serial Programmer for ATtiny13/25/45/85
// using an ATtiny84 for bit-banging the protocol according to the
// respective datasheets.
//
// Core:          ATtinyCore (https://github.com/SpenceKonde/ATTinyCore)
// Board:         ATtiny24/44/84(a) (No bootloader)
// Chip:          ATtiny24(a) or 44(a) or 84(a) (depending on your chip)
// Clock:         8 MHz (internal)
// Millis/Micros: disabled
// Leave the rest on default settings. Don't forget to "Burn bootloader" !
//
// based on the work of Jeff Keyzer
// http://mightyohm.com
// and Paul Willoughby
// http://www.rickety.us/2010/03/arduino-avr-high-voltage-serial-programmer/
//
// The I²C OLED implementation is based on TinyOLEDdemo
// https://github.com/wagiminator/ATtiny13-TinyOLEDdemo
//
// OLED font was adapted from Neven Boyanov and Stephen Denne
// https://github.com/datacute/Tiny4kOLED
//
// 2019 by Stefan Wagner 
// Project Files (EasyEDA): https://easyeda.com/wagiminator
// Project Files (Github):  https://github.com/wagiminator
// License: http://creativecommons.org/licenses/by-sa/3.0/


// Libraries
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

// Pin assignments
#define RST_PIN   PA0     // 12V !RESET                       Pin 1 of target device
#define SCI_PIN   PA1     // Serial Clock Input (SCI)         Pin 2 of target device
#define SDO_PIN   PA2     // Serial Data Output (SDO)         Pin 7 of target device
#define SII_PIN   PA5     // Serial Instruction Input (SII)   Pin 6 of target device
#define SDI_PIN   PA7     // Serial Data Input (SDI)          Pin 5 of target device
#define VCC_PIN   PA3     // Target VCC                       Pin 8 of target device
#define I2C_SCL   PA4     // I2C Serial Clock (SCK)
#define I2C_SDA   PA6     // I2C Serial Data (SDA)
#define BUTTON    PB2     // OK-Button

// Global variables
uint8_t  inLFUSE, inHFUSE, inEFUSE;           // for reading fuses
uint8_t  outLFUSE, outHFUSE, outEFUSE;        // for writing fuses
uint8_t  inLOCK;                              // for reading lock bits
uint16_t signature;                           // for reading signature

// -----------------------------------------------------------------------------
// I2C Implementation
// -----------------------------------------------------------------------------

// I2C macros
#define I2C_SDA_HIGH()  DDRA &= ~(1<<I2C_SDA) // release SDA   -> pulled HIGH by resistor
#define I2C_SDA_LOW()   DDRA |=  (1<<I2C_SDA) // SDA as output -> pulled LOW  by MCU
#define I2C_SCL_HIGH()  DDRA &= ~(1<<I2C_SCL) // release SCL   -> pulled HIGH by resistor
#define I2C_SCL_LOW()   DDRA |=  (1<<I2C_SCL) // SCL as output -> pulled LOW  by MCU
#define I2C_DELAY()     asm("lpm")            // delay 3 clock cycles
#define I2C_CLOCKOUT()  I2C_SCL_HIGH();I2C_DELAY();I2C_SCL_LOW()  // clock out

// I2C init function
void I2C_init(void) {
  DDRA  &= ~((1<<I2C_SDA)|(1<<I2C_SCL));  // pins as input (HIGH-Z) -> lines released
  PORTA &= ~((1<<I2C_SDA)|(1<<I2C_SCL));  // should be LOW when as ouput
}

// I2C transmit one data byte to the slave, ignore ACK bit, no clock stretching allowed
void I2C_write(uint8_t data) {
  for(uint8_t i = 8; i; i--, data<<=1) {  // transmit 8 bits, MSB first
    (data & 0x80) ? (I2C_SDA_HIGH()) : (I2C_SDA_LOW());  // SDA HIGH if bit is 1
    I2C_CLOCKOUT();                       // clock out -> slave reads the bit
  }
  I2C_DELAY();                            // delay 3 clock cycles
  I2C_SDA_HIGH();                         // release SDA for ACK bit of slave
  I2C_CLOCKOUT();                         // 9th clock pulse is for the ignored ACK bit
}

// I2C start transmission
void I2C_start(uint8_t addr) {
  I2C_SDA_LOW();                          // start condition: SDA goes LOW first
  I2C_SCL_LOW();                          // start condition: SCL goes LOW second
  I2C_write(addr);                        // send slave address
}

// I2C stop transmission
void I2C_stop(void) {
  I2C_SDA_LOW();                          // prepare SDA for LOW to HIGH transition
  I2C_SCL_HIGH();                         // stop condition: SCL goes HIGH first
  I2C_SDA_HIGH();                         // stop condition: SDA goes HIGH second
}

// -----------------------------------------------------------------------------
// OLED Implementation
// -----------------------------------------------------------------------------

// OLED definitions
#define OLED_ADDR       0x78              // OLED write address
#define OLED_CMD_MODE   0x00              // set command mode
#define OLED_DAT_MODE   0x40              // set data mode
#define OLED_INIT_LEN   9                 // length of init command array

// OLED 5x8 pixels character set
const uint8_t OLED_FONT[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2F, 0x00, 0x00, 0x00, 0x07, 0x00, 0x07, 0x00,
  0x14, 0x7F, 0x14, 0x7F, 0x14, 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x23, 0x13, 0x08, 0x64, 0x62,
  0x36, 0x49, 0x55, 0x22, 0x50, 0x00, 0x05, 0x03, 0x00, 0x00, 0x00, 0x1C, 0x22, 0x41, 0x00,
  0x00, 0x41, 0x22, 0x1C, 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14, 0x08, 0x08, 0x3E, 0x08, 0x08,
  0x00, 0x00, 0xA0, 0x60, 0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x60, 0x60, 0x00, 0x00,
  0x20, 0x10, 0x08, 0x04, 0x02, 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x42, 0x7F, 0x40, 0x00,
  0x42, 0x61, 0x51, 0x49, 0x46, 0x21, 0x41, 0x45, 0x4B, 0x31, 0x18, 0x14, 0x12, 0x7F, 0x10,
  0x27, 0x45, 0x45, 0x45, 0x39, 0x3C, 0x4A, 0x49, 0x49, 0x30, 0x01, 0x71, 0x09, 0x05, 0x03,
  0x36, 0x49, 0x49, 0x49, 0x36, 0x06, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x36, 0x36, 0x00, 0x00,
  0x00, 0x56, 0x36, 0x00, 0x00, 0x08, 0x14, 0x22, 0x41, 0x00, 0x14, 0x14, 0x14, 0x14, 0x14,
  0x00, 0x41, 0x22, 0x14, 0x08, 0x02, 0x01, 0x51, 0x09, 0x06, 0x32, 0x49, 0x59, 0x51, 0x3E,
  0x7C, 0x12, 0x11, 0x12, 0x7C, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x3E, 0x41, 0x41, 0x41, 0x22,
  0x7F, 0x41, 0x41, 0x22, 0x1C, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x7F, 0x09, 0x09, 0x09, 0x01,
  0x3E, 0x41, 0x49, 0x49, 0x7A, 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x41, 0x7F, 0x41, 0x00,
  0x20, 0x40, 0x41, 0x3F, 0x01, 0x7F, 0x08, 0x14, 0x22, 0x41, 0x7F, 0x40, 0x40, 0x40, 0x40,
  0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x3E, 0x41, 0x41, 0x41, 0x3E,
  0x7F, 0x09, 0x09, 0x09, 0x06, 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x7F, 0x09, 0x19, 0x29, 0x46,
  0x46, 0x49, 0x49, 0x49, 0x31, 0x01, 0x01, 0x7F, 0x01, 0x01, 0x3F, 0x40, 0x40, 0x40, 0x3F,
  0x1F, 0x20, 0x40, 0x20, 0x1F, 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x63, 0x14, 0x08, 0x14, 0x63,
  0x07, 0x08, 0x70, 0x08, 0x07, 0x61, 0x51, 0x49, 0x45, 0x43, 0x00, 0x7F, 0x41, 0x41, 0x00,
  0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x41, 0x41, 0x7F, 0x00, 0x04, 0x02, 0x01, 0x02, 0x04,
  0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x01, 0x02, 0x04, 0x00, 0x20, 0x54, 0x54, 0x54, 0x78,
  0x7F, 0x48, 0x44, 0x44, 0x38, 0x38, 0x44, 0x44, 0x44, 0x20, 0x38, 0x44, 0x44, 0x48, 0x7F,
  0x38, 0x54, 0x54, 0x54, 0x18, 0x08, 0x7E, 0x09, 0x01, 0x02, 0x18, 0xA4, 0xA4, 0xA4, 0x7C,
  0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, 0x44, 0x7D, 0x40, 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00,
  0x7F, 0x10, 0x28, 0x44, 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78,
  0x7C, 0x08, 0x04, 0x04, 0x78, 0x38, 0x44, 0x44, 0x44, 0x38, 0xFC, 0x24, 0x24, 0x24, 0x18,
  0x18, 0x24, 0x24, 0x18, 0xFC, 0x7C, 0x08, 0x04, 0x04, 0x08, 0x48, 0x54, 0x54, 0x54, 0x20,
  0x04, 0x3F, 0x44, 0x40, 0x20, 0x3C, 0x40, 0x40, 0x20, 0x7C, 0x1C, 0x20, 0x40, 0x20, 0x1C,
  0x3C, 0x40, 0x30, 0x40, 0x3C, 0x44, 0x28, 0x10, 0x28, 0x44, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C,
  0x44, 0x64, 0x54, 0x4C, 0x44, 0x08, 0x36, 0x41, 0x41, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00,
  0x00, 0x41, 0x41, 0x36, 0x08, 0x08, 0x04, 0x08, 0x10, 0x08, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

// OLED init settings
const uint8_t OLED_INIT_CMD[] PROGMEM = {
  0xC8, 0xA1,   // flip screen
  0xA8, 0x1F,   // set multiplex ratio
  0xDA, 0x02,   // set com pins hardware configuration
  0x8D, 0x14,   // set DC-DC enable
  0xAF          // display on
};

// OLED variables
uint8_t OLED_x, OLED_y;                   // current cursor position

// OLED init function
void OLED_init(void) {
  I2C_init();                             // initialize I2C first
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_CMD_MODE);               // set command mode
  for (uint8_t i = 0; i < OLED_INIT_LEN; i++)
    I2C_write(pgm_read_byte(&OLED_INIT_CMD[i])); // send the command bytes
  I2C_stop();                             // stop transmission
}

// OLED set the cursor
void OLED_setCursor(uint8_t xpos, uint8_t ypos) {
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_CMD_MODE);               // set command mode
  I2C_write(xpos & 0x0F);                 // set low nibble of start column
  I2C_write(0x10 | (xpos >> 4));          // set high nibble of start column
  I2C_write(0xB0 | (ypos & 0x07));        // set start page
  I2C_stop();                             // stop transmission
  OLED_x = xpos; OLED_y = ypos;           // set the cursor variables
}

// OLED clear line
void OLED_clearLine(uint8_t line) {
  OLED_setCursor(0, line);                // set cursor to line start
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  for(uint8_t i=128; i; i--) I2C_write(0x00); // clear the line
  I2C_stop();                             // stop transmission
}

// OLED clear screen
void OLED_clearScreen(void) {
  for(uint8_t i=0; i<4; i++)              // 4 lines
    OLED_clearLine(i);                    // clear line
}

// OLED print a single character
void OLED_printChar(char c) {
  uint16_t ptr = c - 32;                  // character pointer
  ptr += ptr << 2;                        // -> ptr = (ch - 32) * 5;
  I2C_write(0x00);                        // write space between characters
  for (uint8_t i=5 ; i; i--) I2C_write(pgm_read_byte(&OLED_FONT[ptr++]));
  OLED_x += 6;                            // update cursor
  if (OLED_x > 122) {                     // line end ?
    I2C_stop();                           // stop data transmission
    OLED_setCursor(0,++OLED_y);           // set next line start
    I2C_start(OLED_ADDR);                 // start transmission to OLED
    I2C_write(OLED_DAT_MODE);             // set data mode
  }
}

// OLED print a string from program memory
void OLED_printPrg(const char* p) {
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  char ch = pgm_read_byte(p);             // read first character from program memory
  while (ch) {                            // repeat until string terminator
    OLED_printChar(ch);                   // print character on OLED
    ch = pgm_read_byte(++p);              // read next character
  }
  I2C_stop();                             // stop transmission
}

// OLED convert byte nibble into hex character and prints it
void OLED_printNibble(uint8_t nibble) {
  char c;
  if (nibble <= 9)  c = '0' + nibble;
  else              c = 'A' + nibble - 10;
  OLED_printChar(c);
}

// OLED print byte as hex
void OLED_printHex(uint8_t value) {
  I2C_start(OLED_ADDR);                   // start transmission to OLED
  I2C_write(OLED_DAT_MODE);               // set data mode
  OLED_printNibble(value >> 4);           // print high nibble
  OLED_printNibble(value & 0x0F);         // print low nibble
  I2C_stop();                             // stop transmission
}

// -----------------------------------------------------------------------------
// High-Voltage Serial Programmer Implementation
// -----------------------------------------------------------------------------

// Desired fuse configuration (defaults) for ATtiny13
#define T13_LFUSE         0x6A
#define T13_HFUSE         0xFF

// Desired fuse configuration (defaults) for ATtiny25/45/85
#define Tx5_LFUSE         0x62
#define Tx5_HFUSE         0xDF

// Signatures
#define T13_SIG           0x9007
#define T25_SIG           0x9108
#define T45_SIG           0x9206
#define T85_SIG           0x930B

// HVSP macros
#define HVSP_RST_12V()    PORTA &= ~(1<<RST_PIN)
#define HVSP_RST_LOW()    PORTA |=  (1<<RST_PIN)
#define HVSP_VCC_ON()     PORTA |=  (1<<VCC_PIN)
#define HVSP_VCC_OFF()    PORTA &= ~(1<<VCC_PIN)
#define HVSP_SCI_HIGH()   PORTA |=  (1<<SCI_PIN)
#define HVSP_SCI_LOW()    PORTA &= ~(1<<SCI_PIN)
#define HVSP_SII_HIGH()   PORTA |=  (1<<SII_PIN)
#define HVSP_SII_LOW()    PORTA &= ~(1<<SII_PIN)
#define HVSP_SDI_HIGH()   PORTA |=  (1<<SDI_PIN)
#define HVSP_SDI_LOW()    PORTA &= ~(1<<SDI_PIN)
#define HVSP_SDO_REL()    DDRA  &= ~(1<<SDO_PIN)
#define HVSP_SDO_BIT      (PINA &   (1<<SDO_PIN))
#define HVSP_CLOCKOUT()   {HVSP_SCI_HIGH(); HVSP_SCI_LOW();}

// HVSP set up all control lines
void HVSP_init(void) {
  PORTA &= ~((1<<SCI_PIN) | (1<<SDO_PIN) | (1<<SII_PIN) | (1<<SDI_PIN) | (1<<VCC_PIN));
  PORTA |=   (1<<RST_PIN);
  DDRA  |=   (1<<SCI_PIN) | (1<<SDO_PIN) | (1<<SII_PIN) | (1<<SDI_PIN) | (1<<VCC_PIN) | (1<<RST_PIN);
}

// HVSP release all control lines (except RST)
void HVSP_release(void) {
  PORTA &= ~((1<<SCI_PIN) | (1<<SDO_PIN) | (1<<SII_PIN) | (1<<SDI_PIN) | (1<<VCC_PIN));
  DDRA  &= ~((1<<SCI_PIN) | (1<<SDO_PIN) | (1<<SII_PIN) | (1<<SDI_PIN) | (1<<VCC_PIN));
}

// HVSP enter the programming mode
void HVSP_enterProgMode(void) {
  HVSP_init();                            // initialize control lines
  HVSP_VCC_ON();                          // apply VCC to target
  _delay_us(20);                          // wait 20us
  HVSP_RST_12V();                         // apply 12V to RESET pin of target
  _delay_us(10);                          // wait 10us
  HVSP_SDO_REL();                         // release SDO line
  _delay_us(300);                         // delay 300us
}

// HVSP exit the programming mode
void HVSP_exitProgMode(void) {
  HVSP_SCI_LOW();                         // set SCI line LOW
  HVSP_RST_LOW();                         // RESET to 0V
  HVSP_VCC_OFF();                         // power down the target
  HVSP_release();                         // release all control lines
}

// HVSP send instruction and receive reply
uint8_t HVSP_sendInstr(uint8_t SDI_BYTE, uint8_t SII_BYTE) {
  uint8_t SDO_BYTE = 0;                   // received byte from target

  // wait until SDO_PIN goes high (target ready) or 10ms time-out
  for(uint8_t i=10; i; i--) {             // 10 x 1ms
    if(HVSP_SDO_BIT) break;               // check SDO line
    _delay_ms(1);                         // delay 1ms
  }
       
  // send start bit (SDI/SII = '0')
  HVSP_SDI_LOW();                         // SDI = '0'
  HVSP_SII_LOW();                         // SII = '0'
  HVSP_CLOCKOUT();                        // SCI HIGH, SCI LOW

  // send instruction bytes, MSB first; receive reply
  for(uint8_t i=8; i; i--) {
    (SDI_BYTE & 0x80) ? (HVSP_SDI_HIGH()) : (HVSP_SDI_LOW());
    (SII_BYTE & 0x80) ? (HVSP_SII_HIGH()) : (HVSP_SII_LOW());
    SDI_BYTE <<= 1;
    SII_BYTE <<= 1;
    SDO_BYTE <<= 1;
    if(HVSP_SDO_BIT) SDO_BYTE |= 1;
    HVSP_CLOCKOUT();
  }
      
  // send end bits (two times SDI/SII = '0')
  HVSP_SDI_LOW();                         // SDI = '0'
  HVSP_SII_LOW();                         // SII = '0'
  HVSP_CLOCKOUT();                        // SCI HIGH, SCI LOW
  HVSP_CLOCKOUT();                        // SCI HIGH, SCI LOW
        
  return SDO_BYTE;                        // return read SDO byte
}

// HVSP read signature of target device
void HVSP_readSignature(void) {
  HVSP_sendInstr(0x08, 0x4C);             // Instr1: read sig/calib command
  HVSP_sendInstr(0x01, 0x0C);             // Instr2: select signature byte 1
  HVSP_sendInstr(0x00, 0x68);             // Instr3: signature, not calibration
  signature = HVSP_sendInstr(0x00, 0x6C); // Instr4: read the signature byte
  signature <<= 8;                        // shift left 8 bits

  HVSP_sendInstr(0x02, 0x0C);             // Instr2: select signature byte 2
  HVSP_sendInstr(0x00, 0x68);             // Instr3: signature, not calibration
  signature |= HVSP_sendInstr(0x00, 0x6C);// Instr4: read the signature byte
}

// HVSP read lock bits
void HVSP_readLock(void) {
  HVSP_sendInstr(0x04, 0x4C);             // Instr1: read fuses/lock command
  HVSP_sendInstr(0x00, 0x78);             // Instr2: select lock bits
  inLOCK  = HVSP_sendInstr(0x00, 0x7C);   // Instr3: read lock bits
  inLOCK &= 0x03;                         // mask the lock bits
}

// HVSP read current fuse settings from target device
void HVSP_readFuses(void) {
  HVSP_sendInstr(0x04, 0x4C);             // Instr1: read fuses/lock command
  HVSP_sendInstr(0x00, 0x68);             // Instr2: select low fuse
  inLFUSE = HVSP_sendInstr(0x00, 0x6C);   // Instr3: read low fuse

  HVSP_sendInstr(0x00, 0x7A);             // Instr2: select high fuse
  inHFUSE = HVSP_sendInstr(0x00, 0x7E);   // Instr3: read high fuse

  HVSP_sendInstr(0x00, 0x6A);             // Instr2: select extended fuse
  inEFUSE = HVSP_sendInstr(0x00, 0x6E);   // Instr3: read extended fuse
}

// HVSP write fuse settings to target device
void HVSP_writeFuses(void) {
  HVSP_sendInstr(0x40,  0x4C);            // Instr1: write fuses/lock command
  HVSP_sendInstr(outLFUSE, 0x2C);         // Instr2: write low fuse
  HVSP_sendInstr(0x00,  0x64);            // Instr3: select low fuse
  HVSP_sendInstr(0x00,  0x6C);            // Instr4: select low fuse

  HVSP_sendInstr(outHFUSE, 0x2C);         // Instr2: write high fuse
  HVSP_sendInstr(0x00,  0x74);            // Instr3: select high fuse
  HVSP_sendInstr(0x00,  0x7C);            // Instr4: select high fuse

  HVSP_sendInstr(outEFUSE & 0x01, 0x2C);  // Instr2: write extended fuse
  HVSP_sendInstr(0x00,  0x66);            // Instr3: select extended fuse
  HVSP_sendInstr(0x00,  0x6E);            // Instr4: select extended fuse

  while (!HVSP_SDO_BIT);                  // wait for write cycle to finish
}

// HVSP perform chip erase
void HVSP_eraseChip(void) {
  HVSP_sendInstr(0x80, 0x4C);             // Instr1: chip erase command
  HVSP_sendInstr(0x00, 0x64);             // Instr2
  HVSP_sendInstr(0x00, 0x6C);             // Instr3
  while (!HVSP_SDO_BIT);                  // wait for the chip erase cycle to finish
  HVSP_sendInstr(0x00, 0x4C);             // no operation command to finish chip erase
}

// -----------------------------------------------------------------------------
// Additional Functions
// -----------------------------------------------------------------------------

// Text strings stored in program memory
const char CurrentFuseStr[] PROGMEM = "Current fuse settings";
const char LowStr[] PROGMEM =         "l: ";
const char HighStr[] PROGMEM =        " - h: ";
const char ExtendedStr[] PROGMEM =    " - e: ";

// Print current fuse settings on the OLED
void printFuses() {
  OLED_setCursor(0,1); OLED_printPrg(CurrentFuseStr);
  OLED_printPrg(LowStr); OLED_printHex(inLFUSE);
  OLED_printPrg(HighStr); OLED_printHex(inHFUSE);
  OLED_printPrg(ExtendedStr); OLED_printHex(inEFUSE);
}

// Wait until OK-Button was pressed
void waitButton() {
  while (~PINB & (1<<BUTTON));            // wait for button released
  _delay_ms(10);                          // debounce
  while ( PINB & (1<<BUTTON));            // wait for button pressed
}

// -----------------------------------------------------------------------------
// Main Function
// -----------------------------------------------------------------------------

// Text strings stored in program memory
const char TitleScreen[] PROGMEM =
  "Tiny HVSP Version 1.0"
  "Insert  ATtiny  into "
  "the socket and press "
  "OK-Button to continue";
const char ErrorScreen[] PROGMEM =
  "Check correct place- "
  "ment of the chip and "
  "press <OK> to retry. ";
const char EraseScreen[] PROGMEM = 
  "Error burning fuses !"
  "Press <OK> to perform"
  "a chip erase and     "
  "retry burning fuses. ";
const char DetectedStr[] PROGMEM =  "Detected:    ";
const char ATtinyStr[] PROGMEM =    "ATtiny";
const char ErrorStr[] PROGMEM =     " Error !";
const char DefaultStr[] PROGMEM =   "<OK> to burn defaults";
const char BurnedStr[] PROGMEM =    "Fuses burned         ";
const char QuitStr[] PROGMEM =      "Press <OK> to quit.  ";

int main(void) {
  // Setup
  DDRA  |= (1<<RST_PIN);                  // RST pin as ouput
  PORTA |= (1<<RST_PIN);                  // RST HIGH -> shut off 12V
  PORTB |= (1<<BUTTON);                   // pullup on button pin
  outEFUSE = 0xFF;                        // extended fuse default
  OLED_init();                            // setup I2C OLED
  OLED_clearScreen();                     // clear screen
  
  // Loop
  while(1) {
    // Print title screen    
    OLED_setCursor(0,0);
    OLED_printPrg(TitleScreen);
    
    // Start detection of device and repeat until positive detection
    uint8_t chipDetected = 0;             // assume negative detection by now
    while (!chipDetected) {               // repeat until a valid chip was detected
      waitButton();                       // wait for OK-Button pressed

      chipDetected = 1;                   // assume positive detection by now
      HVSP_enterProgMode();               // start programming mode
      HVSP_readSignature();               // read device signature

      // Prepare to show information on OLED
      OLED_setCursor(0,0);
      OLED_printPrg(DetectedStr);

      // Make settings depending on detected device
      switch (signature) {
        case T13_SIG: OLED_printPrg(ATtinyStr); OLED_printHex(0x13);
                      outLFUSE=T13_LFUSE; outHFUSE=T13_HFUSE; break;
        case T25_SIG: OLED_printPrg(ATtinyStr); OLED_printHex(0x25);
                      outLFUSE=Tx5_LFUSE; outHFUSE=Tx5_HFUSE; break;
        case T45_SIG: OLED_printPrg(ATtinyStr); OLED_printHex(0x45);
                      outLFUSE=Tx5_LFUSE; outHFUSE=Tx5_HFUSE; break;
        case T85_SIG: OLED_printPrg(ATtinyStr); OLED_printHex(0x85);
                      outLFUSE=Tx5_LFUSE; outHFUSE=Tx5_HFUSE; break;
        default:      OLED_printPrg(ErrorStr); OLED_setCursor(0,1);
                      OLED_printPrg(ErrorScreen); HVSP_exitProgMode();
                      chipDetected = 0; break;
      }
    }

    // Read and show detected fuse settings on OLED
    HVSP_readFuses();                     // read current fuse setting
    printFuses();                         // show detected fuse settings
    OLED_printPrg(DefaultStr);

    // Write and read back fuses
    waitButton();                         // wait for OK-Button pressed
    HVSP_writeFuses();                    // write default fuse settings
    HVSP_readFuses();                     // read back fuse settings

    // Check if fuses were burned correctly
    if( (inLFUSE != outLFUSE) | (inHFUSE != outHFUSE) ) {
      OLED_setCursor(0,0);
      OLED_printPrg(EraseScreen);
      waitButton();                       // wait for OK-Button pressed
      HVSP_eraseChip();                   // perform chip erase
      HVSP_writeFuses();                  // write default fuse settings
      HVSP_readFuses();                   // read back fuse settings
    }

    HVSP_exitProgMode();                  // exit programming mode

    // Write information on OLED
    OLED_setCursor(0,0);
    OLED_printPrg(BurnedStr);
    printFuses();
    OLED_printPrg(QuitStr);
    waitButton();                         // wait for OK-Button pressed 
  }
}
