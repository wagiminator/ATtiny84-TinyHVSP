
// tinyHVSP
//
// Stand-alone AVR High-Voltage Serial Programmer for ATtiny13/25/45/85
// using an ATtiny84 for bit-banging the protocol according to the
// respective datasheets.
//
//
// Clockspeed 8 MHz internal.
//
// 2019 by Stefan Wagner
//
// based on the work of Jeff Keyzer (http://mightyohm.com)
// and Paul Willoughby (http://www.rickety.us/2010/03/arduino-avr-high-voltage-serial-programmer/)



// Libraries
#include <Tiny4kOLED.h>
#include <avr/pgmspace.h>

// Pin assignments
#define  RST_PIN  0    // Connect via 12V level shifter to !RESET   Pin 1 of target device
#define  SCI_PIN  1    // Connect to Serial Clock Input (SCI)       Pin 2 of target device
#define  SDO_PIN  2    // Connect to Serial Data Output (SDO)       Pin 7 of target device
#define  SII_PIN  5    // Connect to Serial Instruction Input (SII) Pin 6 of target device
#define  SDI_PIN  7    // Connect to Serial Data Input (SDI)        Pin 5 of target device
#define  VCC_PIN  3    // Connect to VCC                            Pin 8 of target device
#define  BUTTON   8    // OK-Button

// Desired fuse configuration (defaults) for ATtiny13
#define  T13_HFUSE  0xFF
#define  T13_LFUSE  0x6A

// Desired fuse configuration (defaults) for ATtiny25/45/85
#define  Tx5_HFUSE  0xDF
#define  Tx5_LFUSE  0x62

// Signatures
#define T13     0x1E9007
#define T25     0x1E9108
#define T45     0x1E9206
#define T85     0x1E930B

// Text strings stored in program memory
const char TitleScreen[]  PROGMEM = 
  "Tiny HVSP Version 0.5"
  "Insert ATtiny into   "
  "the socket and press "
  "OK-Button to continue";
const char ErrorScreen[]  PROGMEM = 
  "Check correct place- "
  "ment of the chip and "
  "press <OK> to retry. ";
const char EraseScreen[]  PROGMEM = 
  "Error burning fuses !"
  "Press <OK> to perform"
  "a chip erase and     "
  "retry burning fuses. ";

// Variables
uint8_t inLFUSE, inHFUSE, inEFUSE;            // for reading fuses
uint8_t outLFUSE, outHFUSE;                   // for writing fuses
uint32_t signature;                           // for reading signature
boolean chipDetected;                         // chip detection flag
uint32_t lastmillis;                          // for time-out


void setup()
{
  // set up control lines for HV serial programming
  pinMode(VCC_PIN, OUTPUT);
  pinMode(SDI_PIN, OUTPUT);
  pinMode(SII_PIN, OUTPUT);
  pinMode(SCI_PIN, OUTPUT);
  pinMode(SDO_PIN, OUTPUT);     // configured as input when in programming mode
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(RST_PIN, HIGH);  // Level shifter is inverting, this shuts off 12V

  // OK-Button
  pinMode(BUTTON, INPUT_PULLUP);

  // prepare and start OLED
  oled.begin();
  oled.setFont(FONT6X8);
  oled.clear();
  oled.on();
  oled.switchRenderFrame();  
}


void loop()
{
  // show start screen
  oled.clear(); oled.setCursor(0, 0); printP(TitleScreen);
  oled.switchFrame();

  // start detection of device and repeat until positive detection
  chipDetected = false;                 // assume negative detection by now
  while (!chipDetected) {               // repeat until a valid chip was detected
    waitforButton();                    // wait for OK-Button pressed

    chipDetected = true;                // assume positive detection by now
    enterProgrammingMode();             // start programming mode
    readSignature();                    // read device signature

    // prepare to show information on OLED
    oled.clear(); oled.setCursor(0, 0); oled.print("Detected:    ");

    // make settings depending on detected device
    switch (signature) {
      case T13: oled.print("ATtiny13"); outLFUSE=T13_LFUSE; outHFUSE=T13_HFUSE; break;
      case T25: oled.print("ATtiny25"); outLFUSE=Tx5_LFUSE; outHFUSE=Tx5_HFUSE; break;
      case T45: oled.print("ATtiny45"); outLFUSE=Tx5_LFUSE; outHFUSE=Tx5_HFUSE; break;
      case T85: oled.print("ATtiny85"); outLFUSE=Tx5_LFUSE; outHFUSE=Tx5_HFUSE; break;
      default:  oled.print("Error !"); oled.setCursor(0, 1); printP(ErrorScreen);
                oled.switchFrame(); exitProgrammingMode(); chipDetected = false; break;
    }
  }

  // read and show detected fuse settings on OLED
  readFuses();                        // read current fuse setting
  printFuses();                       // show detected fuse settings
  oled.setCursor(0, 3); oled.print("<OK> to burn defaults");
  oled.switchFrame();

  // write and read back fuses
  waitforButton();                    // wait for OK-Button pressed
  writeFuses();                       // write default fuse settings
  readFuses();                        // read back fuse settings

  // check if fuses were burned correctly
  if( (inLFUSE != outLFUSE) | (inHFUSE != outHFUSE) ) {
    oled.clear(); oled.setCursor(0, 0); printP(EraseScreen);
    oled.switchFrame();
    waitforButton();                  // wait for OK-Button pressed
    eraseChip();                      // perform chip erase
    writeFuses();                     // write default fuse settings
    readFuses();                      // read back fuse settings
  }

  
  exitProgrammingMode();              // exit programming mode

  // write information on OLED
  oled.clear();
  oled.setCursor(0, 0); oled.print("Fuses burned");
  printFuses();
  oled.setCursor(0, 3); oled.print("Press <OK> to quit");
  oled.switchFrame();

  waitforButton();                    // wait for OK-Button pressed
}


// waits until OK-Button was pressed
void waitforButton() {
  while (!digitalRead(BUTTON));       // wait for button released
  delay (10);                         // debounce
  while (digitalRead(BUTTON));        // wait for button pressed
}


// starts the programming mode
void enterProgrammingMode() {
  // Initialize pins to enter programming mode
  pinMode     (SDO_PIN, OUTPUT);      // temporary
  digitalWrite(SDI_PIN, LOW);
  digitalWrite(SII_PIN, LOW);
  digitalWrite(SDO_PIN, LOW);
  digitalWrite(RST_PIN, HIGH);        // level shifter is inverting, this shuts off 12V
    
  // Enter High-voltage Serial programming mode
  digitalWrite(VCC_PIN, HIGH);        // apply VCC to start programming process
  delayMicroseconds(20);
  digitalWrite(RST_PIN, LOW);         // turn on 12v
  delayMicroseconds(10);
  pinMode(SDO_PIN, INPUT);            // release data line from target device
  delayMicroseconds(300);
}


// exits the programming mode
void exitProgrammingMode() {
  digitalWrite(SCI_PIN, LOW);
  digitalWrite(VCC_PIN, LOW);
  digitalWrite(RST_PIN, HIGH);        // turn off 12v
}


// reads signature of target device
void readSignature() {
  sendInstr(0x08, 0x4C);
  sendInstr(0x00, 0x0C);
  sendInstr(0x00, 0x68);
  signature = sendInstr(0x00, 0x6C);
  signature <<= 8;

  sendInstr(0x01, 0x0C);
  sendInstr(0x00, 0x68);
  signature |= sendInstr(0x00, 0x6C);
  signature <<= 8;

  sendInstr(0x02, 0x0C);
  sendInstr(0x00, 0x68);
  signature |= sendInstr(0x00, 0x6C);
}


// reads current fuse settings from target device
void readFuses() {
  // read lfuse
  sendInstr(0x04, 0x4C);
  sendInstr(0x00, 0x68);
  inLFUSE = sendInstr(0x00, 0x6C);
    
  // read hfuse
  sendInstr(0x04, 0x4C);
  sendInstr(0x00, 0x7A);
  inHFUSE = sendInstr(0x00, 0x7E);
    
  // read efuse
  sendInstr(0x04, 0x4C);
  sendInstr(0x00, 0x6A);
  inEFUSE = sendInstr(0x00, 0x6E);
}


// writes fuse settings to target device
void writeFuses() {
  // write hfuse
  sendInstr(0x40,  0x4C);
  sendInstr(outHFUSE, 0x2C);
  sendInstr(0x00,  0x74);
  sendInstr(0x00,  0x7C);
    
  // write lfuse
  sendInstr(0x40,  0x4C);
  sendInstr(outLFUSE, 0x2C);
  sendInstr(0x00,  0x64);
  sendInstr(0x00,  0x6C);
}


// performs chip erase
void eraseChip() {
  sendInstr(0x80, 0x4C);
  sendInstr(0x00, 0x64);
  sendInstr(0x00, 0x6C);
  while (!digitalRead(SDO_PIN));     // wait for the chip erase cycle to finish
}


// sends instruction to target device
uint8_t sendInstr(uint8_t SDI_BYTE, uint8_t SII_BYTE) {
  uint8_t SDO_BYTE = 0;             // byte read back from target device

  // wait until SDO_PIN goes high or time-out
  lastmillis = millis();
  while ( (!digitalRead(SDO_PIN)) && ((millis() - lastmillis) < 10) );
        
  // start bit
  digitalWrite(SDI_PIN, LOW);
  digitalWrite(SII_PIN, LOW);
  digitalWrite(SCI_PIN, HIGH);
  digitalWrite(SCI_PIN, LOW);

  // instruction bytes
  for (uint8_t i = 0; i < 8; i++)  {
    digitalWrite(SDI_PIN, bitRead(SDI_BYTE, 7 - i));
    digitalWrite(SII_PIN, bitRead(SII_BYTE, 7 - i));
                
    SDO_BYTE <<= 1;
    SDO_BYTE |= digitalRead(SDO_PIN);
    digitalWrite(SCI_PIN, HIGH);
    digitalWrite(SCI_PIN, LOW);            
  }
      
  // end bits
  digitalWrite(SDI_PIN, LOW);
  digitalWrite(SII_PIN, LOW);
  digitalWrite(SCI_PIN, HIGH);
  digitalWrite(SCI_PIN, LOW);
  digitalWrite(SCI_PIN, HIGH);
  digitalWrite(SCI_PIN, LOW);
        
  return SDO_BYTE;
}


// prints a string from progmem on the OLED
void printP(const char* p) {
  char ch = pgm_read_byte(p);
  while (ch != 0) {
    oled.print(ch);
    ch = pgm_read_byte(++p);
  }
}


// prints current fuse settings on the OLED
void printFuses() {
  oled.setCursor(0, 1); oled.print("Current fuse settings");
  oled.setCursor(0, 2);
  oled.print("l: ");  oled.print(inLFUSE, HEX);
  oled.print(" - h: "); oled.print(inHFUSE, HEX);
  oled.print(" - e: "); oled.print(inEFUSE, HEX);
}
