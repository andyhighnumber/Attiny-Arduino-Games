/*
   ATtiny Tetris Gold - Multi Button
                 music, additional sounds and improvements

   ===========================================================================
   Andy Jackson, Anthony Russell, Tobozo, Neven Boyanov, Jarosław Mazurkiewicz
   2015 / 2016 / 2017 / 2019

   Before the game starts:
   ===========================================================================
   Hold "DROP" BUTTON - Turns 'ghost' piece on and off
   Hold "DROP" and "ROTATE" together - Turns 'challenge mode' on and off

   The code that does not fall under the licenses of sources listed below
   can be used non-commercially with or without attribution.
   This software is supplied without warranty of any kind.

   For more information please visit:
   https://jm.iq.pl/tetris

*/

// The custom font file is the only additional file you should need to compile this game
#include "font8x8AJ.h"

// Standard Arduino headers
#include <EEPROM.h> 
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/interrupt.h> // needed for the additional interrupt

// Mode settings for functions with multiple purposes
#define NORMAL 0
#define GHOST 1
#define FULL 0
#define PARTIAL 1
#define DRAW 0
#define ERASE 1

// The horizontal width of the playing area
#define HORIZ 10
// The vertical visible space of the playing area
#define VERTDRAW 19
// The size of the array of blocks (some are outside visible area to allow them to drop in from off screen)
#define VERTMAX 24

// The horizontal position where pieces begin
#define STARTX 3
// The vertical position where pieces begin
#define STARTY 19
// What level does the game start on
#define STARTLEVEL 1
// The multiplying factor that sets how the speed scales with level (one level increment for every row cleared)
#define LEVELFACTOR 4
// The number of milliseconds before each drop (baseline)
#define DROPDELAY 600

#define DIGITAL_WRITE_HIGH(PORT) PORTB |= (1 << PORT)
#define DIGITAL_WRITE_LOW(PORT) PORTB &= ~(1 << PORT)

// Routines to set and clear bits (used in the sleep code)
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// Defines for OLED output
#define SSD1306XLED_H
#define SSD1306_SCL   PORTB4  // SCL, Pin 4 on SSD1306 Board - for webbogles board
#define SSD1306_SDA   PORTB3  // SDA, Pin 3 on SSD1306 Board - for webbogles board
#define SSD1306_SA    0x78  // Slave address

// The bitmaps for the little images of next block
static const byte miniBlock[][4] PROGMEM = {
    {0x77, 0x77, 0x00, 0x00}, 
    {0x70, 0x77, 0x70, 0x00}, 
    {0x70, 0x00, 0x70, 0x77}, 
    {0x70, 0x07, 0x70, 0x07}, 
    {0x70, 0x07, 0x00, 0xEE}, 
    {0x70, 0x77, 0x00, 0x0E}, 
    {0x70, 0x07, 0xEE, 0x00}
    };

// The bitmaps for the main blocks
static const int blocks[7] PROGMEM = {
  0x4444, 0x44C0, 
  0x4460, 0x0660, 
  0x06C0, 0x0E40, 
  0x0C60};

// The bitmaps for blocks on the screen
static const byte  blockout[16] PROGMEM = {
  0xF8, 0x00, 0x3E, 0x80, 
  0x0F, 0xE0, 0x03, 0xF8, 
  0x3E, 0x80, 0x0F, 0xE0, 
  0x03, 0xF8, 0x3E, 0x00};

// The bitmaps for ghost blocks on the screen
static const byte  ghostout[16] PROGMEM = {
  0x88, 0x00, 0x22, 0x80, 
  0x08, 0x20, 0x02, 0x88, 
  0x22, 0x80, 0x08, 0x20, 
  0x02, 0x88, 0x22, 0x00};

// Decode lookup to translate block positions to the 8 columns on the screen
static const byte startDecode[11] PROGMEM = {0,1,1,2,3,4,4,5,6,7,8};
static const byte endDecode[11] PROGMEM =   {1,2,3,3,4,5,6,6,7,8,8};

// The  logo on the opening screen - adapted from the original by Tobozo https://github.com/tobozo
const byte brickLogo[] PROGMEM= {
  0x01, 0x01, 0x01, 0x01, 0x81, 0x81, 0xC1, 0xE1, 
  0xF1, 0xF1, 0x01, 0x11, 0xF1, 0xF1, 0xE1, 0xC1, 
  0xC1, 0x81, 0x81, 0x01, 0x01, 0x01, 0x01, 0x01, 
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 
  0x01, 0x01, 0x01, 0x01, 0xFC, 0xFC, 0xFE, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFE, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xFF, 0xFF, 0x7F, 0x3F, 0xBF, 0x9F, 0xCF, 0xEF, 
  0xE7, 0xF7, 0xFB, 0xE0, 0x01, 0xFB, 0xF3, 0xF7, 
  0xE7, 0xEF, 0xCF, 0xDF, 0xDF, 0xBF, 0xBF, 0x30, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x06, 0xFE, 0xFC, 
  0xFC, 0xFC, 0xF8, 0xF8, 0xF0, 0xF0, 0xE0, 0x00, 
  0x00, 0xFF, 0xFF, 0x7F, 0x7F, 0x3F, 0xBF, 0x9F, 
  0xDF, 0xCF, 0xEF, 0xEF, 0xE4, 0x00, 0xF7, 0xE7, 
  0xEF, 0xEF, 0xCF, 0xDF, 0xDF, 0x9F, 0xBF, 0xBF, 
  0x3F, 0x00, 0x07, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0xFC, 0xFE, 0xFE, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0x80, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x0E, 
  0x3E, 0x1E, 0x1C, 0x1D, 0x0D, 0x09, 0x03, 0x03, 
  0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0x7F, 0x7F, 0x3F, 0x00, 0x3F, 
  0x3F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0xFF, 0xFF, 
  0xFF, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x83, 0x83, 
  0x83, 0x89, 0x8D, 0x8D, 0x8C, 0x8E, 0x8E, 0x8E, 
  0x8F, 0x8F, 0x9F, 0x8F, 0x8F, 0x8F, 0x8F, 0x87, 
  0x86, 0x86, 0x82, 0x82, 0x82, 0x80, 0x80, 0x80,
  0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80};

const int music[][2] PROGMEM = {
  {  752, 828 }, { 1004, 384 }, {  948, 384 }, {  845, 384 }, {  752,  190 },
  {  845, 190 }, {  948, 384 }, { 1004, 384 }, { 1127, 808 }, {    0,   40 },
  { 1127, 364 }, {  948, 384 }, {  752, 828 }, {  845, 384 }, {  948,  384 },
  { 1004, 808 }, {    0,  40 }, { 1004, 380 }, {  948, 384 }, {  845,  828 },
  {  752, 828 }, {  948, 828 }, { 1127, 768 }, {    0,  40 }, { 1127, 1676 },

  {  845, 798 }, {    0,  40 }, {  845, 354 }, {  710, 384 }, {  562,  828 },
  {  632, 384 }, {  710, 384 }, {  752, 808 }, {    0,  40 }, {  752,  364 },
  {  948, 384 }, {  752, 828 }, {  845, 384 }, {  948, 384 }, { 1004,  808 },
  {    0,  40 }, { 1004, 380 }, {  948, 384 }, {  845, 828 }, {  752,  828 },
  {  948, 828 }, { 1127, 748 }, {    0,  14 }, {    0,  26 }, { 1127, 1696 }};

// Function prototypes - generic ones I use in all games
void beep(int,int);

// Function prototypes - screen control modified from https://bitbucket.org/tinusaur/ssd1306xled
void ssd1306_init(void);
void ssd1306_xfer_start(void);
void ssd1306_xfer_stop(void);
void ssd1306_send_byte(uint8_t byte);
void ssd1306_send_command(uint8_t command);
void ssd1306_send_data_start(void);
void ssd1306_send_data_stop(void);
void ssd1306_setpos(uint8_t x, uint8_t y);
void ssd1306_fillscreen(uint8_t fill_Data);
void ssd1306_char_f8x8(uint8_t x, uint8_t y, const char ch[]);

// Function prototypes - tetris-specific
void playTetris(void);
void handleInput(void);

void drawScreen(int startCol, int endCol, int startRow, int endRow, byte mode);
void drawScreenBorder(void);

byte readBlockArray(byte x, byte y);
void writeblockArray(byte x, byte y, bool value);
byte readGhostArray(byte x, byte y);
void writeGhostArray(byte x, byte y, bool value);
void fillGrid(byte value, bool mode);

void rotatePiece(void);
bool movePieceDown(void);
void movePieceLeft(void);
void movePieceRight(void);
byte checkCollision(void);

bool createGhost(void);
void drawGhost(byte action);
void loadPiece(byte pieceNumber, byte row, byte column);
void drawPiece(byte action);
void setNextBlock(byte pieceNumber);

// Variables 
struct pieceSpace {
  byte blocks[4][4];
  int row;
  int column;
};

pieceSpace currentPiece = {0};  // The piece in play
pieceSpace oldPiece = {0};      // Buffer to hold the current piece whilst its manipulated
pieceSpace ghostPiece = {0};    // Current ghost piece

unsigned long moveTime = 0;     // Baseline time for current move
unsigned long keyTime = 0;      // Baseline time for current keypress

byte keyLock = 0;               // Holds the mode of the last keypress (for debounce and stuff)

byte nextBlockBuffer[8][2];     // The little image of the next block 
byte nextPiece = 0;             // The identity of the next piece
byte blockArray[HORIZ][3];      // The byte-array of blocks
byte ghostArray[HORIZ][3];      // The byte-array of ghost pieces
bool stopAnimate;               // True when the game is running

int lastGhostRow = 0;           // Buffer to hold previous ghost position - for accurate drawing
int score = 0;                  // Score buffer
int topScore = 0;               // High score buffer

bool challengeMode = 0;         // Is the system in "Hard" mode?
bool ghost = 0;                 // Is the ghost active?

int level = 0;                  // Current level (increments once per cleared line)



/* 
 * Interrupt handlers - to make sure every button press is caught promptly! 
 */

// PB0 pin button interrupt
ISR(PCINT0_vect) { 
  if (keyLock == 0) {
    keyLock = 4; // Nothing complicated to do here - this is picked up in the handleInput() function 
    }
}

/*
 * This interrupt routine relies on having three buttons wired to pin 7 of the Attiny85 - all with a single pull-down resistor (10k) and 
 *  - One that pulls-up to +Vcc (giving an analog reading of 1023)
 *  - One that pulls-up via a 1k resistor (analog reading somewhere around 930ish)
 *  - One that pulls-up via a 2.2 resistor (analog reading around 840)
 * 
 * The key point is that all three buttons provide a voltage that sends the pin to logic-high, thus triggering the interrupt
 * (therefore you could add more buttons, as long as the voltage is always above the threshold that the Attiny85 would class
 * as logic-high). 
 * 
 * In practice, this technique requires a reasonably well made and well insulated board - adding moisture to to board (via damp
 * hands for example) would likely throw the voltages out enough that this technique would become unreliable.
 * 
 */
 // PB2 pin button interrupt
void playerIncTetris() { 
  if (keyLock == 0) {
    int ic = analogRead(1);
    if (ic > 800 && ic < 855) {
      keyLock = 1;
    } else if (ic > 855 && ic < 970) {
      keyLock = 2;
    } else if (ic > 970 && ic < 1025) {
      keyLock = 3;
    }
  }
}

// Beep function - originally from the games by www.webboggles.com 
void beep(int bCount,int bDelay){
 for (int i = 0; i<=bCount; i++){digitalWrite(1,HIGH);for(int i2=0; i2<bDelay; i2++){__asm__("nop\n\t");}digitalWrite(1,LOW);for(int i2=0; i2<bDelay; i2++){__asm__("nop\n\t");}}
}

// SSD1306 screen drawing functions - adapted from https://bitbucket.org/tinusaur/ssd1306xled
void ssd1306_char_f8x8(uint8_t x, uint8_t y, const char ch[]) {
  uint8_t c, i, j = 0;

  while (ch[j] != '\0')
  {
    // These corrections allow for the fact that I've stripped out sections of the ASCII map from the font file
    // If code space allows, then you could add some or all of them back in, but then you'd need to remove some or all of these lines.
    c = ch[j] - 32; // to space
    if (c > 0) c = c - 12; // to dash
    if (c > 15) c = c - 7; 
    if (c > 40) c = c - 6;

    ssd1306_setpos(y, x);
    ssd1306_send_data_start();
    for (byte lxn = 0; lxn < 8; lxn++) {    
      ssd1306_send_byte(pgm_read_byte(&font[c][7-lxn]));      
      }
    ssd1306_send_data_stop();
    x+= 1;
    j++;
  }
}

void ssd1306_init(void) {
  DDRB |= (1 << SSD1306_SDA); // Set port as output
  DDRB |= (1 << SSD1306_SCL); // Set port as output

  ssd1306_send_command(0xAE); // display off
  ssd1306_send_command(0x00); // Set Memory Addressing Mode
  ssd1306_send_command(0x10); // 00,Horizontal Addressing Mode;01,VERTDRAWical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
  ssd1306_send_command(0x40); // Set Page Start Address for Page Addressing Mode,0-7
  ssd1306_send_command(0x81); // Set COM Output Scan Direction
  ssd1306_send_command(0xCF); // ---set low column address
  ssd1306_send_command(0xA1); // ---set high column address
  ssd1306_send_command(0xC8); // --set start line address
  ssd1306_send_command(0xA6); // --set contrast control register
  ssd1306_send_command(0xA8);
  ssd1306_send_command(0x3F); // --set segment re-map 0 to 127
  ssd1306_send_command(0xD3); // --set normal display
  ssd1306_send_command(0x00); // --set multiplex ratio(1 to 64)
  ssd1306_send_command(0xD5); //
  ssd1306_send_command(0x80); // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
  ssd1306_send_command(0xD9); // -set display offset
  ssd1306_send_command(0xF1); // -not offset
  ssd1306_send_command(0xDA); // --set display clock divide ratio/oscillator frequency
  ssd1306_send_command(0x12); // --set divide ratio
  ssd1306_send_command(0xDB); // --set pre-charge period
  ssd1306_send_command(0x40); //
  ssd1306_send_command(0x20); // --set com pins hardware configuration
  ssd1306_send_command(0x02);
  ssd1306_send_command(0x8D); // --set vcomh
  ssd1306_send_command(0x14); // 0x20,0.77xVcc
  ssd1306_send_command(0xA4); // --set DC-DC enable
  ssd1306_send_command(0xA6); //
  ssd1306_send_command(0xAF); // --turn on oled panel
}

void ssd1306_xfer_start(void) {
  DIGITAL_WRITE_HIGH(SSD1306_SCL);  // Set to HIGH
  DIGITAL_WRITE_HIGH(SSD1306_SDA);  // Set to HIGH
  DIGITAL_WRITE_LOW(SSD1306_SDA);   // Set to LOW
  DIGITAL_WRITE_LOW(SSD1306_SCL);   // Set to LOW
}

void ssd1306_xfer_stop(void) {
  DIGITAL_WRITE_LOW(SSD1306_SCL);   // Set to LOW
  DIGITAL_WRITE_LOW(SSD1306_SDA);   // Set to LOW
  DIGITAL_WRITE_HIGH(SSD1306_SCL);  // Set to HIGH
  DIGITAL_WRITE_HIGH(SSD1306_SDA);  // Set to HIGH
}

void ssd1306_send_byte(uint8_t byte) {
  uint8_t i;
  for (i = 0; i < 8; i++)
  {
    if ((byte << i) & 0x80)
      DIGITAL_WRITE_HIGH(SSD1306_SDA);
    else
      DIGITAL_WRITE_LOW(SSD1306_SDA);

    DIGITAL_WRITE_HIGH(SSD1306_SCL);
    DIGITAL_WRITE_LOW(SSD1306_SCL);
  }
  DIGITAL_WRITE_HIGH(SSD1306_SDA);
  DIGITAL_WRITE_HIGH(SSD1306_SCL);
  DIGITAL_WRITE_LOW(SSD1306_SCL);
}

void ssd1306_send_command(uint8_t command) {
  ssd1306_xfer_start();
  ssd1306_send_byte(SSD1306_SA);  // Slave address, SA0=0
  ssd1306_send_byte(0x00);  // write command
  ssd1306_send_byte(command);
  ssd1306_xfer_stop();
}

void ssd1306_send_data_start(void) {
  ssd1306_xfer_start();
  ssd1306_send_byte(SSD1306_SA);
  ssd1306_send_byte(0x40);  //write data
}

void ssd1306_send_data_stop(void) {
  ssd1306_xfer_stop();
}

void ssd1306_setpos(uint8_t x, uint8_t y)
{
  if (y > 7) return;
  ssd1306_xfer_start();
  ssd1306_send_byte(SSD1306_SA);  //Slave address,SA0=0
  ssd1306_send_byte(0x00);  //write command

  ssd1306_send_byte(0xb0 + y);
  ssd1306_send_byte(((x & 0xf0) >> 4) | 0x10); // |0x10
  ssd1306_send_byte((x & 0x0f) | 0x01); // |0x01

  ssd1306_xfer_stop();
}

void ssd1306_fillscreen(uint8_t fill_Data) {
  uint8_t m, n;
  for (m = 0; m < 8; m++)
  {
    ssd1306_send_command(0xb0 + m); //page0-page1
    ssd1306_send_command(0x00);   //low column start address
    ssd1306_send_command(0x10);   //high column start address
    ssd1306_send_data_start();
    for (n = 0; n < 128; n++)
    {
      ssd1306_send_byte(fill_Data);
    }
    ssd1306_send_data_stop();
  }
}

// Sleep code from http://www.re-innovation.co.uk/web12/index.php/en/blog-75/306-sleep-modes-on-attiny85
void system_sleep() {
  ssd1306_fillscreen(0x00);
  ssd1306_send_command(0xAE);
  cbi(ADCSRA,ADEN);                    // switch Analog to DigitalconVERTDRAWer OFF
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  sleep_mode();                        // System actually sleeps here
  sleep_disable();                     // System continues execution here when watchdog timed out 
  sbi(ADCSRA,ADEN);                    // switch Analog to DigitalconVERTDRAWer ON  
  ssd1306_send_command(0xAF);
}

void randomSeedEEPROM() {
  uint16_t seed = eeprom_read_word( (uint16_t *) 0x03);
  randomSeed(seed++);
  eeprom_write_word( (uint16_t *) 0x03, seed & 0xFF);
}

void soundPlay(int note, int duration) {
  if (note > 0) {
    for (long c = 0; c < duration * 1000L; c += note * 2) {
      PORTB |= (1 << 1);
      delayMicroseconds(note);
      PORTB &= ~(1 << 1);
      delayMicroseconds(note);
    }
  } else { delay(duration); }
}

// Arduino stuff
void setup() {
  DDRB = 0b00000010;    // set PB1 as output (for the speaker)
  PCMSK = 0b00000001;   // pin change mask: listen to portb bit 1
  GIMSK |= 0b00100000;  // enable PCINT interrupt
  randomSeedEEPROM();   // initializing the pseudorandom number generator
  sei();                // enable all interrupts
  ssd1306_init();       // initialise the screen
}

void loop() {
  ssd1306_init();
  ssd1306_fillscreen(0x00);

  drawScreenBorder();
  ssd1306_char_f8x8(1, 64,"TETRIS");
  ssd1306_char_f8x8(1, 48, "Attiny");
  ssd1306_char_f8x8(1, 40, "Arcade");
  drawScreenBorder();

  for (byte lxn = 0; lxn < 8; lxn++) {
    ssd1306_setpos(78, lxn); 
    ssd1306_send_data_start(); 
    for (byte lxn2 = 0; lxn2 < 36; lxn2++) {
      ssd1306_send_byte(pgm_read_byte(&brickLogo[36*lxn+lxn2]));  
    }
    ssd1306_send_data_stop(); 
  }

  long startT = millis();
  long nowT =0;
  boolean sChange = 0;
  while(digitalRead(0) == HIGH) {
    nowT = millis();
    if (nowT - startT > 2000) {
      sChange = 1;
      beep(20, 956);
      if (digitalRead(2) == HIGH) {
        ssd1306_char_f8x8(2, 8, "MODE"); 
        if (challengeMode == 0) { 
          challengeMode = 1; 
          ssd1306_char_f8x8(2, 16, "HARD"); 
        } else { 
          challengeMode = 0; 
          ssd1306_char_f8x8(1, 16, "NORMAL"); 
        }
      } else {
        ssd1306_char_f8x8(1, 16, "GHOST"); 
        if (ghost == 0) { 
          ghost = 1; 
          ssd1306_char_f8x8(2, 8, "ON"); 
        } else { 
          ghost = 0; 
          ssd1306_char_f8x8(2, 8, "OFF"); 
        }    
      }
      break;
    }
    if (sChange == 1) break;
  }  
  while(digitalRead(0) == HIGH);
  
  if (sChange == 0) {
    ssd1306_char_f8x8(2, 21, "GOLD"); 
    // playing the Tetris Theme
    uint8_t sng = (ghost == 1) ? 50 : 25;
    for (uint8_t i = 0; i < sng; i++) {
      soundPlay(pgm_read_word_near(&(music[i][0])), pgm_read_word_near(&(music[i][1])));
    }
    ssd1306_fillscreen(0x00);
    playTetris();
  }
  delay(1000);
  system_sleep();
}

/*
 *  These functional allow optimal storing of the array of Tetris and Ghost blocks by using a 10x3 array of bytes and extracting the relevant bit through shifting and masking
 */

byte readBlockArray(byte x, byte y) {
    if (y < 8) {
      return ((blockArray[x][0] & B00000001 << y) >> y);
    } else if (y > 15) {
      return ((blockArray[x][2] & B00000001 << y-15) >> y-15);
    } else {
      return ((blockArray[x][1] & B00000001 << y-8) >> y-8);
    }   
}

void writeblockArray(byte x, byte y, bool value) {
    byte arr = 0;
    if (y<8) {
      // do nothing 
    } else if (y > 15) {
      arr = 2;
      y-=15;
    } else  {
      arr = 1;
      y-=8;
    }  
    if (value == 1) blockArray[x][arr] |= B00000001 << y; else blockArray[x][arr] &= (B11111110 << y) | (B01111111 >> 7-y);
}

byte readGhostArray(byte x, byte y) {
    if (y < 8) {
      return ((ghostArray[x][0] & B00000001 << y) >> y);
    } else if (y > 15) {
      return ((ghostArray[x][2] & B00000001 << y-15) >> y-15);
    } else {
      return ((ghostArray[x][1] & B00000001 << y-8) >> y-8);
    }   
}

void writeGhostArray(byte x, byte y, bool value) {
    byte arr = 0;
    if (y<8) {
      // do nothing
    } else if (y > 15) {
      arr = 2;
      y-=15;
    } else  {
      arr = 1;
      y-=8;
    }  
    if (value == 1) ghostArray[x][arr] |= B00000001 << y; else ghostArray[x][arr] &= (B11111110 << y) | (B01111111 >> 7-y);
}

void fillGrid(byte value, bool mode) {
  for (char r = 0; r < VERTMAX; r++) {
    for (char c = 0; c < HORIZ; c++) {
      if (mode == GHOST) writeGhostArray(c,r, value); else writeblockArray(c,r, value);
    }
  }
}

void rotatePiece(void) {
  byte blocks[4][4];

  memcpy(oldPiece.blocks, currentPiece.blocks, 16);
  oldPiece.row = currentPiece.row;
  oldPiece.column = currentPiece.column;

  for (byte i = 0; i < 4; ++i) {
    for (byte j = 0; j < 4; ++j) {
      blocks[j][i] = currentPiece.blocks[4 - i - 1][j];
    }
  }
  oldPiece = currentPiece;
  memcpy(currentPiece.blocks, blocks, 16);
  if (checkCollision()) currentPiece = oldPiece; else {
    drawGhost(ERASE);
    if (createGhost()) drawGhost(DRAW);
    }
}

bool movePieceDown(void) {
  int rndPiece = 0;

  memcpy(oldPiece.blocks, currentPiece.blocks, 16);
  oldPiece.row = currentPiece.row;
  oldPiece.column = currentPiece.column;

  currentPiece.row--;

  //check collision
  if (checkCollision()) {
    currentPiece.row = oldPiece.row;
    drawPiece(DRAW);
    byte totalRows = 0;
    
    for (byte row = 0; row < VERTMAX; row++) { // scan the whole block (it's quick - there's no drawing to do)
      bool rowFull = 1;
      for (byte col = 0; col < HORIZ; col++) { // scan across this row - every column
        if (readBlockArray(col,row) == 0) rowFull = 0; // if we hit any blank spaces, the row's not full
      }
      if (rowFull) {
        totalRows++;
        for (int i = 800; i>200; i = i - 200) beep(30,i); // happy sound
        for (byte col = 0; col < HORIZ; col++) writeblockArray(col,row,0); // write zeros across this whole row
        drawGameScreen(0,HORIZ-1,row,row+1,PARTIAL); // draw the row we're removing (for animation)
        delay(30); // delay slightly to make the deletion of rows visible
        for (byte dropCol = 0; dropCol < HORIZ; dropCol++) { // for every column
          for (byte dropRow = row; dropRow < VERTMAX-1; dropRow ++) writeblockArray(dropCol,dropRow,readBlockArray(dropCol,dropRow+1)); // drop everything down as many as the row's we've cleared
        }
        row--; // we need to check this row again as it could now have things in it!
      }
    }
    level += totalRows;
    switch (totalRows) {
      case 1:   score += 40; break;
      case 2:   score += 100; break;
      case 3:   score += 300; break;
      case 4:   score += 800; 
    }
    drawGameScreen(0,10, 0,VERTDRAW,FULL); 
    displayScore(score, 0,117,0);
    loadPiece(nextPiece, STARTY, STARTX);
    if (checkCollision()) {
      stopAnimate = true;
    } else {
      loadPiece(nextPiece, STARTY, STARTX);
      drawGhost(ERASE);
      if (createGhost()) drawGhost(DRAW);
    }
    nextPiece = random(1, 8);
    setNextBlock(nextPiece);
  }
  drawGhost(ERASE);
  if (createGhost()) drawGhost(DRAW);
}

void movePieceLeft(void) {
  byte response;
 
  oldPiece = currentPiece;
  currentPiece.column = currentPiece.column - 1;

  response = checkCollision();

  if (checkCollision()) {
    currentPiece = oldPiece; // back to where it was
  } else {
    drawGhost(ERASE);
    if (createGhost()) drawGhost(DRAW);
  }
}

void movePieceRight(void) {
  oldPiece = currentPiece;
  currentPiece.column = currentPiece.column + 1;
  //check collision
  if (checkCollision()) 	{
    currentPiece = oldPiece; // back to where it was
  } else {
    drawGhost(ERASE);
    if (createGhost()) drawGhost(DRAW);
  }
}

byte checkCollision(void) {
  byte pieceRow = 0;
  byte pieceColumn = 0;

  for (int c = currentPiece.column; c < currentPiece.column + 4; c++) {
    for (int r = currentPiece.row; r < currentPiece.row + 4; r++) {
      if (currentPiece.blocks[pieceColumn][pieceRow]) {
        if (c < 0) return 2;
        if (c > 9) return 1;
        if (r < 0) return 1;
        if (c >= 0 && r >= 0 && c < HORIZ && r < VERTMAX) {
          if (readBlockArray(c,r)) {
            return 1; //is it on landed blocks?
          }
        }
      }
      pieceRow++;
    }
    pieceRow = 0;
    pieceColumn++;
  }
  return 0;
}

void handleInput(void) {
  if (digitalRead(0) == LOW && digitalRead(2) == LOW) {
    if (keyLock == 1) {
      drawPiece(ERASE);
      movePieceLeft();
      drawPiece(DRAW);
      drawGameScreen(currentPiece.column, currentPiece.column + 5, currentPiece.row, currentPiece.row+4, PARTIAL);         
    } else if (keyLock == 2) {
      drawPiece(ERASE);
      movePieceRight();
      drawPiece(DRAW);
      drawGameScreen(currentPiece.column-1, currentPiece.column + 4, currentPiece.row, currentPiece.row+4,PARTIAL);         
    } else if (keyLock == 3) {
      drawPiece(ERASE);
      rotatePiece();
      drawPiece(DRAW);
      drawGameScreen(currentPiece.column, currentPiece.column + 4, currentPiece.row, currentPiece.row+4,PARTIAL);         
    }
    keyLock = 0;      
  }

  if (keyLock == 4) {
    while (digitalRead(0) == HIGH) {
      drawPiece(ERASE);
      movePieceDown();
      drawPiece(DRAW);
      drawGameScreen(currentPiece.column, currentPiece.column + 4, currentPiece.row, currentPiece.row+5,PARTIAL);         
      delay(10);
      if (stopAnimate) return;
    }
    keyLock = 0;
  }
  delay(30);
}

void setNextBlock(byte pieceNumber) {
  beep(20, 956);
  memset(nextBlockBuffer, 0, sizeof nextBlockBuffer); //clear buffer
  pieceNumber--;
  if (pieceNumber == 0) {
      for (int k = 2; k < 6; k++) {
        nextBlockBuffer[k][0] = pgm_read_byte(&miniBlock[pieceNumber][0]);
        nextBlockBuffer[k][1] = pgm_read_byte(&miniBlock[pieceNumber][0]);
      }
  
  } else {  
      for (int k = 0; k < 3; k++) {
        nextBlockBuffer[k][0] = pgm_read_byte(&miniBlock[pieceNumber][0]);
        nextBlockBuffer[k][1] = pgm_read_byte(&miniBlock[pieceNumber][1]);
      }
      for (int k = 4; k < 7; k++) {
        nextBlockBuffer[k][0] = pgm_read_byte(&miniBlock[pieceNumber][2]);
        nextBlockBuffer[k][1] = pgm_read_byte(&miniBlock[pieceNumber][3]);
      }
  }
  drawGameScreen(0,10,0,VERTDRAW, FULL);
}

void drawScreenBorder(void) {
    ssd1306_setpos(0, 0);
    ssd1306_send_data_start();
    ssd1306_send_byte(0xFF);  
    for (byte c = 1; c < 126; c++) {
      ssd1306_send_byte(B00000001);  
    }
    ssd1306_send_byte(0xFF);  
    ssd1306_send_data_stop();  
    
    for (byte r = 1; r < 7; r++) {
    ssd1306_setpos(0, r);
    ssd1306_send_data_start();
    ssd1306_send_byte(0xFF);        
    ssd1306_send_data_stop();  
    ssd1306_setpos(127, r);
    ssd1306_send_data_start();
    ssd1306_send_byte(0xFF);        
    ssd1306_send_data_stop();  
    }

    ssd1306_setpos(0, 7);
    ssd1306_send_data_start();
    ssd1306_send_byte(0xFF);  
    for (byte c = 1; c < 126; c++) {
      ssd1306_send_byte(B10000000);  
    }
    ssd1306_send_byte(0xFF);  
    ssd1306_send_data_stop();  
}

void displayScore(int score, int xpos, int y, bool blank) {
  byte scoreOut[6];
  scoreOut[5] = (score % 10);
  scoreOut[4] = ((score / 10) % 10);
  scoreOut[3] = ((score / 100) % 10);
  scoreOut[2] = ((score / 1000) % 10);
  scoreOut[1] = ((score / 10000) % 10);
  scoreOut[0] = ((score / 100000) % 10);

  for (byte x = xpos; x<xpos+6; x++) {
    ssd1306_setpos(y, x);
    ssd1306_send_data_start();
    for (byte lxn = 0; lxn < 8; lxn++) {    
      if (blank) ssd1306_send_byte(0); else ssd1306_send_byte(pgm_read_byte(&font[4+scoreOut[x-xpos]][7-lxn]));      
    }
    ssd1306_send_data_stop();
  }
}

void drawGameScreen(int startCol, int endCol, int startRow, int endRow, byte mode) {
  drawScreen(startCol, endCol, startRow, endRow, mode);

  if (mode == PARTIAL) {
    if (ghostPiece.row < lastGhostRow) { // ghost has moved down :)
      drawScreen(startCol, endCol, ghostPiece.row, lastGhostRow + 4, mode) ;  
    } else { // ghost has moved up (presumably!)
      drawScreen(startCol, endCol, lastGhostRow, ghostPiece.row + 4, mode) ;  
    }
    
  }
}

void drawScreen(int startCol, int endCol, int startRow, int endRow, byte mode) {
  byte temp = 0;
  byte separator = 0;
  byte reader = 0;
  byte blockReader = 0;
  
  if(startCol < 0) startCol = 0;
  if(endCol > 10) endCol = 10;
  if(startRow < 0) startRow = 0;
  if(endRow > VERTDRAW) endRow = VERTDRAW;
  
  byte startScreenCol = pgm_read_byte(&startDecode[startCol]);
  byte endScreenCol = pgm_read_byte(&endDecode[endCol]);
  
    for (byte col = startScreenCol; col < endScreenCol; col++) {
    if (col < 4) reader = col; else if (col < 7) reader = col+1; else reader = col + 2;
    blockReader = 2 * col;
    ssd1306_setpos(startRow*6, col); // Start from the end of this column (working up the screen) on the required row
    ssd1306_send_data_start(); 
    if (startRow == 0) ssd1306_send_byte(B11111111); else {
      if (col == 0) ssd1306_send_byte(B00000001); else if (col == 7) ssd1306_send_byte(B10000000); else ssd1306_send_byte(B00000000);
    }
    for (byte r = startRow; r < endRow; r++ ) { // For each row in the array of tetris blocks
      for (byte piece = 0; piece < 5; piece ++) { // for each of the 5 filled lines of the block
        if (col == 0) temp = B00000001; else if (col == 7) temp = B10000000; else temp = 0x00; // if we're on the far left, draw the left wall, on the far right draw the right wall, otherwise its a blank separator between blocks
        separator = temp; // we'll need this again later! 

        if (readBlockArray(reader,r)) { 
          temp = temp | pgm_read_byte(&blockout[blockReader]);
        }
        if (readBlockArray(reader+1,r)) { 
          temp = temp | pgm_read_byte(&blockout[blockReader+1]);
        }

        if(ghost) {
          if (readGhostArray(reader,r) && (piece == 0 || piece == 4)) { 
            temp = temp | pgm_read_byte(&blockout[blockReader]);
          } else if (readGhostArray(reader,r)) { 
            temp = temp | pgm_read_byte(&ghostout[blockReader]);
          }
                
          if (readGhostArray(reader+1,r) && (piece == 0 || piece == 4)) { 
            temp = temp | pgm_read_byte(&blockout[blockReader+1]);
          } else if (readGhostArray(reader+1,r)) { 
            temp = temp | pgm_read_byte(&ghostout[blockReader+1]);
          }
        }
        ssd1306_send_byte(temp);          
      }
      ssd1306_send_byte(separator); // between blocks - same one as we used at the start
    }    
    if (mode == FULL) if (col > 5) for (byte blockline = 0; blockline < 8; blockline++) ssd1306_send_byte(nextBlockBuffer[blockline][col-6]);
    ssd1306_send_data_stop(); 
  }
}

bool createGhost(void) {
  byte tempRow = currentPiece.row;

  if (currentPiece.row < 3) return 0;

  currentPiece.row-=2;
  while(checkCollision() == 0) currentPiece.row--;

  memcpy(ghostPiece.blocks, currentPiece.blocks, 16);
  ghostPiece.row = currentPiece.row+1;
  ghostPiece.column = currentPiece.column;
  currentPiece.row = tempRow;
  
  if (ghostPiece.row > currentPiece.row - 3) return 0; else return 1;
}

void loadPiece(byte pieceNumber, byte row, byte column) {
  byte incr = 0;

  pieceNumber--;
  
  for (byte lxn = 0; lxn < 4; lxn++) {
    for (byte lxn2 = 0; lxn2 < 4; lxn2++) {
      if ( ((1 << incr) & pgm_read_word(&blocks[pieceNumber])) >> incr == 1) {
        currentPiece.blocks[lxn][lxn2] = 1;                
      } else currentPiece.blocks[lxn][lxn2] = 0;
      incr++;
    }
  }
  currentPiece.row = row;
  currentPiece.column = column;
}

void drawPiece(byte action) {
  for (byte lxn = 0; lxn < 4; lxn++) {
    for (byte lxn2 = 0; lxn2 < 4; lxn2++) {
      if (currentPiece.blocks[lxn][lxn2] == 1) {
        if (action == DRAW) writeblockArray(currentPiece.column + lxn,currentPiece.row + lxn2,1); else if (action == ERASE) writeblockArray(currentPiece.column + lxn,currentPiece.row + lxn2,0);                
      }
    }
  }
}

void drawGhost(byte action) {
  for (byte lxn = 0; lxn < 4; lxn++) {
    for (byte lxn2 = 0; lxn2 < 4; lxn2++) {
      if (ghostPiece.blocks[lxn][lxn2] == 1) {
        if (action == DRAW) writeGhostArray(ghostPiece.column + lxn,ghostPiece.row + lxn2,1); else if (action == ERASE) {writeGhostArray(ghostPiece.column + lxn,ghostPiece.row + lxn2,0); lastGhostRow = ghostPiece.row; }                
      }
    }
  }
}

void playTetris(void) {
  stopAnimate = 0;
  score = 0;
  keyLock = 0;

  fillGrid(0, NORMAL);
  fillGrid(0, GHOST);

  // Attach the interrupt to read key 2
    attachInterrupt(0,playerIncTetris,RISING);

  loadPiece(random(1, 8), STARTY, STARTX);
  drawPiece(DRAW);
  if (createGhost()) drawGhost(DRAW);
  drawGhost(DRAW);
  nextPiece = random(1, 8);
  setNextBlock(nextPiece);
  
  // Fill up the screen with random crap if it's in challenge mode!
  if (challengeMode) {
    for (byte cl = 0; cl < 100; cl++) {
      drawPiece(ERASE);
      movePieceDown();
      if (random(1,8) >4) movePieceLeft();
      drawPiece(DRAW);      
    }
  } 

  // Reset the level
  level = STARTLEVEL;
  
  drawGameScreen(0,10,0,VERTDRAW, FULL); 
  displayScore(score, 0,117,0);
  
  while (stopAnimate == 0) {
    drawPiece(ERASE);
    movePieceDown();
    drawPiece(DRAW);
    beep(20, 568);
    drawGameScreen(currentPiece.column, currentPiece.column + 4, currentPiece.row, currentPiece.row+5, PARTIAL);         
    moveTime = millis();
    if (level * LEVELFACTOR > DROPDELAY) level = DROPDELAY / LEVELFACTOR;
    while ((millis() - moveTime) < (DROPDELAY - level* LEVELFACTOR)) {
      handleInput();
    }
  }

  ssd1306_fillscreen(0x00);

  bool newHigh = false;
  topScore = EEPROM.read(0);
  topScore = topScore << 8;
  topScore = topScore |  EEPROM.read(1);
  
  if (score > topScore) { 
    topScore = score;
    EEPROM.write(1,score & 0xFF); 
    EEPROM.write(0,(score>>8) & 0xFF);
    newHigh = true; 
  }
  drawScreenBorder();

  displayScore(score, 1,80,0);
  displayScore(topScore, 1,40,0);
   for (int i = 0; i<1000; i = i+ 50){
    beep(50,i);
    }
  for (byte lx = 0; lx<4;lx++) {
    displayScore(score, 1,80,1);    
    if (newHigh) displayScore(topScore, 1,40,1);
    delay(200);
    displayScore(score, 1,80,0);    
    if (newHigh) displayScore(topScore, 1,40,0);
    delay(200);
  }
}
