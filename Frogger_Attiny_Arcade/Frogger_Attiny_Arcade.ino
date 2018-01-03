/*  2015 / 2016 / 2017
 *  Frogger game by Andy Jackson - Twitter @andyhighnumber
 *  
 *  Special thanks to @senkunmusahi, who created the artwork bitmaps in the game using https://www.riyas.org/2013/12/online-led-matrix-font-generator-with.html
 *  
 *  Inspired by http://webboggles.com/ and includes some code from the #AttinyArcade games on that site
 *  The code that does not fall under the licenses of sources listed below can be used non commercially with attribution.
 *  This software is supplied without warranty of any kind.
 *  
 *  Controls:
 *  On the standard AttinyArcade:
 *  LEFT and RIGHT buttons move the frog across 
 *  BOTH BOTTONS TOGETHER move the frog forwards
 *  
 *  HIGHLY RECOMMENDED:
 *  On custom hardware (see schematic in folder where you found this file) there is an additional button to move frog forward  
 *  
 *  Also, from standby....
 *  Press and hold left button to turn sound on and off
 *  Press and hold left button with the right button held to reset high score
 * 
 *  This sketch is using the screen control and font functions written by Neven Boyanov for the http://tinusaur.wordpress.com/ project
 *  Source code and font files available at: https://bitbucket.org/tinusaur/ssd1306xled
 *  **Note that this highly size-optimised version requires modified library functions (which are in this source code file) 
 *  and a modified font header
 * 
 *  Sleep code is based on this blog post by Matthew Little:
 *  http://www.re-innovation.co.uk/web12/index.php/en/blog-75/306-sleep-modes-on-attiny85
*/
#include <EEPROM.h>
#include "font6x8AJ2.h"
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/interrupt.h> // needed for the additional interrupt

// Uncomment this #define to make the logs smaller (/thinner)
//#define SMALLLOGS

// Make click delay an even number - it gets halved and then used in an integer comparison
#define CLICKDELAY 120 

// The basline speed - higher number is slower
#define MOVEBASE 1000 

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

// Function prototypes

// Drawing functions - adapted from those at https://bitbucket.org/tinusaur/ssd1306xled
void ssd1306_init(void);
void ssd1306_xfer_start(void);
void ssd1306_xfer_stop(void);
void ssd1306_send_byte(uint8_t byte);
void ssd1306_send_command(uint8_t command);
void ssd1306_send_data_start(void);
void ssd1306_send_data_stop(void);
void ssd1306_setpos(uint8_t x, uint8_t y);
void ssd1306_fillscreen(uint8_t fill_Data);
void ssd1306_char_f6x8(uint8_t x, uint8_t y, const char ch[]);
void ssd1306_draw_bmp(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t bitmap[]);

// Custom draw functions - allow for extra functionality like inverse display
void sendBlock(byte, bool);
void sendByte(byte, bool);

// Other generic functions for games (both originated in code from webboggles.com and the sleep code is by Matthew Little - see above)
void beep(int,int);
void system_sleep(void);
void doNumber (int,int,int);

// Game functions
void playFrogger(void);
void levelUp(int);
void moveBlocks(void);
void initScreen(void);
void drawDocks(void);
void drawLives(void);
void displayTitle(void);
void resetDock(byte);
void checkCollision(void);

// Global variables - yes I know all these global vars is a lazy way to code but it makes it easier to prevent stack overflows when you're working with 512 bytes! 
// Most of these are initialised in the main game function (playFrogger())
int watchDog;             // Counts drawing cycles so I can shut the game down if there's inactivity - battery saver!
boolean stopAnimate;      // this is set to 1 when a collision is detected
int lives;                // Lives in the game - this can go negative to end the game, which is why it's a signed variable  
bool frogDocks[5];        // Tracks which frog docks are full (at the top of the screen)
bool flipFlop;            // Used in routines that flip-flop between two states (left and right)
bool flipFlopShift;       // Same as previous one
byte frogColumn;          // Column location of frog (there are 16 altogether)
byte frogRow;             // Row locaiton of frog (there are 8, but 0 is the frog docks at the top and 7 is the start row)
byte frogLeftLimit;       // Left limit of frog travel on start row (changes as digits in score increases)
byte frogRightLimit;      // Right limit of frog travel on start row (changes as lives decrease as there's then more space)
byte level;               // Level - starts at 1
byte blockShiftL;         // Number of pixels to shift the left-going rows by
byte blockShiftR;         // Number of pixels to shift the right-going rows by
int interimStep;          // Used as timer for incremental movements
int moveDelay;            // How long to wait until the next movement of logs etc - changes as levels increase to make the game go faster
int dockedFrogs;          // How many frogs are in the docks at the top
unsigned long clickBase;  // Timer for debounce
boolean clickLock;        // For debounce routine
int score;                // Obvious I hope
int topScore;             // High score
boolean newHigh;          // Is there a new high score?
boolean mute = 0;         // Mute the speaker
byte grid[6][16];         // Grid for items like logs, crocs, cars and lorries
byte frogMode;            // Represents the frog direction 
bool moveForward=0;       // Captures when the 'forward' button is pressed
bool moveLeft=0;          // Captures when the 'left' button is pressed
bool moveRight=0;         // Captures when the 'right' button is pressed

// Bitmaps created by @senkunmusahi using https://www.riyas.org/2013/12/online-led-matrix-font-generator-with.html
static const byte  bitmaps[15][8] PROGMEM = {
// Frogs
  {0x83, 0xDC, 0x7A, 0x3F, 0x3F, 0x7A, 0xDC, 0x83},
  {0x99, 0xBD, 0xDB, 0x7E, 0x7E, 0x3C, 0xE7, 0x81},
  {0x81, 0xE7, 0x3C, 0x7E, 0x7E, 0xDB, 0xBD, 0x99},

#ifdef SMALLLOGS
// Small logs
  {0x1C, 0x22, 0x41, 0x55, 0x55, 0x51, 0x43, 0x61},
  {0x69, 0x6B, 0x43, 0x61, 0x45, 0x45, 0x61, 0x65},
  {0x45, 0x55, 0x41, 0x5D, 0x63, 0x5D, 0x22, 0x1C},
#else
// Bigger logs
  {0x3C, 0x7E, 0xD7, 0xB5, 0xAD, 0xBF, 0xFF, 0xED}, 
  {0xAD, 0xAD, 0xFF, 0xB7, 0xF5, 0xBF, 0xB7, 0xAD}, 
  {0xED, 0xBD, 0xC3, 0xBD, 0xA5, 0xBD, 0x42, 0x3C},  
#endif

// Trucks
  {0x00, 0x7F, 0x41, 0x55, 0x55, 0x55, 0x55, 0x55},
  {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}, 
  {0x41, 0x7F, 0x22, 0x7F, 0x7F, 0x63, 0x22, 0x1C},

// Crocs
  {0x41, 0x63, 0x46, 0x6E, 0x7C, 0x7E, 0x7A, 0x3E},
  {0xBC, 0xFE, 0x7E, 0x3E, 0xBE, 0xBE, 0xFC, 0x7C},  
  {0x78, 0x38, 0x38, 0x38, 0x70, 0x60, 0x60, 0x40},

// Cars
  {0x00, 0x1C, 0x22, 0x63, 0x7F, 0x7F, 0x22, 0x22},
  {0x22, 0x3E, 0x3E, 0x7F, 0x63, 0x63, 0x22, 0x1C},
  {0x22, 0x3E, 0x3E, 0x7F, 0x63, 0x63, 0x22, 0x1C}
  };

// Opening artwork created by @senkunmusahi using https://www.riyas.org/2013/12/online-led-matrix-font-generator-with.html
static const byte titleBmp[] PROGMEM = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xF0, 0x7C, 0x06, 0x73, 0x59, 0x43,
0x06, 0x3C, 0x38, 0x30, 0x30, 0x38, 0x3E, 0x26, 0x7B, 0x59, 0x43, 0x06, 0x7C, 0xF0, 0xC0, 0x80,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0xFF,
0xCF, 0x01, 0x00, 0x00, 0x30, 0x60, 0xE0, 0xC0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xC0,
0xC0, 0x60, 0x30, 0x00, 0x00, 0x00, 0x01, 0xCF, 0xFE, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x7C, 0xFE, 0x86, 0x0E, 0x0E, 0x1C, 0x18, 0x31, 0x7F, 0xFE, 0xFC, 0x1C, 0x18, 0x38, 0x38, 0x38,
0x39, 0x39, 0x39, 0x39, 0x39, 0x39, 0x39, 0x38, 0x38, 0x38, 0x38, 0x18, 0x1C, 0xFC, 0xFE, 0x7F,
0x39, 0x18, 0x1C, 0x0E, 0x0E, 0xC6, 0xFE, 0x3C, 0x00, 0x01, 0x07, 0x0E, 0x1C, 0x38, 0x70, 0xC0,
0xC0, 0x80, 0x03, 0x07, 0xFC, 0xF8, 0x00, 0x00, 0xF0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xE0, 0xF0,
0x00, 0x00, 0xF8, 0xFC, 0x0F, 0x03, 0x80, 0xC0, 0xE0, 0x70, 0x38, 0x1C, 0x0E, 0x03, 0x01, 0x00,
0x04, 0x06, 0x0F, 0x0F, 0x06, 0x06, 0x03, 0x03, 0x03, 0x63, 0x73, 0x33, 0x3B, 0xFF, 0xFF, 0x7F,
0x3F, 0x38, 0xF0, 0xC0, 0x00, 0xF0, 0xF8, 0x3F, 0x7F, 0xFF, 0xFF, 0x3B, 0x33, 0x63, 0x63, 0x03,
0x03, 0x03, 0x06, 0x06, 0x0F, 0x0F, 0x06, 0x00, 
};

// Interrupt handlers
ISR(PCINT0_vect){ // PB0 pin button interrupt           
  if (clickLock == 0) {
    moveLeft = 1;
    clickLock = 1;
    clickBase = millis();
  }
}

void playerIncFrogger(){ // PB2 pin button interrupt
  if (clickLock == 0) {
    moveRight = 1;
    clickLock = 1;
    clickBase = millis();
  }
}

void displayTitle(void) {
  int incr = 0;
  for(int lxn = 2; lxn < 7; lxn++) {
    ssd1306_setpos(85,lxn); 
    ssd1306_send_data_start();
    for(int lxn2 = 0; lxn2 < 40; lxn2++) {
      ssd1306_send_byte(pgm_read_byte(&titleBmp[incr]));      
      incr++;
    }
    ssd1306_send_data_stop();                    
  }
}

// Arduino stuff - setup
void setup() {
  DDRB = 0b00000010;    // set PB1 as output (for the speaker)
  PCMSK = 0b00000001; // pin change mask: listen to portb bit 1
  GIMSK |= 0b00100000;  // enable PCINT interrupt 
  sei();          // enable all interrupts
}

// Arduino stuff - loop
void loop() { 
  ssd1306_init();
  ssd1306_fillscreen(0x00);

  // The lower case character set is seriously compromised because I've had to truncate the ASCII table
  // to release space for executable code.
  // There is no z in the table as this isn't used anywhere in the text here and most of the 
  // symbols are also missing for the same reason (see my hacked version of font6x8.h - font6x8AJ.h for more detail)
  ssd1306_char_f6x8(0, 2, "F R O G G E R");
  ssd1306_char_f6x8(0, 4, "andy jackson"); // see comments above !

  ssd1306_setpos(0,1); 
  for (int incr = 0; incr < 80; incr++) {
    ssd1306_send_data_start();
    ssd1306_send_byte(B00111000);
    ssd1306_send_data_stop();                    
  }
  ssd1306_setpos(0,3); 
  for (int incr = 0; incr < 80; incr++) {
    ssd1306_send_data_start();
    ssd1306_send_byte(B00011100);
    ssd1306_send_data_stop();                    
  }

  displayTitle();

  ssd1306_char_f6x8(0, 6, "inspired by");
  ssd1306_char_f6x8(0, 7, "webboggles.com");
  delay(1500);
  ssd1306_char_f6x8(0, 6, "artwork by ");
  ssd1306_char_f6x8(0, 7, "zsenkunmusashi");  // see comments above - f has been replaced by @ in the ASCII table

  long startT = millis();
  long nowT =0;
  boolean sChange = 0;

  while(digitalRead(0) == HIGH) {
    nowT = millis();
    if (nowT - startT > 2000) {
      sChange = 1;     
      if (digitalRead(2) == HIGH) {
        EEPROM.write(0,0);
        EEPROM.write(1,0);
        ssd1306_char_f6x8(8, 0, "-HIGH SCORE RESET-");  
      } else if (mute == 0) { mute = 1; ssd1306_char_f6x8(32, 0, "-- MUTE --"); } else { mute = 0; ssd1306_char_f6x8(31, 0, "- SOUND ON -");  }    
      break;
    }
    if (sChange == 1) break;
  }  
  while(digitalRead(0) == HIGH);

  if (sChange == 0) {
    delay(2000);

    ssd1306_init();
    ssd1306_fillscreen(0x00);

    playFrogger(); 

    topScore = EEPROM.read(0);
    topScore = topScore << 8;
    topScore = topScore |  EEPROM.read(1);

    newHigh = 0;
    if (score > topScore) { 
      topScore = score;
      EEPROM.write(1,score & 0xFF); 
      EEPROM.write(0,(score>>8) & 0xFF); 
      newHigh = 1;
      }

    ssd1306_fillscreen(0x00);
    ssd1306_char_f6x8(11, 1, "----------------");
    ssd1306_char_f6x8(11, 2, "G A M E  O V E R");
    ssd1306_char_f6x8(11, 3, "----------------");
    ssd1306_char_f6x8(37, 5, "SCORE:");
    doNumber(75, 5, score);
    if (!newHigh) {
      ssd1306_char_f6x8(21, 7, "HIGH SCORE:");
      doNumber(88, 7, topScore);
    }
    delay(1000);
    if (newHigh) {
      ssd1306_fillscreen(0x00);
      ssd1306_char_f6x8(10, 1, "----------------");
      ssd1306_char_f6x8(10, 3, " NEW HIGH SCORE ");
      ssd1306_char_f6x8(10, 7, "----------------");
      doNumber(50,5,topScore);
      for (int i = 700; i>200; i = i - 50){
      beep(30,i);
      }
      delay(1200);    
    } 
  }
  system_sleep();
}

void doNumber (int x, int y, int value) {
    char temp[10] = {0,0,0,0,0,0,0,0,0,0};
    itoa(value,temp,10);
    ssd1306_char_f6x8(x, y, temp);
}

void ssd1306_init(void){
  DDRB |= (1 << SSD1306_SDA); // Set port as output
  DDRB |= (1 << SSD1306_SCL); // Set port as output

  ssd1306_send_command(0xAE); // display off
  ssd1306_send_command(0x00); // Set Memory Addressing Mode
  ssd1306_send_command(0x10); // 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
  ssd1306_send_command(0x40); // Set Page Start Address for Page Addressing Mode,0-7
  ssd1306_send_command(0x81); // Set COM Output Scan Direction
  ssd1306_send_command(0xCF); // ---set low rowumn address
  ssd1306_send_command(0xA1); // ---set high rowumn address
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

void ssd1306_xfer_start(void){
  DIGITAL_WRITE_HIGH(SSD1306_SCL);  // Set to HIGH
  DIGITAL_WRITE_HIGH(SSD1306_SDA);  // Set to HIGH
  DIGITAL_WRITE_LOW(SSD1306_SDA);   // Set to LOW
  DIGITAL_WRITE_LOW(SSD1306_SCL);   // Set to LOW
}

void ssd1306_xfer_stop(void){
  DIGITAL_WRITE_LOW(SSD1306_SCL);   // Set to LOW
  DIGITAL_WRITE_LOW(SSD1306_SDA);   // Set to LOW
  DIGITAL_WRITE_HIGH(SSD1306_SCL);  // Set to HIGH
  DIGITAL_WRITE_HIGH(SSD1306_SDA);  // Set to HIGH
}

void ssd1306_send_byte(uint8_t byte){
  uint8_t i;
  for(i=0; i<8; i++)
  {
    if((byte << i) & 0x80)
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

void ssd1306_send_command(uint8_t command){
  ssd1306_xfer_start();
  ssd1306_send_byte(SSD1306_SA);  // Slave address, SA0=0
  ssd1306_send_byte(0x00);  // write command
  ssd1306_send_byte(command);
  ssd1306_xfer_stop();
}

void ssd1306_send_data_start(void){
  ssd1306_xfer_start();
  ssd1306_send_byte(SSD1306_SA);
  ssd1306_send_byte(0x40);  //write data
}

void ssd1306_send_data_stop(void){
  ssd1306_xfer_stop();
}

void ssd1306_setpos(uint8_t x, uint8_t y)
{
  if (y>7) return;
  ssd1306_xfer_start();
  ssd1306_send_byte(SSD1306_SA);  //Slave address,SA0=0
  ssd1306_send_byte(0x00);  //write command

  ssd1306_send_byte(0xb0+y);
  ssd1306_send_byte(((x&0xf0)>>4)|0x10); // |0x10
  ssd1306_send_byte((x&0x0f)|0x01); // |0x01

  ssd1306_xfer_stop();
}

void ssd1306_fillscreen(uint8_t fill_Data){
  uint8_t m,n;
  for(m=0;m<8;m++)
  {
    ssd1306_send_command(0xb0+m); //page0-page1
    ssd1306_send_command(0x00);   //low rowumn start address
    ssd1306_send_command(0x10);   //high rowumn start address
    ssd1306_send_data_start();
    for(n=0;n<128;n++)
    {
      ssd1306_send_byte(fill_Data);
    }
    ssd1306_send_data_stop();
  }
}

void ssd1306_char_f6x8(uint8_t x, uint8_t y, const char ch[]){
  uint8_t c,i,j=0;
  while(ch[j] != '\0')
  {
    c = ch[j] - 32;
    if (c >0) c = c - 12;
    if (c >15) c = c - 6;
    if (c>40) c=c-9;
    if(x>126)
    {
      x=0;
      y++;
    }
    ssd1306_setpos(x,y);
    ssd1306_send_data_start();
    for(i=0;i<6;i++)
    {
      ssd1306_send_byte(pgm_read_byte(&ssd1306xled_font6x8[c*6+i]));
    }
    ssd1306_send_data_stop();
    x += 6;
    j++;
  }
}

void system_sleep(void) {
  ssd1306_fillscreen(0x00);
  ssd1306_send_command(0xAE);
  cbi(ADCSRA,ADEN);                    // switch analog to digital converter off
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  sleep_mode();                        // system actually sleeps here
  sleep_disable();                     // system continues execution here when watchdog timed out 
  sbi(ADCSRA,ADEN);                    // switch analog to digital converter on
  ssd1306_send_command(0xAF);
}

void beep(int bCount,int bDelay){
  if (mute) return;
  for (int i = 0; i<=bCount; i++){digitalWrite(1,HIGH);for(int i2=0; i2<bDelay; i2++){__asm__("nop\n\t");}digitalWrite(1,LOW);for(int i2=0; i2<bDelay; i2++){__asm__("nop\n\t");}}
}


/* ------------------------
 *  Frogger main game code
 */
void playFrogger(){
  stopAnimate = 0;
  score = 0;
  moveDelay = MOVEBASE;
  level = 1;
  frogColumn = 8;
  frogRow = 7;
  clickLock = 0;
  frogMode = 1;
  interimStep =0;
  blockShiftL = 0;
  blockShiftR = 0;
  flipFlop = 1;
  flipFlopShift = 1;
  dockedFrogs = 0;
  lives = 2;
  frogRightLimit = 12;
  watchDog = 1; // we use this to see if there's been movement - it's only ever zero when the frog has just moved!

  attachInterrupt(0,playerIncFrogger,CHANGE);

  initScreen();
  resetDock(0);

  drawFrog(frogMode,0);
  drawGameScreen(frogMode);
  drawLives();
  drawDocks();
  
  doNumber(0,7,score);
  
  while (lives >= 0) {
    interimStep++;

    if (watchDog >= 500) lives = -1; // Stop the game if nothing's happening - maybe triggered in someone's pocket so this is to save battery!
    
    // Calculate left limit of frog movement so it doesn't hit the score
    frogLeftLimit = 1;
    if ((score / 10) % 10 != 0) frogLeftLimit++; 
    if ((score / 100) % 10 != 0) frogLeftLimit++;
    if ((score / 1000) % 10 != 0) frogLeftLimit++;

    // Move stuff along if it's time to
    if (interimStep > moveDelay/8) {
      watchDog++;
      blockShiftL++;
      if (flipFlopShift == 1) flipFlopShift = 0; else flipFlopShift = 1;
      if (flipFlopShift == 1) blockShiftR++;
      if (blockShiftL == 7) {
        moveBlocks();
        blockShiftL = 0;
      }
      if (blockShiftR == 7) {
        blockShiftR = 0;
      }
      interimStep = 0;
      checkCollision();
      if (stopAnimate == 0) {
        drawGameScreen(frogMode);
        drawFrog(frogMode,0);
      }
    }

    // Handle input from 'jump' button (the other two buttons are captured in the interrupt routines)
    if (analogRead(0) < 940 && clickLock == 0) {
      moveForward = 1;
      watchDog = 0;   // reset the watchdog so the game doesn't end!
      clickLock = 1;
      clickBase = millis();
    }

    // Handle moving left
    if(moveLeft == 1 && millis() > clickBase + CLICKDELAY/2) {
      watchDog = 0;   // reset the watchdog so the game doesn't end!
      moveLeft = 0;
      if (digitalRead(2) == HIGH) moveForward = 1; else {
        drawFrog(0,0);  // delete the frog
        // move the frog, checking it isn't jumping off the edge of the screen
        if ((frogRow == 7 && frogColumn > frogLeftLimit) || (frogRow < 7 && frogColumn > 0)) {
          frogColumn --;
        } else if (frogRow < 7) stopAnimate = 1;
        frogMode = 2; // pointing left
      }
    }

    // Handle moving right
    if(moveRight == 1 && millis() > clickBase + CLICKDELAY/2){
      watchDog = 0;   // reset the watchdog so the game doesn't end!
      moveRight = 0;
      if (digitalRead(0) == HIGH) moveForward = 1; else {
        drawFrog(0,0); // delete the frog
        // move the frog, checking it isn't jumping off the edge of the screen
        if ((frogRow == 7 && frogColumn < frogRightLimit) || (frogRow < 7 && frogColumn < 14)) {
          frogColumn ++;
        } else if (frogRow < 7) stopAnimate = 1;
        frogMode = 3; // pointing right    
      }
    }

    // Handle 'move forward' button press
    if (moveForward == 1) {
      moveForward = 0;
  
      score+= level;          // increment the score for every move
      doNumber(0,7,score);    // display new score
      drawFrog(0,0);          // delete the frog
  
      if (frogRow > 1) {
        frogRow--; 
        // Correct for the skew in frog position created by the blockShift scrolling parameter
        if (frogRow == 3 && blockShiftL < 4) frogColumn--;
        if (frogRow == 2 && blockShiftR + blockShiftL < 5) frogColumn++;
        if (frogRow == 1 && blockShiftR + blockShiftL < 5) frogColumn--;
                
      } else {
        // frog is at the docks!
        if (blockShiftL < 4 && frogColumn <15) frogColumn++;  // account for skew due to block shifting
        byte dockPos = (byte)floor(frogColumn/3);
        if (frogDocks[dockPos] == 0 ) {
          dockedFrogs++;          
          frogDocks[dockPos] = 1;                             // assign this dock as filled
          frogRow = 7;                                        // reposition the frog at the start
          frogColumn = 8;
          for (int i = 1000; i>200; i = i - 100){             // make sound
          beep(10,i);
          drawDocks();                                        // redraw the docks
          }
        } else stopAnimate = 1;
      }
      frogMode = 1;             // mode 1 = forwards position

      // check if all docks are full - if so, then level up!
      if (dockedFrogs >= 5) {
        level++;
        levelUp(level);
        if (moveDelay > 99) moveDelay -=100; // make the game speed up 
        initScreen();                        // reinitalise the position of game items
        resetDock(0);                        // reinitliase the dock
        dockedFrogs = 0;              
        drawDocks();                         // display the (now empty) docks
        drawLives();                         // display the lives
        doNumber(0,7,score);                 // display the score
      }      
    }

    // The frog has moved 
    if (watchDog == 0 && stopAnimate == 0) {
      watchDog = 1;               // set to something other than zero so this routine doesn't run again
      // redraw the frog
      drawFrog(frogMode,0);
      // redraw the screen
      drawGameScreen(frogMode);
      // make jump sound
      beep(30,400);
      beep(30,300);
      beep(30,200);
    }
    
    checkCollision();
    
    if (clickLock == 1 && millis() > clickBase + CLICKDELAY && digitalRead(2)==0 && digitalRead(0)==0 && analogRead(0) > 940) clickLock = 0; // normal debounce

    // check to see if the frog has been killed
    if (stopAnimate != 0) {
      // redraw the screen
      drawGameScreen(frogMode);
      // animation for frog death
      drawFrog(0,1);
      for (int i = 0; i<250; i = i+ 50){  
        beep(50,i);
      }
      drawFrog(frogMode,1);    
      for (int i = 250; i<500; i = i+ 50){  
        beep(50,i);
      }
      drawFrog(0,1);
      for (int i = 500; i<750; i = i+ 50){  
        beep(50,i);
      }
      drawFrog(frogMode,1);
      for (int i = 750; i<1000; i = i+ 50){  
        beep(50,i);
      }
      delay(600);
      lives--;          // increment the score for every move
      frogRightLimit++; // there's one less frog drawn on right so you can move a bit further across (if you really want to!)
      stopAnimate = 0;  // reset parameter
      drawLives();      // display number of lives left
      frogColumn = 8;   // reinitalise frog location
      frogRow = 7;
      }
  }  // Big while loop (main game loop) goes until lives is negative
} 

void checkCollision(void) {
  if (frogRow > 0 && frogRow < 4 && grid[frogRow-1][frogColumn] == 0) stopAnimate = 1; // the frog has fallen in the river
  if (frogRow > 0 && frogRow < 4 && grid[frogRow-1][frogColumn] > 9) stopAnimate = 1; // the frog has stepped on a croc
  if ((frogRow < 7 && frogRow > 3) && (grid[frogRow-1][frogColumn] != 0 || grid[frogRow-1][frogColumn-1] != 0)) stopAnimate = 1; // the frog has been hit by a vehicle
}

// Initialise all the moving objects on the game screen
void initScreen(void) {
  int initCounter[6] = {3,2,4,2,2,3};       // the length of the objects on each row - doesn't change
  int gapCounter[6] = {-2,-3,-4,-4,-3,-5};  // the gaps between objects - change with levels to make it harder as you go thru the game
  int counter[6];                           // used to hold the gap data
  byte stepMode = 0;                        // which component of the object are we drawing (they all have three - a start a middle and an end)
  byte stepShift = 0;                       // offset to shift up to the different objects in the array
  byte crocStartColumn = 0;                 // column at which to stop drawing crocs - is zero at start hence no crocs!

  
  // Adjust difficulty by changing gaps between objects according to level
  if (level == 1) {
    gapCounter[5] = -14;           // easiset setting, for start of game
  }
  if (level < 3) {
    gapCounter[4] = -6;            // make it easier for levels less than 3 by increasing the gap in the cars on this row
  }
  if (level < 4) {
    gapCounter[3] = -7;
  }
  if (level > 4) {
    for (byte incr = 1; incr < 3; incr++) {
      gapCounter[incr]--;         // increase the gaps between the logs for levels over 4  
    }
  }
  if (level > 7) {                // set smaller gaps between cars for levels over 7
    gapCounter[3] = -4; 
    gapCounter[4] = -2;
    gapCounter[5] = -3;
  }
  if (level > 2) crocStartColumn = 5; // one croc appears at level 3 and above
  if (level > 6) crocStartColumn = 9; // two croc appear at level 7 and above
  
  // Initialise the counters
  for (byte incr = 0; incr < 6;incr++) counter[incr] = initCounter[incr];  

  // Initialise array with zeros
  for (byte col = 0; col < 16; col++) {
    for (byte row = 0; row < 6; row++) {
      grid[row][col] = 0;
    }
  }
       
  stepMode = 0;
  // Initialise array with obstacles
  for (byte row = 0; row < 6; row++) {
    for (byte col = 0; col < 15; col++) {         
      if (counter[row] > 0) {
        if (14-row > counter[row]) {
          if (counter[row] == 1) if (stepMode == 1) stepMode = 2; // the next space is blank and we are drawing the middle - draw the end! 
          if (row > 2) stepShift = 3; else stepShift = 0;         // shift up to the trucks in the array
          if (row == 4) stepShift = 9;                            // shift up to the cars in the array - also theres no middle
          
          if (row > 0) {
            grid[row][col] = 4+stepMode+stepShift;                // if you are on any row but the first - draw whatever is appropriate from the bitmaps
          } else if (col >= crocStartColumn) {                    
            grid[row][col] = 4+stepMode+stepShift;                // if you're on row zero (top row of logs) and you are above where crocs should be drawm, draw logs ...
          } else grid[row][col] = 10+stepMode;                    // .. otherwise draw crocs
          if (stepMode == 0) stepMode = 1;                        // we've drawn the left side now switch to central sections
          if (stepMode == 2) stepMode = 0;                        // we've drawn the end, now reset
        }
      } 
      counter[row]--;                                             // decrement the counter
      if (counter[row] <= gapCounter[row]) {
        counter[row] = initCounter[row];                          // if we have gone negative enough to account for the gaps - reset the counter and start again
      }
    }
  }
}

// Display the frog
void drawFrog(byte mode, bool frogDead) {
  if (frogRow > 6 || frogRow < 1 || frogDead == 1) {           // don't draw the frog when it's on the road or on logs - because they are moving, that's handled in the main drawing routine below- exception is when you are animating frog death
    if (frogRow == 1 || frogRow == 3) {                        // these allow for the blocks being shifted when animating the frog death on rows with logs
      ssd1306_setpos(frogColumn*8 + 7 - blockShiftL,frogRow);
    } else if (frogRow == 2) {
      ssd1306_setpos(frogColumn*8 + blockShiftR,frogRow);      
    } else {
      ssd1306_setpos(frogColumn*8,frogRow); 
    }
    ssd1306_send_data_start();
    sendBlock(mode,0);                   // draw the frog - mode is direction
    ssd1306_send_data_stop();                    
  }    
}

// Display the frog and all the moving items on the screen
void drawGameScreen(byte mode) {
  bool inverse = 0;
  
  // Draw objects going left
  for (byte row = 0; row < 6; row+=2) {
    if (row >=0 && row < 3) inverse = 1; else inverse = 0;                              // draw everything (except the frog) in inverse video on the river rows (0,1,2)
    ssd1306_setpos(0,row+1);                                                            // +1 because row 0 here is actually row 1 on the screen
    ssd1306_send_data_start();
    for (byte incr = 0; incr < 7-blockShiftL; incr++) if (grid[row][15] == 0) {         // cover the tiny bit to the far left of the screen up to wherever the main blocks will be drawn (depends on how far they are shifted)
      sendByte(0,inverse);                                                              // draw an empty 8-bit line if there's nothing wrapping around
    } else {
      sendByte(pgm_read_byte(&bitmaps[grid[row][15]-1][1+blockShiftL+incr]), inverse);  // pick the correct bit of whatever is wrapping from the right of the screen
    }
    for (byte col = 0; col < 15; col++) {         
      if (frogRow == row+1 && frogColumn == col && frogRow < 4 && frogRow > 0) {
        sendBlock(mode,0);                                                                          // if we are in a location with the frog, and it's on the logs, draw it - never invert it (hence zero as second parameter here)
      } else if (stopAnimate == 0 && frogRow == row+1 && frogColumn == col + 1 && frogRow > 3 && frogRow < 7) {         // frog is amongst the cars and needs drawing
        for (byte incr = 0; incr < blockShiftL; incr++) sendByte(0,0);                              // draw the blank space up to the frog
        sendBlock(mode,0);                                                                          // draw frog
        for (byte incr = 0; incr < 7-blockShiftL; incr++) sendByte(0,0);                            // draw the blank space after the frog
        col++;                                                                                      // we've now drawn two columns so increment
      } else {
        sendBlock(grid[row][col],inverse);                                                          // draw the correct object for this space - it's not a frog ;)
      }
    }
    // fill in the bit to the right of the main blocks 
    for (byte incr = 0; incr < blockShiftL; incr++) if (grid[row][15] == 0) sendByte(0,inverse); else sendByte(pgm_read_byte(&bitmaps[grid[row][15]-1][incr]),inverse);

    ssd1306_send_data_stop();
  }
  if (frogColumn == 0) drawFrog(mode,1); // this covers the exceptional case where the frog is in the far left colum, in which case the normal routine can't draw it when it's on the road
  
  // Draw objects going right - see comments above, works in basically the same way
  for (byte row = 1; row < 6; row+=2) {
    if (row > 0 && row < 3) inverse = 1; else inverse = 0;
    ssd1306_setpos(0,row+1);
    ssd1306_send_data_start();
    for (byte incr = 0; incr < blockShiftR; incr++) if (grid[row][15] == 0) sendByte(0, inverse); else sendByte(pgm_read_byte(&bitmaps[grid[row][15]-1][incr+(8-blockShiftR)]),inverse);
    for (byte col = 0; col < 15; col++) {         
      if (frogRow == row+1 && frogColumn == col && frogRow < 4 && frogRow > 0) {
        sendBlock(mode,0);    
      } else if (stopAnimate == 0 && frogRow == row+1 && frogColumn == col + 1 && frogRow > 3 && frogRow < 7) {  
        for (byte incr = 0; incr < 7-blockShiftR; incr++) sendByte(0,0);
        sendBlock(mode,0); // draw frog
        for (byte incr = 0; incr < blockShiftR; incr++) sendByte(0,0);
        col++;        
      } else {
        sendBlock(grid[row][col],inverse);        
      }
    }
    for (byte incr = 0; incr < 7-blockShiftR; incr++) if (grid[row][15] == 0) sendByte(0,inverse); else sendByte(pgm_read_byte(&bitmaps[grid[row][15]-1][incr]),inverse);
    ssd1306_send_data_stop();
  }
  if (frogColumn == 0) drawFrog(mode,1);
}

// Send one byte to the screen
void sendByte(byte fill, bool inverse) {
  if (inverse == 0) ssd1306_send_byte(fill); else ssd1306_send_byte(~fill);
}

// Send one block of 8 bytes to the screen - inverse means inverse video, for the river section
void sendBlock(byte fill, bool inverse){
  for (int incr = 0; incr < 8; incr++) {
    if (fill > 0) {
      if (inverse == 0) ssd1306_send_byte(pgm_read_byte(&bitmaps[fill-1][incr])); else ssd1306_send_byte(~pgm_read_byte(&bitmaps[fill-1][incr]));  
    } else if (inverse ==0) ssd1306_send_byte(0); else ssd1306_send_byte(0xFF); 
  }
}

// Draw the frog lives (in the right hand corner)
void drawLives(void) {
  byte tempRow = frogColumn;
  byte tempCol = frogRow;
  frogRow = 7;

  for (int incr = 2; incr > 0; incr--) {
    frogColumn = 15-incr;
    drawFrog(0,1);
  }

  for (int incr = lives; incr > 0; incr--) {
    frogColumn = 15-incr;
    drawFrog(1,1);
  }
  frogRow = tempCol;
  frogColumn = tempRow;    
}

// Draw the docks for the frog to land in at top of screen
void drawDocks(void) {
  byte drawPos = 3;
  for (byte incr = 0; incr < 5; incr++) {
    ssd1306_setpos(drawPos,0);
    ssd1306_send_data_start();
    ssd1306_send_byte(B11111111);
    ssd1306_send_byte(B00000001);
    ssd1306_send_byte(B00000001);
    if (frogDocks[incr] == 1) sendBlock(1,0); else for(byte lxn = 0; lxn < 8; lxn++) ssd1306_send_byte(B00000001);
    ssd1306_send_byte(B00000001);
    ssd1306_send_byte(B00000001);
    ssd1306_send_byte(B11111111);
    ssd1306_send_data_stop();
    drawPos+= 24;
    }    
}

// Set all the frog docks to a single value
void resetDock(byte value) {   for (byte incr = 0; incr < 5;incr++) frogDocks[incr] = value; }

// Handle what happens at the end of a level
void levelUp(int number) {  
  // Flash the frog docks
  delay(200);
  for (byte incr = 0; incr < 5; incr ++) {
    resetDock(0);
    drawDocks();
    for (int i = 800; i>200; i = i - 200){
    beep(20,i);
    }
    resetDock(1);
    drawDocks();
    for (int i = 800; i>200; i = i - 200){
    beep(20,i);
    }
  }  
  delay(500);
  ssd1306_fillscreen(0x00);
  ssd1306_char_f6x8(35, 1, "---------");
  ssd1306_char_f6x8(35, 3, " LEVEL ");
  ssd1306_char_f6x8(35, 5, "---------");
  doNumber(77,3,number);
  delay(1500);    
  ssd1306_fillscreen(0x00);
}

// Move all the items on the game screen (wrapping at the ends) and check for frog dropping off the end of the screen
void moveBlocks(void) {
  int direct = 0;

  if (flipFlop == 1) flipFlop = 0; else flipFlop = 1;
  
  for (byte row = 0; row < 6; row++) {
    // Move the frog along and check to see whether it's gone off the screen, in which case it dies
    if (frogRow < 4 && frogRow > 0) { 
      if (frogRow == row + 1) {
        if (direct == 1 && flipFlop == 1) {
          if (frogColumn >= 14) stopAnimate = 1; else frogColumn++; 
        } else if (direct == 0) {
          if (frogColumn < 1) stopAnimate = 1; else frogColumn--;
        }
      }  
    }
    if (direct == 0) { // move left
      byte temp = grid[row][0];
      for (byte col = 0; col < 15; col++) {         
        grid[row][col] = grid[row][col+1];
      }
      grid[row][15] = temp; // wrap around
      direct = 1;
    } else { // move right
      if (flipFlop == 1) {
        byte temp = grid[row][15];
        for (byte col = 15; col > 0; col--) {         
          grid[row][col] = grid[row][col-1];
          }
        grid[row][0] = temp; // wrap around
        }
      direct = 0;  
      }
    }
  }
