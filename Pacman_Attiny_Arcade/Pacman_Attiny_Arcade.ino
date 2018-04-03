/*  2015 / 2016 / 2017
 *  Pacman for Attiny Arcade by Andy Jackson - Twitter @andyhighnumber
 *  
 *  Inspired by http://webboggles.com/ and includes some code from the #AttinyArcade games on that site
 *  The code that does not fall under the licenses of sources listed below can be used non commercially with attribution.
 *  This software is supplied without warranty of any kind.
 *  
 *  **** BEFORE USE, BURN THE BOOTLOADER ON THE ATTINY85 WITH 16Mhz Internal Clock *  
 *  
 *  Controls:
 *  On the standard AttinyArcade:
 *  Tap left button to turn 90 degrees left (each tap cycles through directions, so if you are going left and you tap once, you will go down, tap twice and you'll be going right)
 *  Tap right button to turn 90 degrees right (each tap cycles through directions, so if you are going left and you tap once, you will go up, tap twice and you'll be going right)
 *  The arrow indicator on the far left of the screen indicates the currently-selected direction
 *  
 *  Also, from standby....
 *  Press and hold left button to turn sound on and off
 *  Press and hold left button with the right button held to reset high score
 * 
 *  No additional external libraries required for build. 
 *  
 *  Instructions for programming the Attiny85 can be found here: https://create.arduino.cc/projecthub/arjun/programming-attiny85-with-arduino-uno-afb829

 *  This sketch is using the screen control and font functions written by Neven Boyanov for the http://tinusaur.wordpress.com/ project
 *  Source code and font files available at: https://bitbucket.org/tinusaur/ssd1306xled
 *  **Note that this highly size-optimised version requires modified library functions (which are in this source code file) 
 *  and a modified font header
 * 
 *  Sleep code is based on this blog post by Matthew Little:
 *  http://www.re-innovation.co.uk/web12/index.php/en/blog-75/306-sleep-modes-on-attiny85
*/
#include <EEPROM.h>
#include "font6x8AJ3.h"
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/interrupt.h> // needed for the additional interrupt

// Debounce factor
#define CLICKDELAY 52

#define DIR_RIGHT 0
#define DIR_DOWN 1
#define DIR_LEFT 2
#define DIR_UP 3

#define PACMAN 1
#define GHOST1 2
#define GHOST2 4
#define GHOST3 8

// Probability of a ghost taking a branch to the side if it can
#define BRANCH_PROBABILITY 50     
// How many ghosts on level 1
#define INITIAL_GHOSTS 2          
// Initial number of cycles before a ghost is released - reduces according to next two defines down to a min level 
#define INITIAL_GHOST_DELAY 150  
#define GHOST_DELAY_REDUCTION 25
#define GHOST_DELAY_MINIMUM 50

// How long does powerUp mode last?
#define POWER_LENGTH 200

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
void playPacman(void);
void levelUp(int);
void displayTitle(void);
int checkCollision(byte xpos, byte ypos);

// Global variables - yes I know all these global vars is a lazy way to code but it makes it easier to prevent stack overflows when you're working with 512 bytes! 
// Most of these are initialised in the main game function (playPacman())
boolean stopAnimate;      // this is set to 1 when a collision is detected
int lives;                // Lives in the game - this can go negative to end the game, which is why it's a signed variable  
unsigned long clickBase;  // Timer for debounce
boolean clickLock;        // For debounce routine
int score;                // Obvious I hope
int topScore;             // High score
boolean newHigh;          // Is there a new high score?
boolean mute = 0;         // Mute the speaker
int pacX, pacY;           // Horiz and vertical position of pacman
int mouth = 0;            // Is his mouth open?
int directions[4];        // Directions of the four possible characters (pacman = 0, ghosts are 1-3)
int commandDir;           // The desired direction set by input
byte pillsEaten = 0;      // How many pills have been eaten (there are 63 altogether)
int ghostCounter = 0;     // How many ghosts 
int ghostsActive[3];      // How many ghosts are out?
int count = 0;            // Counter
int oldDir;               // Holds previous direction
int level = 0;            // Obvious I hope
int releaseDelay =0;      // The delay between ghosts being let out
int maxGhosts = 0;        // How many ghosts maximum on this level?
bool powerUp = 0;         // Is powerUp move active?
int powerCounter = 0;     // Counts the length of powerUp mode

byte ghostStart[3] = {28,17,39}; 

// Artwork 
static const byte pacOpen[4][8] PROGMEM = {
0x3C,0x7E,0xFF,0xFF, 0xE7,0xE7,0xC3,0x42, 
0x7C,0xFE,0x3F,0x0F,0x0F,0x3F,0xFE,0x7C,
0x42,0xC3,0xE7,0xE7,0xFF,0xFF,0x7E,0x3C,
0x3E,0x7F,0xFC,0xF0,0xF0,0xFC,0x7F,0x3E};

static const byte arrows[4][7] PROGMEM = {
0x10,0x10,0x10,0x7C,0x38,0x10,0x00,
0x00,0x20,0x60,0xFF,0x60,0x20,0x00,
0x10,0x38,0x7C,0x10,0x10,0x10,0x00,
0x00,0x04,0x06,0xFF,0x06,0x04,0x00
};

static const byte pacClosed[] PROGMEM = {0x3C,0x7E,0xFF,0xFF,0xFF,0xFF,0x7E,0x3C};

static const byte ghost[2][8] PROGMEM = {0x7C,0xFE,0x22,0xEB,0xFF,0x62,0xEA,0x7C,0x7C,0xC2,0x0A,0x83,0x83,0x0A,0x82,0x7C};

static const byte PAClives[] PROGMEM = {0x1C,0x3C,0x7E,0x66,0x66,0x62,0x40};

byte ghostLoc[3][2];
byte ghostDir[3];

byte pillsActive[64];
byte bigPillsActive[2];

static const byte bigPillLocations[2] PROGMEM = {14,119};

static const byte pillLocations[64][2] PROGMEM = {
14,9,
22,9,
29,9,
37,9,
46,9,
54,9,
62,9,
70,9,
79,9,
86,9,
97,9,
108,9,
119,9,

14,17,
29,19,
38,19,
46,19,
54,19,
86,19,
108,17,
119,17,

14,25,
22,25,
29,25,
41,27,
52,27,
62,27,
86,27,
96,26,
105,26,
113,26,
119,26,

14,38,
22,38,
29,38,
41,35,
52,35,
62,35,
86,35,
96,36,
105,36,
113,36,
119,36,

14,46,
29,44,
36,44,
46,44,
62,44,
86,44,
108,44,
119,44,

14,54,
22,54,
29,54,
37,54,
46,54,
54,54,
62,54,
70,54,
79,54,
86,54,
97,54,
108,54,
119,54
};

static const byte gameScreen[] PROGMEM = {
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF8,0x04,0xE4,0x14,0x14,0x14,0x14,0x14,0x14,
0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,
0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,
0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x13,0x10,0x0F,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x0F,0x10,0x13,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,
0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0xF4,0x04,0xF4,0x14,0x14,
0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,
0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,0xF4,0x04,0xF8,0x00,

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xC0,0x20,0xA0,0xA0,0xA0,0xA0,0x20,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x40,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,
0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0x40,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0xC0,0x20,0x20,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,
0xA0,0x20,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7F,0x80,0x9F,0xA0,0xA0,
0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0x20,0x20,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xC0,0x20,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0xFF,0x00,

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x1F,0x20,0x2F,0x2F,0x2F,0x2F,0x20,0x1F,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0xFF,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xFF,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x20,0x1F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x1F,0x20,0x1F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0xFF,0x00,

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0xFF,0x80,0x80,0x80,0x80,0x80,0x80,
0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xFE,0x01,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x09,0x06,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x01,0xFE,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0xFF,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x07,0xE8,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x01,0x3E,0x40,0x40,
0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x80,0xC0,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x3F,0x00,0xFF,0x00,

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0xFE,0x02,0x02,0x02,0x02,0x02,0x02,
0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x01,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x7F,0x80,0x7F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x90,0x60,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7F,0x80,0x7F,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0xFF,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xE0,0x17,0xE0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7F,0x80,0x7E,0x01,0x01,
0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0xFE,0x00,0xFF,0x00,

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xF8,0x04,0xF4,0xF4,0xF4,0xF4,0x04,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0xFF,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xFF,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x02,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xFC,0x02,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0xFF,0x00,

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x03,0x04,0x05,0x05,0x05,0x05,0x04,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x02,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,
0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x02,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x03,0x04,0x04,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,
0x05,0x04,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x01,0xFD,0x05,0x05,
0x05,0x05,0x05,0x05,0x05,0x05,0x04,0x04,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x03,0x04,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0xFF,0x00,

0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0x20,0x27,0x28,0x28,0x28,0x28,0x28,0x28,
0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,
0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,
0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0xC8,0x08,0xF0,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0xF0,0x08,0xC8,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,
0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x2F,0x20,0x2F,0x28,0x28,
0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,
0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x28,0x27,0x20,0x1F,0x00
};

// Interrupt handlers
ISR(PCINT0_vect){ // PB0 pin button interrupt           
  if (clickLock == 0) {
    if (commandDir > 0) commandDir--; else commandDir = 3;
    clickLock = 1;
    clickBase = millis();
  }
}

void playerIncPacman(){ // PB2 pin button interrupt
  if (clickLock == 0) {
    if (commandDir < 3) commandDir++; else commandDir = 0;
    clickLock = 1;
    clickBase = millis();
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

  // The lower case character set is missing because I've had to truncate the ASCII table
  // to release space for executable code.
  ssd1306_char_f6x8(20, 3, " P A C M A N");
  ssd1306_char_f6x8(24, 5, "ANDY JACKSON"); // see comments above !
  
  ssd1306_setpos(20,2); 
  for (int incr = 0; incr < 80; incr++) {
    ssd1306_send_data_start();
    ssd1306_send_byte(B00111000);
    ssd1306_send_data_stop();                    
  }
  ssd1306_setpos(20,4); 
  for (int incr = 0; incr < 80; incr++) {
    ssd1306_send_data_start();
    ssd1306_send_byte(B00011100);
    ssd1306_send_data_stop();                    
  }
  delay(1000);

  
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
    delay(1000);

    ssd1306_init();
    ssd1306_fillscreen(0x00);

    playPacman(); 

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
    delay(1500);
    if (newHigh) {
      ssd1306_fillscreen(0x00);
      ssd1306_char_f6x8(10, 1, "----------------");
      ssd1306_char_f6x8(10, 3, " NEW HIGH SCORE ");
      ssd1306_char_f6x8(10, 7, "----------------");
      doNumber(50,5,topScore);
      for (int i = 700; i>200; i = i - 50){
        beep(30,i);
      }
      delay(1500);    
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


void displayScreen(void) {
  int incr = 0;
  int pacDir = directions[0];
  byte out;

  for(byte c = 0; c < 8; c++) {
    ssd1306_setpos(0,c); 
    ssd1306_send_data_start();
    for (byte r = 0; r < 128; r++) {
      out = (pgm_read_byte(&gameScreen[incr]));

      if ( (r >= pacX) && (r < pacX + 8) ) { // draw the pacMan if any of his body is on this column
        if (c == pacY/8) {
          if (mouth == 1) {
            out |= (pgm_read_byte(&pacOpen[directions[0]][r-pacX]) << pacY % 8);            
          } else {
            out |= (pgm_read_byte(&pacClosed[r-pacX]) << pacY % 8);
          }
        } else if (c == pacY/8+1) {
          if (mouth == 1) {
            out |= (pgm_read_byte(&pacOpen[directions[0]][r-pacX]) >> (8- pacY % 8) );
          } else {
            out |= (pgm_read_byte(&pacClosed[r-pacX]) >> (8- pacY % 8) );
          }
        } 
      }     
      
      if (r < 7) {
        if (c == 2) out |= (pgm_read_byte(&arrows[commandDir][r]));            
        if (c == 5 && lives > 1) out |= (pgm_read_byte(&PAClives[r]));
        if (c == 6 && lives > 0) out |= (pgm_read_byte(&PAClives[r]));
      } 
      
      for (byte lxn = 0; lxn < 3; lxn++) {
        if ( (r >= ghostLoc[lxn][0]) && (r < ghostLoc[lxn][0] + 8) ) { // draw the ghost if any of his body is on this column
          if (c == ghostLoc[lxn][1]/8) {
            out |= (pgm_read_byte(&ghost[powerUp][r-ghostLoc[lxn][0]]) << ghostLoc[lxn][1] % 8);            
          } else if (c == ghostLoc[lxn][1]/8+1) {
            out |= (pgm_read_byte(&ghost[powerUp][r-ghostLoc[lxn][0]]) >> (8- ghostLoc[lxn][1] % 8) );
          } 
        }     
      }


      int startP, stopP;
      switch (c) {
        case 0:
          startP = 0;
          stopP = 0;
        break;
        case 1:
          startP = 0;
          stopP = 13;
        break;
        case 2:
          startP = 13;
          stopP = 21;
        break;
        case 3:
          startP = 21;
          stopP = 32;
        break;
        case 4:
          startP = 32;
          stopP = 44;
        break;
        case 5:
          startP = 43;
          stopP = 53;
        break;
        case 6:
          startP = 51;
          stopP = 64;
        break;
        case 7:
          startP = 0;
          stopP = 0;
        break;
      }
      
      for (byte lxn = startP; lxn < stopP; lxn++) {        
        if (pillsActive[lxn] == 1) { // this pill is active
          if (pgm_read_byte(&pillLocations[lxn][0]) > pacX && pgm_read_byte(&pillLocations[lxn][0]) < pacX + 8 && pgm_read_byte(&pillLocations[lxn][1]) > pacY && pgm_read_byte(&pillLocations[lxn][1]) < pacY + 8) {pillsActive[lxn] = 0; pillsEaten++;score++; break;}
          if (r == pgm_read_byte(&pillLocations[lxn][0])) { // draw the pills if they appear here
            if (c == pgm_read_byte(&pillLocations[lxn][1])/8) {
              out |= ((B00000001) << pgm_read_byte(&pillLocations[lxn][1]) % 8);            
            } else if (c == pgm_read_byte(&pillLocations[lxn][1])/8 +1) {
              out |= ((B00000001) >> (8- pgm_read_byte(&pillLocations[lxn][1]) % 8) );
            } 
          }     
        }
      }
      for (byte lxn = 0; lxn < 2; lxn++) {
        if (bigPillsActive[lxn] == 1 && c == 6 && (r==pgm_read_byte(&bigPillLocations[lxn]) || r==(pgm_read_byte(&bigPillLocations[lxn])+1) )) {
          out |= B11100000;
        }        
      }
      
      if (pacY == 51) {
        if (pacX < 14 && bigPillsActive[0] == 1) {bigPillsActive[0] = 0; powerUp = 1;}
        if (pacX > 111 && bigPillsActive[1] == 1){bigPillsActive[1] = 0; powerUp = 1;}
      }
      
      ssd1306_send_byte(out);
      incr++;
      } // end for 128     

    ssd1306_send_data_stop();                    
  }
}

int checkCollision(byte xpos, byte ypos) {
  int incr=0;
  int returnValue = 0;
  
  for(byte c = 0; c < 2; c++) { // for each of the two lines that the item might be on
    incr = (128 * (ypos/8+c)) + xpos;
    for (byte r = 0; r < 8; r++) { // for each of the eight bytes in the sprite
      if (c == 0) {
        if ((pgm_read_byte(&gameScreen[incr]) & (0xFF << ypos % 8) ) != 0) returnValue = 1;
      } else {
        if ((pgm_read_byte(&gameScreen[incr]) & (0xFF >> (8- ypos % 8) ) ) != 0) returnValue = 1;
      }  
      incr++;
      }    
  }
  return returnValue;
}


void moveGhosts(void) {
  int travelDirection;  // which way's the ghost going?
  byte travelParam;     // zero for travelling in x, one for y
  int searchDirection;  // +1 to search up, -1 to search down for side exits
  byte paramSelection;  // zero for xPosition, one for yPosition
  byte switchDirection; // which direction to switch to if there's a switch
  
  ghostCounter++;

  // Pop the next ghost out, if it's time
  if (ghostCounter >= releaseDelay) {
    ghostCounter = 0;
    for (byte lxn = 0; lxn < maxGhosts; lxn++) {
      if (ghostsActive[lxn] == 0) {
        ghostLoc[lxn][0] = 83;
        ghostLoc[lxn][1] = 27;    
        ghostsActive[lxn] = 1;
        break;
      }
    }
  }

  for (byte lxn = 0; lxn < 3; lxn++) {
    if (ghostsActive[lxn] == 1) {
      char clash = 0;  
      switch(ghostDir[lxn]) {
          case(DIR_RIGHT):
            travelDirection = 1;
            travelParam = 0;
            searchDirection = -1;
            paramSelection = 1;
            switchDirection = DIR_UP;
          break;
          
          case(DIR_LEFT):
            travelDirection = -1;
            travelParam = 0;
            searchDirection = 1;
            paramSelection = 1;
            switchDirection = DIR_DOWN; 
          break;     
    
          case(DIR_UP):
            travelDirection = -1;
            travelParam = 1;
            searchDirection = -1;
            paramSelection = 0;
            switchDirection = DIR_LEFT; 
          break;
          
          case(DIR_DOWN):
            travelDirection = 1;
            travelParam = 1;
            searchDirection = 1;
            paramSelection = 0;
            switchDirection = DIR_RIGHT; 
          break;      
        }
         
      ghostLoc[lxn][travelParam]+=travelDirection;
      // Move the ghosts along according to which way they are heading - if there's a clash, then backtrack
      if (checkCollision(ghostLoc[lxn][0], ghostLoc[lxn][1]) == 1) {ghostLoc[lxn][travelParam]-=travelDirection; newDirection(lxn);} else {            
        clash = 0;
        for (byte lxn2 = 0; lxn2 < 3; lxn2++) {
          ghostLoc[lxn][paramSelection]+=searchDirection;
          if (checkCollision(ghostLoc[lxn][0], ghostLoc[lxn][1]) == 1) clash = 1; 
        }
        if (clash == 0 && random(100) > BRANCH_PROBABILITY) ghostDir[lxn] = switchDirection;
        ghostLoc[lxn][paramSelection]-=(searchDirection*3);            
      }  
    }  
    for (byte lxn2 = 0; lxn2 < 3; lxn2 ++) {
      if (ghostLoc[lxn2][1] <= 0) ghostLoc[lxn2][1] = 56; else if (ghostLoc[lxn2][1] >= 56) ghostLoc[lxn2][1] = 0;
    }
  }
}

void newDirection(byte ghostNo) {
  ghostDir[ghostNo] = random(4);
}

byte hitGhosts(void){
  byte returnCode = 0;
  
  for (int lxn = 0; lxn < 3 ;lxn++) {
    if ((ghostLoc[lxn][0] > pacX - 8) && (ghostLoc[lxn][0] < pacX + 8) && (ghostLoc[lxn][1] > pacY - 8) && (ghostLoc[lxn][1] < pacY + 8)) {
      if (powerUp == 1) {
        ghostLoc[lxn][0] = 71;
        ghostLoc[lxn][1] = ghostStart[lxn];
        ghostsActive[lxn] =0;
        for (int i = 700; i>200; i = i - 50){
          beep(30,i);
        }
        score += 10;
        ghostCounter = 0;
      } else returnCode = 1; 
    }
  }
  return returnCode;
}

void pacDie(void) {
  int i;
  directions[0] = DIR_UP;
  mouth = 1;
  displayScreen();
  for (i = 0; i<500; i = i+ 50){  
    beep(50,i);
  }
  mouth = 0;
  displayScreen();
  for (i = 500; i<1000; i = i+ 50){  
    beep(50,i);
  }
  delay(1200);
  stopAnimate = 1;
}

void movePacman(void) {
  count++;
  if (count == 2) {
  if (mouth == 0) {mouth = 1; beep(30,400+(powerUp*100));} else {mouth = 0; beep(30,420);}
  count = 0;
  }

  if (powerUp == 1) {
    powerCounter++;
    if (powerCounter >= POWER_LENGTH) {powerUp = 0; powerCounter = 0;}
  }
  
  oldDir = directions[0];
  directions[0] = commandDir;
  
  switch(directions[0]) {
  case(DIR_RIGHT):
    for (int lxn = 0; lxn < 3; lxn++) {
      pacX++;
      if (checkCollision(pacX, pacY) == 1) {
        directions[0] = oldDir;
      } 
    }  
    pacX-=3;
  break;
  
  case(DIR_LEFT):
    for (int lxn = 0; lxn < 3; lxn++) {        
      pacX--;
      if (checkCollision(pacX, pacY) == 1) {
        directions[0] = oldDir;
      }
    } 
    pacX+=3;
  break;
  
  case(DIR_UP):
    for (int lxn = 0; lxn < 3; lxn++) {
      pacY--;
      if (checkCollision(pacX, pacY) == 1) {  
        directions[0] = oldDir;
      }       
    }
    pacY+=3;
  break;
  
  case(DIR_DOWN):
    for (int lxn = 0; lxn < 3; lxn++) {
      pacY++;
      if (checkCollision(pacX, pacY) == 1) {
        directions[0] = oldDir;
      }
    }
    pacY-=3;              
  break;      
  }
  
  switch(directions[0]) {
    case(DIR_RIGHT):
      pacX+=1;
      if (checkCollision(pacX, pacY) == 1) pacX-=1;
    break;
    
    case(DIR_LEFT):
      pacX-=1;
      if (checkCollision(pacX, pacY) == 1) pacX+=1;
    break;
    
    case(DIR_UP):
      pacY-=1;
      if (checkCollision(pacX, pacY) == 1) pacY+=1;
    break;
    
    case(DIR_DOWN):
      pacY+=1;
      if (checkCollision(pacX, pacY) == 1) pacY-=1;          
    break;      
  }
  if (pacY <= 0) pacY = 56; else if (pacY >= 56) pacY = 0;
}

void initLevel(void) {
  clickLock = 0;
  pillsEaten = 0;
  stopAnimate = 0;

  for(byte lxn = 0; lxn < 64; lxn++) pillsActive[lxn] = 1;  
  bigPillsActive[0] = 1;
  bigPillsActive[1] = 1;
}

void initScreen(void) {
  powerUp = 0;
  powerCounter = 0;
  ghostsActive[0] = 0;
  ghostsActive[1] = 0;
  ghostsActive[2] = 0;
  ghostCounter = 0;
  pacX = 11;
  pacY = 5;

  for (byte lxn = 0; lxn<3; lxn++) {
    ghostLoc[lxn][0] = 71;
    ghostLoc[lxn][1] = ghostStart[lxn];
  }
  
  ghostDir[0] = DIR_UP;
  ghostDir[1] = DIR_DOWN;
  ghostDir[2] = DIR_UP;
  
  for(byte inc = 0; inc < 4; inc++) directions[inc] = DIR_RIGHT;
  commandDir = DIR_RIGHT;
  
}

// Handle what happens at the end of a level
void levelUp() {  
  initScreen();
  initLevel();
  level++;
  if (maxGhosts < 3) maxGhosts++;
  if (releaseDelay > GHOST_DELAY_MINIMUM) releaseDelay -= GHOST_DELAY_REDUCTION;
  ssd1306_fillscreen(0x00);
  ssd1306_char_f6x8(35, 1, "---------");
  ssd1306_char_f6x8(35, 3, " LEVEL ");
  ssd1306_char_f6x8(35, 5, "---------");
  doNumber(77,3,level);
  delay(1500);    
  ssd1306_fillscreen(0x00);
  displayScreen();
}

/* ------------------------
 *  Pacman main game code
 */
void playPacman(){
  score = 0;
  lives = 2;
  level = 1;
  releaseDelay = INITIAL_GHOST_DELAY;
  maxGhosts = INITIAL_GHOSTS;

  attachInterrupt(0,playerIncPacman,CHANGE);

  initLevel();
  initScreen();

  while(lives >= 0) {
    
    if (pillsEaten >= 64) levelUp();

    moveGhosts();
    
    if (hitGhosts() != 0) {
      pacDie();
      lives--;
      initScreen();      
    }

    movePacman();

    if (clickLock == 1 && millis() > clickBase + CLICKDELAY && digitalRead(2)==0 && digitalRead(0)==0) clickLock = 0; // normal debounce
         
    displayScreen();
  }   
}


