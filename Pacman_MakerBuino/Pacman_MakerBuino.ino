/*  2018
 *   
 *  Pacman for MakerBuino and GameBuino by Andy Jackson - Twitter @andyhighnumber
 *  
 *  You need to install the 'Classic' GameBuino libraries in your Arduino IDE, like this:
 *  http://legacy.gamebuino.com/wiki/index.php?title=Getting_started#Install_the_Gamebuino_Library_.28Automatic.29
 *  
 *  Everything you need to build and run this game is contained in this file and the font 
 *  header (font6x8AJ3.h) 
 *  
 *  This is a port of a Pac-Man clone written for the AttinyArcade, which is why some of
 *  the display routines look a bit weird. More info here:
 *  https://github.com/andyhighnumber/Attiny-Arduino-Games
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of 
 *  this software and associated documentation files (the "Software"), to deal in the 
 *  Software without restriction, including without limitation the rights to use, copy, 
 *  modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, 
 *  and to permit persons to whom the Software is furnished to do so, subject to the 
 *  following conditions: 
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 *  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 *  PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 *  FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR 
 *  OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 *  DEALINGS IN THE SOFTWARE.
 * 
*/

//imports the SPI library (needed to communicate with Gamebuino's screen)
#include <SPI.h>
//imports the Gamebuino library
#include <Gamebuino.h>
//creates a Gamebuino object named gb
Gamebuino gb;


#include <EEPROM.h>
#include "font6x8AJ4.h"

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

/* ------ This section needs to be updated if you change the screen bitmap ------
   ------------------------------------------------------------------------------*/

// Where on the screen to the ghosts appear when they pop out onto the playing screen?
#define GHOST_LAUNCH_X 64
#define GHOST_LAUNCH_Y 16

// How many pills are there on the screen?
#define NO_PILLS 56

// Where are the ghosts and pacman drawn initially?
// In each of these, the order is - Ghost1, Ghost2, Ghost3, PAC-MAN
static const byte startPosX[] = {66, 55, 77, 55};
static const byte startPosY[] = {28, 28, 28, 40};
static const byte startDirections[] = {DIR_UP, DIR_UP, DIR_UP, DIR_RIGHT};

// Where are the big pills? (their row is fixed as the last one)
static const byte bigPillLocations[2] PROGMEM = {14, 119};

// How many pills on each row? for the six rows 1-7 that you are allowed pills
static const byte readRow[6][2] PROGMEM = {
  0, 12,
  12, 23,
  23, 31,
  31, 34,
  34, 45,
  45, 56
};

// Where are the pills (x,y)
static const byte pillLocations[NO_PILLS][2] PROGMEM = {
  14, 8,
  24, 8,
  36, 8,
  46, 8,
  55, 8,
  63, 8,
  75, 8,
  84, 8,
  93, 8,
  104, 8,
  111, 8,
  119, 8,
  14, 20,
  24, 20,
  36, 20,
  46, 20,
  55, 20,
  63, 20,
  75, 20,
  84, 20,
  93, 20,
  104, 17,
  119, 17,
  14, 31,
  24, 31,
  36, 31,
  46, 31,
  93, 31,
  104, 25,
  111, 25,
  119, 25,
  104, 37,
  111, 37,
  119, 37,
  14, 43,
  24, 43,
  36, 43,
  46, 43,
  55, 43,
  63, 43,
  75, 43,
  84, 43,
  93, 43,
  104, 46,
  119, 46,
  14, 55,
  24, 55,
  36, 55,
  46, 55,
  55, 55,
  63, 55,
  75, 55,
  84, 55,
  93, 55,
  104, 55,
  111, 55
};

// Display functions - a legacy from the AttinyArcade version - could be replaced with GameBuino function calls if you wish
void lcdDisplay_send_byte(uint8_t byte);
void lcdDisplay_setpos(uint8_t x, uint8_t y);
void lcdDisplay_fillscreen(uint8_t fill_Data);
void lcdDisplay_char_f6x8(uint8_t x, uint8_t y, const char ch[]);

// Other generic functions for games (both originated in code from webboggles.com)
void doNumber (int, int, int);

// Game functions
void playPacman(void);
void levelUp(int);
void showScore(void);
void displayTitle(void);
int checkCollision(byte xpos, byte ypos);
void initScreen(void);
void initLevel(void);
void newDirection(byte ghostNo);
byte hitGhosts(void);
void movePacman(void);
void pacDie(void);
void eatenGhost(void);
void moveGhosts(void);
byte drawPacman(byte c, byte r);


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
int mouth = 0;            // Is his mouth open?
int directions[4];        // Directions of the four possible characters (pacman = 0, ghosts are 1-3)
int commandDir;           // The desired direction set by input
byte pillsEaten = 0;      // How many pills have been eaten (there are 63 altogether)
int ghostCounter = 0;     // How many ghosts
int ghostsActive[3];      // How many ghosts are out?
int count = 0;            // Counter
int oldDir;               // Holds previous direction
int level = 0;            // Obvious I hope
int releaseDelay;         // The delay between ghosts being let out
int maxGhosts;            // How many ghosts maximum on this level?
bool powerUp;             // Is powerUp move active?
int powerCounter;         // Counts the length of powerUp mode
byte locations[4][2];     // The x,y location of the sprites
byte pillsActive[NO_PILLS];    // Which of the pills is still on the screen?
byte bigPillsActive[2];   // Which of the two big pills is still active?

byte currentX, currentY, currentRow, currentCol;

int screenTop, screenLeft;

// Artwork
static const byte pacOpen[4][8] PROGMEM = {
  0x3C, 0x7E, 0xFF, 0xFF, 0xE7, 0xE7, 0xC3, 0x42,
  0x7C, 0xFE, 0x3F, 0x0F, 0x0F, 0x3F, 0xFE, 0x7C,
  0x42, 0xC3, 0xE7, 0xE7, 0xFF, 0xFF, 0x7E, 0x3C,
  0x3E, 0x7F, 0xFC, 0xF0, 0xF0, 0xFC, 0x7F, 0x3E
};

static const byte pacClosed[] PROGMEM = {0x3C, 0x7E, 0xFF, 0xFF, 0xFF, 0xFF, 0x7E, 0x3C};

static const byte ghost[2][8] PROGMEM = {0x7C, 0xFE, 0x22, 0xEB, 0xFF, 0x62, 0xEA, 0x7C, 0x7C, 0xC2, 0x0A, 0x83, 0x83, 0x0A, 0x82, 0x7C};

static const byte PAClives[] PROGMEM = {0x1C, 0x3C, 0x7E, 0x66, 0x66, 0x62, 0x40};

static const byte openBmp[] PROGMEM = {
  0x00, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFC, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE,
  0xFE, 0xFC, 0xFC, 0xF8, 0x70, 0x60, 0x00,
  0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xEF, 0xEF, 0xC7, 0x87, 0x83,
  0x83, 0x01, 0x00, 0x00, 0x10, 0x38, 0x7C, 0x38, 0x10, 0x00, 0x00, 0x00, 0x00, 0x10, 0x38, 0x7C,
  0x38, 0x10,
  0x00, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0x7F, 0x7F, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F,
  0x7F, 0x3F, 0x3F, 0x3E, 0x1E, 0x1E
};

static const byte gameScreen[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x04, 0xF4, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14,
  0x14, 0x14, 0x13, 0x10, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x10, 0x13,
  0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0xE4, 0x04, 0xE4, 0x14, 0x14, 0x14, 0x14, 0x14,
  0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14,
  0x14, 0x14, 0x14, 0x14, 0xE4, 0x04, 0xE4, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14,
  0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14,
  0x14, 0xE4, 0x04, 0xE4, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14,
  0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0xE4, 0x04, 0xF8, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x40, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0x40,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x40, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0x40, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x7F, 0x80, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
  0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x7F, 0x80, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x20, 0xA0, 0xA0,
  0xA0, 0xA0, 0x20, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x20, 0x2F, 0x2F,
  0x2F, 0x2F, 0x20, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xFE, 0x01, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x01, 0xFE,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x08, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xFE, 0x01, 0xFD, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
  0x05, 0x02, 0x00, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x00, 0x02, 0x05, 0x05, 0x05, 0x05, 0x05,
  0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0xFD, 0x01, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xFE, 0x01, 0x3E, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40,
  0x40, 0x40, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x7F, 0x80, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x80, 0x7F,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x20, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x7F, 0x80, 0x9F, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0,
  0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0,
  0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0x9F, 0x80, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x7F, 0x80, 0x7E, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x04, 0xF4, 0xF4,
  0xF4, 0xF4, 0x04, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x02, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x02,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x02, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x02, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0xFE, 0x01, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xFE, 0x01, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x04, 0x05, 0x05,
  0x05, 0x05, 0x04, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00,

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x20, 0x27, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28,
  0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28,
  0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x27, 0x20, 0x27, 0x28, 0x28, 0x28, 0x28, 0x28,
  0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28,
  0x28, 0x28, 0x28, 0x28, 0x27, 0x20, 0x27, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28,
  0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28,
  0x28, 0xC7, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x08, 0xC8, 0x28,
  0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x27, 0x20, 0x1F, 0x00
};

void displayTitle(void) {
  int incr = 0;
  for (int lxn = 2; lxn < 5; lxn++) {
    lcdDisplay_setpos(84, lxn);
    for (int lxn2 = 0; lxn2 < 34; lxn2++) {
      lcdDisplay_send_byte(pgm_read_byte(&openBmp[incr]));
      incr++;
      if ( (lxn == 2 || lxn == 4) && lxn2 > 21) lxn2 = 35;
    }
  }
}

// Arduino stuff - setup
void setup() {
  // initialize the Gamebuino object
  gb.begin();
  //display the main menu:
  gb.titleScreen(F("PAC-MAN"));
}

// Arduino stuff - loop
void loop() {
  lcdDisplay_fillscreen(1);
  gb.display.update();

  while(gb.buttons.pressed(BTN_A) == true) { gb.update(); delay(5);}
  
  while(gb.buttons.pressed(BTN_A) == false) {
    lcdDisplay_char_f6x8(0, 1, "Press A ....");
    gb.display.update();
    gb.update();
  }

  screenLeft = 0;
  for (int incr = 0; incr < 44 ; incr+=3) {
    screenLeft = incr;
    
    lcdDisplay_char_f6x8(0, 1, "P A C-M A N");
    lcdDisplay_char_f6x8(0, 3, "andy jackson");

    lcdDisplay_setpos(0, 0);
    for (int incr2 = 0; incr2 < 76; incr2++) {
      lcdDisplay_send_byte(B00111000);
    }
    
    lcdDisplay_setpos(0, 2);
    for (int incr2 = 0; incr2 < 76; incr2++) {
      lcdDisplay_send_byte(B00011100);
    }

    displayTitle();
    gb.display.update();

    if (incr == 0) delay(1200);
    lcdDisplay_fillscreen(1);
  }

  int sChange = 0;

  if (sChange == 0) {
    delay(1000);

    lcdDisplay_fillscreen(1);

    playPacman();

    topScore = EEPROM.read(0);
    topScore = topScore << 8;
    topScore = topScore |  EEPROM.read(1);

    newHigh = 0;
    if (score > topScore) {
      topScore = score;
      EEPROM.write(1, score & 0xFF);
      EEPROM.write(0, (score >> 8) & 0xFF);
      newHigh = 1;
    }

    screenLeft = 0;

    lcdDisplay_fillscreen(0x00);
    lcdDisplay_char_f6x8(0, 0, "------------");
    lcdDisplay_char_f6x8(0, 1, " GAME  OVER");
    lcdDisplay_char_f6x8(0, 3, "------------");
    showScore();
    if (!newHigh) {
      lcdDisplay_char_f6x8(21, 7, "HIGH SCORE:");
      doNumber(88, 7, topScore);
    }
    gb.display.update();
    delay(1500);
    if (newHigh) {
      lcdDisplay_fillscreen(0x00);
      lcdDisplay_char_f6x8(0, 0, "------------");
      lcdDisplay_char_f6x8(0, 1, " NEW HIGH ");
      lcdDisplay_char_f6x8(0, 3, "------------");
      doNumber(50, 5, topScore);
      gb.display.update();
      delay(1500);
    }
  }
}

void showScore(void) {
  lcdDisplay_char_f6x8(0, 2, " SCORE:");
  doNumber(37, 2, score);
}

// Handle what happens at the end of a level
void levelUp() {
  initScreen();
  initLevel();
  level++;
  if ((level == 3) || (level == 5)) lives++;
  if (maxGhosts < 3) maxGhosts++;
  if (releaseDelay > GHOST_DELAY_MINIMUM) releaseDelay -= GHOST_DELAY_REDUCTION;
  lcdDisplay_fillscreen(0x00);

  screenLeft = 0;

  lcdDisplay_char_f6x8(0, 3, "LEVEL:");
  doNumber(37, 3, level);
  showScore();
  gb.display.update();

  delay(2500);
  lcdDisplay_fillscreen(0x00);
  displayScreen();
}


void doNumber (int x, int y, int value) {
  char temp[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  itoa(value, temp, 10);
  lcdDisplay_char_f6x8(x, y, temp);
}


void lcdDisplay_send_byte(uint8_t input) {
  for (int by = 0; by < 8; by++) {
    if ((input >> by & B00000001) == 0) {
      if ( (currentX >= screenLeft) && (currentX <= screenLeft + 84)) {
        gb.display.setColor(WHITE, BLACK);
        gb.display.drawPixel(currentX - screenLeft, (currentY) * 8 + by);
      }
    } else {
      if ( (currentX >= screenLeft) && (currentX <= screenLeft + 84)) {
        gb.display.setColor(BLACK, WHITE);
        gb.display.drawPixel(currentX - screenLeft, (currentY) * 8 + by);
      }
    }
  }
  currentX++;
}

void lcdDisplay_setpos(uint8_t x, uint8_t y)
{
  currentX = x;
  currentY = y;
}

void lcdDisplay_fillscreen(uint8_t fill_Data) {
  gb.display.setColor(WHITE, BLACK);

  for (int i = 0; i < 64; i++) {
    for (int j = 0; j < 128; j++) {
      gb.display.drawPixel(j, i);
    }
  }

}

void lcdDisplay_char_f6x8(uint8_t x, uint8_t y, const char ch[]) {
  uint8_t c, i, j = 0;
  while (ch[j] != '\0')
  {
    c = ch[j] - 32;
    if (c > 0) c = c - 12;
    if (c > 15) c = c - 6;
    if (c > 40) c = c - 9;
    if (x > 126)
    {
      x = 0;
      y++;
    }
    lcdDisplay_setpos(x, y);
    for (i = 0; i < 6; i++)
    {
      lcdDisplay_send_byte(pgm_read_byte(&lcdDisplayxled_font6x8[c * 6 + i]));
    }
    x += 6;
    j++;
  }
}

byte drawPacman(byte c, byte r) {
  byte out = 0;
  if ( (r >= locations[3][0]) && (r < locations[3][0] + 8) ) { // draw the pacMan if any of his body is on this column
    if (c == locations[3][1] / 8) {
      if (mouth == 1) {
        out = (pgm_read_byte(&pacOpen[directions[3]][r - locations[3][0]]) << locations[3][1] % 8);
      } else {
        out = (pgm_read_byte(&pacClosed[r - locations[3][0]]) << locations[3][1] % 8);
      }
    } else if (c == locations[3][1] / 8 + 1) {
      if (mouth == 1) {
        out = (pgm_read_byte(&pacOpen[directions[3]][r - locations[3][0]]) >> (8 - locations[3][1] % 8) );
      } else {
        out = (pgm_read_byte(&pacClosed[r - locations[3][0]]) >> (8 - locations[3][1] % 8) );
      }
    }
  }
  return out;
}

byte drawGhostSprite(byte c, byte r, byte lxn) {
  byte out = 0;

  if ( (r >= locations[lxn][0]) && (r < locations[lxn][0] + 8) ) {
      if (c == locations[lxn][1] / 8) {
        out |= (pgm_read_byte(&ghost[powerUp][r - locations[lxn][0]]) << locations[lxn][1] % 8);
      } else if (c == locations[lxn][1] / 8 + 1) {
        out |= (pgm_read_byte(&ghost[powerUp][r - locations[lxn][0]]) >> (8 - locations[lxn][1] % 8) );
      }
    }

  return out;
}

byte drawPills(byte c, byte r) {
  byte out = 0;
      // draw the pills and also check whether pacman is eating an active pill
      if (c > 0 && c < 7) {
        for (byte lxn = pgm_read_byte(&readRow[c - 1][0]); lxn < pgm_read_byte(&readRow[c - 1][1]); lxn++) {
          if (pillsActive[lxn]) { // this pill is active
            if (r == pgm_read_byte(&pillLocations[lxn][0])) { // draw the pills if they appear here
              // check if pac-man has eaten a pill
              if (pgm_read_byte(&pillLocations[lxn][0]) > locations[3][0] && pgm_read_byte(&pillLocations[lxn][0]) < locations[3][0] + 8 && pgm_read_byte(&pillLocations[lxn][1]) > locations[3][1] && pgm_read_byte(&pillLocations[lxn][1]) < locations[3][1] + 8) {
                pillsActive[lxn] = 0;
                pillsEaten++;
                score += level;
                break;
              }
              if (c == pgm_read_byte(&pillLocations[lxn][1]) / 8) {
                out |= ((B00000001) << pgm_read_byte(&pillLocations[lxn][1]) % 8);
              } else if (c == pgm_read_byte(&pillLocations[lxn][1]) / 8 + 1) {
                out |= ((B00000001) >> (8 - pgm_read_byte(&pillLocations[lxn][1]) % 8) );
              }
            }
          }
        }
      }
  return out;  
}

byte drawBigPills(byte c, byte r) {
  byte out = 0;
  
  for (byte lxn = 0; lxn < 2; lxn++) {
    if (bigPillsActive[lxn] == 1 && c == 6 && (r == pgm_read_byte(&bigPillLocations[lxn]) || r == (pgm_read_byte(&bigPillLocations[lxn]) + 1) )) {
      out |= B11100000;
    }
  }
  return out;
}

byte createScreenOut(byte c, byte r) {
  byte out;
      // 'out' is the byte we're going to write, starts with the background image
      out = (pgm_read_byte(&gameScreen[c*128+r]));

      // add pacman, if part of his sprite is on this space
      out |= drawPacman(c, r);

      // Draw the lives in the far left column
      if (r < 7) {
        for (byte lxn = 3; lxn < 7; lxn++) {
          if (c == lxn && lives > 6 - lxn) out |= (pgm_read_byte(&PAClives[r]));
        }
      }

      // draw a ghost if any of their sprites are on this column and row
      for (byte lxn = 0; lxn < 3; lxn++) {
        out |= drawGhostSprite(c, r, lxn);
      }

      out |= drawPills(c, r);

      // Draw the big pills
      out |= drawBigPills(c, r);

  return out;
}

void displayScreen(void) {
  int pacDir = directions[3];
  byte out;
  uint8_t *disp;
  char nextRow[84];

  // For each row on the screen (there's 8 - I think of them as columns, hence 'c', I don't know why!)

  disp = gb.display.getBuffer();


  byte c = screenTop /8 + 6;
  for (byte r= screenLeft; r < screenLeft+84; r++) {
    nextRow[r-screenLeft] = createScreenOut(c, r);
  }

  for (byte c = screenTop /8 + 5; c > screenTop /8; c--) {
    for (byte r = screenLeft; r < screenLeft+84; r++) {

    byte thisRow = createScreenOut(c, r);
    
    out = ( (thisRow >> screenTop % 8) | (nextRow[r-screenLeft] << 8- screenTop % 8));

    nextRow[r-screenLeft] = thisRow;
    
    *(disp + r - screenLeft + ((c - screenTop/8) * 84)) = out;

    }
  }

  c = screenTop /8;
  for (byte r= screenLeft; r < screenLeft+84; r++) {
    byte thisRow = createScreenOut(c, r);
    out = ( (thisRow >> screenTop % 8) | (nextRow[r-screenLeft] << 8- screenTop % 8));
    *(disp + r - screenLeft + ((c - screenTop/8) * 84)) = out;
  }
  
  gb.display.update();
}

int checkCollision(byte xpos, byte ypos) {
  int incr = 0;
  int returnValue = 0;

  for (byte c = 0; c < 2; c++) { // for each of the two lines that the item might be on
    incr = (128 * (ypos / 8 + c)) + xpos;
    for (byte r = 0; r < 8; r++) { // for each of the eight bytes in the sprite
      if (c == 0) {
        if ((pgm_read_byte(&gameScreen[incr]) & (0xFF << ypos % 8) ) != 0) returnValue = 1;
      } else {
        if ((pgm_read_byte(&gameScreen[incr]) & (0xFF >> (8 - ypos % 8) ) ) != 0) returnValue = 1;
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
        locations[lxn][0] = GHOST_LAUNCH_X;
        locations[lxn][1] = GHOST_LAUNCH_Y;
        ghostsActive[lxn] = 1;
        break;
      }
    }
  }

  for (byte lxn = 0; lxn < 3; lxn++) {
    if (ghostsActive[lxn] == 1) {
      char clash = 0;

      // Start with a default set (which is DIR_RIGHT) then only change the things that differ from this in the switch statements below
      // (saves two bytes of code, but every bit helps!)
      travelDirection = 1;
      travelParam = 0;
      searchDirection = -1;
      paramSelection = 1;
      switchDirection = DIR_UP;

      switch (directions[lxn]) {
        case (DIR_LEFT):
          travelDirection = -1;
          searchDirection = 1;
          switchDirection = DIR_DOWN;
          break;

        case (DIR_UP):
          travelDirection = -1;
          travelParam = 1;
          paramSelection = 0;
          switchDirection = DIR_LEFT;
          break;

        case (DIR_DOWN):
          travelParam = 1;
          searchDirection = 1;
          paramSelection = 0;
          switchDirection = DIR_RIGHT;
          break;
      }

      locations[lxn][travelParam] += travelDirection;
      // Move the ghosts along according to which way they are heading - if there's a clash, then backtrack
      if (checkCollision(locations[lxn][0], locations[lxn][1]) == 1) {
        locations[lxn][travelParam] -= travelDirection;
        newDirection(lxn);
      } else {
        clash = 0;
        for (byte lxn2 = 0; lxn2 < 3; lxn2++) {
          locations[lxn][paramSelection] += searchDirection;
          if (checkCollision(locations[lxn][0], locations[lxn][1]) == 1) clash = 1;
        }
        if (clash == 0 && random(100) > BRANCH_PROBABILITY) directions[lxn] = switchDirection;
        locations[lxn][paramSelection] -= (searchDirection * 3);
      }
    }
  }
}

void newDirection(byte ghostNo) {
  directions[ghostNo] = random(4);
}

void eatenGhost(void) {
  /*
  for (int i = 20; i < 40; i = i + 4) {
    gb.sound.playNote(i, 1, 0);
  }
  */
}

byte hitGhosts(void) {
  byte returnCode = 0;

  for (int lxn = 0; lxn < 3 ; lxn++) {
    if ((locations[lxn][0] > locations[3][0] - 8) && (locations[lxn][0] < locations[3][0] + 8) && (locations[lxn][1] > locations[3][1] - 8) && (locations[lxn][1] < locations[3][1] + 8)) {
      if (powerUp == 1) {
        locations[lxn][0] = startPosX[lxn];
        locations[lxn][1] = startPosY[lxn];
        ghostsActive[lxn] = 0;
        eatenGhost();
        score += 10;
        ghostCounter = 0;
      } else returnCode = 1;
    }
  }
  return returnCode;
}

void pacDie(void) {
  int i;
  directions[3] = DIR_UP;
  mouth = 1;
  displayScreen();
  for (i = 50; i > 30; i = i - 5) {
    gb.sound.playNote(i, 1, 0);
    gb.update();
    delay(20);
  }
  displayScreen();
  gb.display.update();
  delay(300);
  mouth = 0;
  displayScreen();
  for (i = 30; i > 10; i = i - 5) {
    gb.sound.playNote(i, 1, 0);
    gb.update();
    delay(20);
  }
  displayScreen();
  delay(500);
  stopAnimate = 1;
}

void movePacman(void) {
  count++;
  if (count == 2) {
    if (mouth == 0) {
      mouth = 1;
      gb.sound.playNote(10+(powerUp*2),1,0);
      gb.update();
    } else {
      mouth = 0;
      gb.sound.playNote(9,1,0);
      gb.update();
    }
    count = 0;
  }

  if (powerUp == 1) {
    powerCounter++;
    if (powerCounter >= POWER_LENGTH) {
      powerUp = 0;
      powerCounter = 0;
    }
  }

  oldDir = directions[3];
  directions[3] = commandDir;

  screenLeft = locations[3][0] - 30;
  screenTop =  locations[3][1] - 20;

  if (screenLeft < 0) screenLeft = 0;
  if (screenTop < 0) screenTop = 0;
  if (screenLeft > 43) screenLeft = 43;
  if (screenTop > 16) screenTop = 16;

  switch (directions[3]) {
    case (DIR_RIGHT):
      for (int lxn = 0; lxn < 3; lxn++) {
        locations[3][0]++;
        if (checkCollision(locations[3][0], locations[3][1]) == 1) {
          directions[3] = oldDir;
        }
      }
      locations[3][0] -= 3;
      break;

    case (DIR_LEFT):
      for (int lxn = 0; lxn < 3; lxn++) {
        locations[3][0]--;
        if (checkCollision(locations[3][0], locations[3][1]) == 1) {
          directions[3] = oldDir;
        }
      }
      locations[3][0] += 3;
      break;

    case (DIR_UP):
      for (int lxn = 0; lxn < 3; lxn++) {
        locations[3][1]--;
        if (checkCollision(locations[3][0], locations[3][1]) == 1) {
          directions[3] = oldDir;
        }
      }
      locations[3][1] += 3;
      break;

    case (DIR_DOWN):
      for (int lxn = 0; lxn < 3; lxn++) {
        locations[3][1]++;
        if (checkCollision(locations[3][0], locations[3][1]) == 1) {
          directions[3] = oldDir;
        }
      }
      locations[3][1] -= 3;
      break;
  }

  switch (directions[3]) {
    case (DIR_RIGHT):
      locations[3][0] += 1;
      if (checkCollision(locations[3][0], locations[3][1]) == 1) locations[3][0] -= 1;
      break;

    case (DIR_LEFT):
      locations[3][0] -= 1;
      if (checkCollision(locations[3][0], locations[3][1]) == 1) locations[3][0] += 1;
      break;

    case (DIR_UP):
      locations[3][1] -= 1;
      if (checkCollision(locations[3][0], locations[3][1]) == 1) locations[3][1] += 1;
      break;

    case (DIR_DOWN):
      locations[3][1] += 1;
      if (checkCollision(locations[3][0], locations[3][1]) == 1) locations[3][1] -= 1;
      break;
  }

  // If pacman is on the bottom row, check if he's eaten either of the big pills
  if (locations[3][1] == 51) {
    if ((locations[3][0] < 14 && bigPillsActive[0] == 1) || (locations[3][0] > 111 && bigPillsActive[1] == 1)) {
      if (locations[3][0] < 14) bigPillsActive[0] = 0; else bigPillsActive[1] = 0;
      powerUp = 1;
      powerCounter = 0;
    }
  }
}

void initLevel(void) {
  //clickLock = 0;
  pillsEaten = 0;
  stopAnimate = 0;

  for (byte lxn = 0; lxn < NO_PILLS; lxn++) pillsActive[lxn] = 1;
  bigPillsActive[0] = 1;
  bigPillsActive[1] = 1;
}

void initScreen(void) {

  gb.sound.playNote(63,1,0);
  gb.update();
  
  powerUp = 0;
  powerCounter = 0;
  ghostsActive[0] = 0;
  ghostsActive[1] = 0;
  ghostsActive[2] = 0;
  ghostCounter = 0;

  // Set up the initial directions of the ghosts and pacman
  for (byte lxn = 0; lxn < 4; lxn++) {
    locations[lxn][0] = startPosX[lxn];
    locations[lxn][1] = startPosY[lxn];
    directions[lxn] = startDirections[lxn];
  }

  // Initially set pacman off in the direction he is facing
  commandDir = startDirections[3];
}

/* ------------------------
    Pacman main game code
*/
void playPacman() {
  score = 0;
  lives = 2;
  level = 1;
  releaseDelay = INITIAL_GHOST_DELAY;
  maxGhosts = INITIAL_GHOSTS;

  initLevel();
  initScreen();

  while (lives >= 0) {
    gb.update();

    if (gb.buttons.pressed(BTN_LEFT) == true) commandDir = DIR_LEFT;
    if (gb.buttons.pressed(BTN_RIGHT) == true) commandDir = DIR_RIGHT;
    if (gb.buttons.pressed(BTN_UP) == true) commandDir = DIR_UP;
    if (gb.buttons.pressed(BTN_DOWN) == true) commandDir = DIR_DOWN;

    if (pillsEaten >= NO_PILLS) levelUp();

    moveGhosts();
    movePacman();

    // Check for ghosrs or pacman going thru the tunnel teleport thing
    for (byte lxn2 = 0; lxn2 < 4; lxn2 ++) {
      if (locations[lxn2][1] <= 0) {
        locations[lxn2][1] = 56;
        locations[lxn2][0] = 100;
      } else if (locations[lxn2][1] >= 56) {
        locations[lxn2][1] = 0;
        locations[lxn2][0] = 21;
      }
    }

    if (hitGhosts() != 0) {
      pacDie();
      lives--;
      initScreen();
    }
    
    displayScreen();

  }
  initScreen();
}
