/*  2018
 *   
 *  Frogger for MakerBuino and GameBuino by Andy Jackson - Twitter @andyhighnumber
 *  
 *  You need to install the 'Classic' GameBuino libraries in your Arduino IDE, like this:
 *  http://legacy.gamebuino.com/wiki/index.php?title=Getting_started#Install_the_Gamebuino_Library_.28Automatic.29
 *  
 *  Everything you need to build and run this game is contained in this file and the font 
 *  header (font6x8AJ3.h) 
 *  
 *  This is a port of a Frogger clone written for the AttinyArcade, which is why some of
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


// Uncomment this #define to make the logs smaller (/thinner)
#define SMALLLOGS

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
#define lcdDisplayXLED_H
#define lcdDisplay_SCL   PORTB4  // SCL, Pin 4 on lcdDisplay Board - for webbogles board
#define lcdDisplay_SDA   PORTB3  // SDA, Pin 3 on lcdDisplay Board - for webbogles board
#define lcdDisplay_SA    0x78  // Slave address

// Function prototypes

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
bool moveBack=0;         // Captures when the 'right' button is pressed

int screenLeft = 0;       // Current left position of the displayed screen
int screenTop = 0;        // Current top of the displayed screen
byte currentX = 0;        // Current X positon to draw
byte currentY = 0;        // Current Y positon to draw


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


// Display functions - a legacy from the AttinyArcade version - could be replaced with GameBuino function calls if you wish
void lcdDisplay_send_byte(uint8_t byte);
void lcdDisplay_setpos(uint8_t x, uint8_t y);
void lcdDisplay_fillscreen(uint8_t fill_Data);
void lcdDisplay_char_f6x8(uint8_t x, uint8_t y, const char ch[]);

// Other generic functions for games (both originated in code from webboggles.com)
void doNumber (int, int, int);
void beep(int,int);

void displayTitle(void) {
  int incr = 0;
  for(int lxn = 0; lxn < 5; lxn++) {
    lcdDisplay_setpos(84,lxn); 
    for(int lxn2 = 0; lxn2 < 40; lxn2++) {
      lcdDisplay_send_byte(pgm_read_byte(&titleBmp[incr]));      
      incr++;
    }
  }
}

void beep(int bCount,int bDelay){
  if (mute) return;
  for (int i = 0; i<=bCount; i++){
    digitalWrite(3,HIGH);
    for(int i2=0; i2<bDelay; i2++){
      __asm__("nop\n\t"); // 62.5ns delay @ 16MHz
      }
    digitalWrite(3,LOW);
    for(int i2=0; i2<bDelay; i2++) {
      __asm__("nop\n\t"); // 62.5ns delay @ 16Mhz
    }
  }
}

// Arduino stuff - setup
void setup() {
  // initialize the Gamebuino object
  gb.begin();
  //display the main menu:
  gb.titleScreen(F("FROGGER"));
}

void displayOpenScreen(int incr) {
    lcdDisplay_fillscreen(1);
    if (incr < 99) screenLeft = incr;
    
    lcdDisplay_char_f6x8(0, 1, "F R O G G E R");
    lcdDisplay_char_f6x8(0, 3, "andy jackson");
    lcdDisplay_char_f6x8(64, 5, "Press A...");

    lcdDisplay_setpos(0, 0);
    for (int incr2 = 0; incr2 < 76; incr2++) {
      lcdDisplay_send_byte(B00111000);
    }
    
    lcdDisplay_setpos(0, 2);
    for (int incr2 = 0; incr2 < 76; incr2++) {
      lcdDisplay_send_byte(B00011100);
    }

    displayTitle();
}

// Arduino stuff - loop
void loop() {
  lcdDisplay_fillscreen(1);
  gb.display.update();

  while(gb.buttons.pressed(BTN_A) == true) { while(!gb.update()); delay(5);}

  gb.sound.playNote(63,1,0);
  while(!gb.update());
  
  screenLeft = 0;
  for (int incr = 0; incr < 46 ; incr+=3) {
    displayOpenScreen(incr);
    gb.display.update();
    if (incr == 0) delay(1700);
  }

  while(gb.buttons.pressed(BTN_A) == false && gb.buttons.pressed(BTN_B) == false ) {
    displayOpenScreen(100);    
    while(!gb.update());
  }


  lcdDisplay_fillscreen(1);
  int sChange = 0;

  screenLeft = 0;
  screenTop = 0;
  
  if (gb.buttons.pressed(BTN_B) == true) {
      sChange = 1;     
      EEPROM.write(0,0);
      EEPROM.write(1,0);
      lcdDisplay_char_f6x8(0, 0, "--------------");
      lcdDisplay_char_f6x8(0, 1, "HI SCORE RESET");
      lcdDisplay_char_f6x8(0, 3, "--------------");
      while(!gb.update());
      delay(2000);
  }
  
  if (sChange == 0) {

    lcdDisplay_fillscreen(1);

    playFrogger();

    screenLeft = 0;
    screenTop = 0;

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
    lcdDisplay_char_f6x8(0, 1, "GAME  OVER");
    lcdDisplay_char_f6x8(0, 3, "------------");
    showScore();
    while(!gb.update());
    delay(2500);
    lcdDisplay_fillscreen(0x00);
    lcdDisplay_char_f6x8(0, 0, "---------------");
    lcdDisplay_char_f6x8(0, 3, "---------------");
    doNumber(24, 2, topScore);
    if (!newHigh) {
      lcdDisplay_char_f6x8(0, 1, "HIGH SCORE:");
    } else {
      lcdDisplay_char_f6x8(0, 1, "NEW HIGH:");
      for (int i = 700; i>200; i = i - 50){
        beep(30,i);
      }
    }
    while(!gb.update());
    delay(2500);
  }
  
}

void doNumber (int x, int y, int value) {
  char temp[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  itoa(value, temp, 10);
  lcdDisplay_char_f6x8(x, y, temp);
}

void showScore(void) {
  lcdDisplay_char_f6x8(0, 2, "SCORE:");
  doNumber(44, 2, score);
}

void lcdDisplay_send_byte(uint8_t input) {
  uint8_t *disp;

  disp = gb.display.getBuffer();

  if ( (currentX >= screenLeft) && (currentX < screenLeft + 84) && (currentY >= screenTop) && (currentY <= screenTop + 5)) {
    *(disp + currentX - screenLeft + ((currentY - screenTop) * 84)) = input;
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

  initScreen();
  resetDock(0);

  drawFrog(frogMode,0);
  drawLives();
  
  doNumber(0,7,score);

  drawGameScreen(frogMode);
  drawDocks();

  screenLeft = 0;
  screenTop = 0;
  
  while(!gb.update());
  while (lives >= 0) {
    drawFrog(frogMode,0);
    drawLives();
    doNumber(0,7,score);  
    drawGameScreen(frogMode);
    drawDocks();
    
    if (frogRow == 1 || frogRow == 3) {
      screenLeft = frogColumn*8 - 30 - blockShiftL;
    } else if (frogRow == 2) {
      screenLeft = frogColumn*8 - 30 + blockShiftR;
    } else {
      screenLeft = frogColumn*8 - 30;
    }
    
    screenTop =  frogRow - 3;

    if (screenLeft < 0) screenLeft = 0;
    if (screenTop < 0) screenTop = 0;
    if (screenLeft > 43) screenLeft = 43;
    if (screenTop > 2) screenTop = 2;
    
    while(!gb.update());
    interimStep++;

    if (watchDog >= 500) lives = -1; // Stop the game if nothing's happening - maybe triggered in someone's pocket so this is to save battery!
    
    // Calculate left limit of frog movement so it doesn't hit the score
    frogLeftLimit = 1;
    if ((score / 10) % 10 != 0) frogLeftLimit++; 
    if ((score / 100) % 10 != 0) frogLeftLimit++;
    if ((score / 1000) % 10 != 0) frogLeftLimit++;

    // Move stuff along if it's time to
    //if (interimStep > moveDelay/8) {
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
      /*
      if (stopAnimate == 0) {
        drawGameScreen(frogMode);
        drawFrog(frogMode,0);
      }
      */
    //}

    // Handle input 
    if (gb.buttons.pressed(BTN_UP) == true && clickLock == 0) {
      moveForward = 1;
      watchDog = 0;   // reset the watchdog so the game doesn't end!
      clickLock = 1;
      clickBase = millis();
    }

    /* DISABLE THIS FOR NOW
    if (gb.buttons.pressed(BTN_DOWN) == true && clickLock == 0) {
      moveBack = 1;
      watchDog = 0;   // reset the watchdog so the game doesn't end!
      clickLock = 1;
      clickBase = millis();
    }
    */

    if (gb.buttons.pressed(BTN_LEFT) == true && clickLock == 0) {
      moveLeft = 1;
      watchDog = 0;   // reset the watchdog so the game doesn't end!
      clickLock = 1;
      //clickBase = millis();
    }

    if (gb.buttons.pressed(BTN_RIGHT) == true && clickLock == 0) {
      moveRight = 1;
      watchDog = 0;   // reset the watchdog so the game doesn't end!
      clickLock = 1;
      //clickBase = millis();
    }

    // Handle moving left
//    if(moveLeft == 1 && millis() > clickBase + CLICKDELAY/2) {
    if(moveLeft == 1) {
      watchDog = 0;   // reset the watchdog so the game doesn't end!
      moveLeft = 0;
      drawFrog(0,0);  // delete the frog
      // move the frog, checking it isn't jumping off the edge of the screen
      if ((frogRow == 7 && frogColumn > frogLeftLimit) || (frogRow < 7 && frogColumn > 0)) {
        frogColumn --;
      } else if (frogRow < 7) stopAnimate = 1;
      frogMode = 2; // pointing left
    }

    // Handle moving right
//    if(moveRight == 1 && millis() > clickBase + CLICKDELAY/2){
    if(moveRight == 1){
      watchDog = 0;   // reset the watchdog so the game doesn't end!
      moveRight = 0;
      drawFrog(0,0); // delete the frog
      // move the frog, checking it isn't jumping off the edge of the screen
      if ((frogRow == 7 && frogColumn < frogRightLimit) || (frogRow < 7 && frogColumn < 14)) {
        frogColumn ++;
      } else if (frogRow < 7) stopAnimate = 1;
      frogMode = 3; // pointing right    
    }


    // Handle 'move forward' button press
    if (moveBack == 1) {
      moveBack = 0;
      if (frogRow < 7) {
        // Correct for the skew in frog position created by the blockShift scrolling parameter
        if (frogRow == 3 && blockShiftL < 4) frogColumn++;
        if (frogRow == 2 && blockShiftR + blockShiftL < 5) frogColumn--;
        if (frogRow == 1 && blockShiftR + blockShiftL < 5) frogColumn++;
        frogRow++; 
        frogMode = 1;             // mode 1 = forwards position              
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
          drawFrog(frogMode,0);
          drawLives();
          doNumber(0,7,score);  
          drawGameScreen(frogMode);
          drawDocks();
          while(!gb.update());
          for (int i = 1000; i>200; i = i - 100){             // make sound
          beep(10,i);
          }
          delay(600);
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
    
    //if (clickLock == 1 && millis() > clickBase + CLICKDELAY && digitalRead(2)==0 && digitalRead(0)==0 && analogRead(0) > 940) clickLock = 0; // normal debounce
    clickLock = 0; // No Debounce

    // check to see if the frog has been killed
    if (stopAnimate != 0) {
      // redraw the screen
      drawGameScreen(frogMode);
      // animation for frog death
      drawGameScreen(frogMode);
      drawDocks();
      drawFrog(0,1);
      while(!gb.update());
      for (int i = 0; i<250; i = i+ 50){  
        beep(50,i);
      }
      drawGameScreen(frogMode);
      drawDocks();
      drawFrog(frogMode,1);    
      for (int i = 250; i<500; i = i+ 50){  
        beep(50,i);
      }
      drawGameScreen(frogMode);
      drawDocks();
      drawFrog(0,1);
      while(!gb.update());
      for (int i = 500; i<750; i = i+ 50){  
        beep(50,i);
      }
      drawGameScreen(frogMode);
      drawDocks();
      drawFrog(frogMode,1);
      for (int i = 750; i<1000; i = i+ 50){  
        beep(50,i);
      }
      while(!gb.update());
      delay(600);
      lives--;          // increment the score for every move
      frogRightLimit++; // there's one less frog drawn on right so you can move a bit further across (if you really want to!)
      stopAnimate = 0;  // reset parameter
      drawLives();      // display number of lives left
      frogColumn = 8;   // reinitalise frog location
      frogRow = 7;
      while(!gb.update());
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
      lcdDisplay_setpos(frogColumn*8 + 7 - blockShiftL,frogRow);
    } else if (frogRow == 2) {
      lcdDisplay_setpos(frogColumn*8 + blockShiftR,frogRow);      
    } else {
      lcdDisplay_setpos(frogColumn*8,frogRow); 
    }
    sendBlock(mode,0);                   // draw the frog - mode is direction
  }    
}

// Display the frog and all the moving items on the screen
void drawGameScreen(byte mode) {
  bool inverse = 0;
  
  // Draw objects going left
  for (byte row = 0; row < 6; row+=2) {
    if (row >=0 && row < 3) inverse = 1; else inverse = 0;                              // draw everything (except the frog) in inverse video on the river rows (0,1,2)
    lcdDisplay_setpos(0,row+1);                                                            // +1 because row 0 here is actually row 1 on the screen
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

  }
  if (frogColumn == 0) drawFrog(mode,1); // this covers the exceptional case where the frog is in the far left colum, in which case the normal routine can't draw it when it's on the road
  
  // Draw objects going right - see comments above, works in basically the same way
  for (byte row = 1; row < 6; row+=2) {
    if (row > 0 && row < 3) inverse = 1; else inverse = 0;
    lcdDisplay_setpos(0,row+1);
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
  }
  if (frogColumn == 0) drawFrog(mode,1);
}

// Send one byte to the screen
void sendByte(byte fill, bool inverse) {
  if (inverse == 0) lcdDisplay_send_byte(fill); else lcdDisplay_send_byte(~fill);
}

// Send one block of 8 bytes to the screen - inverse means inverse video, for the river section
void sendBlock(byte fill, bool inverse){
  for (int incr = 0; incr < 8; incr++) {
    if (fill > 0) {
      if (inverse == 0) lcdDisplay_send_byte(pgm_read_byte(&bitmaps[fill-1][incr])); else lcdDisplay_send_byte(~pgm_read_byte(&bitmaps[fill-1][incr]));  
    } else if (inverse ==0) lcdDisplay_send_byte(0); else lcdDisplay_send_byte(0xFF); 
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
    lcdDisplay_setpos(drawPos,0);
    lcdDisplay_send_byte(B11111111);
    lcdDisplay_send_byte(B00000001);
    lcdDisplay_send_byte(B00000001);
    if (frogDocks[incr] == 1) sendBlock(1,0); else for(byte lxn = 0; lxn < 8; lxn++) lcdDisplay_send_byte(B00000001);
    lcdDisplay_send_byte(B00000001);
    lcdDisplay_send_byte(B00000001);
    lcdDisplay_send_byte(B11111111);
    drawPos+= 24;
    }    
}

// Set all the frog docks to a single value
void resetDock(byte value) {   for (byte incr = 0; incr < 5;incr++) frogDocks[incr] = value; }

// Handle what happens at the end of a level
void levelUp(int number) {  
  // Flash the frog docks
  screenLeft = 0;
  screenTop = 0;

  delay(200);
  for (byte incr = 0; incr < 5; incr ++) {
    resetDock(0);
    drawDocks();
    drawGameScreen(frogMode);
    while(!gb.update());
    for (int i = 800; i>200; i = i - 200){
    beep(20,i);
    }
    resetDock(1);
    drawDocks();
    drawGameScreen(frogMode);
    while(!gb.update());
    for (int i = 800; i>200; i = i - 200){
    beep(20,i);
    }
  }  
  delay(500);
  lcdDisplay_fillscreen(0x00);  
  lcdDisplay_char_f6x8(14, 1,  "---------");
  lcdDisplay_char_f6x8(14, 2, "  LEVEL  ");
  lcdDisplay_char_f6x8(14, 4, "---------");
  doNumber(32, 3, level);
  while(!gb.update());
  delay(1500);    
  lcdDisplay_fillscreen(0x00);
  while(!gb.update());
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
