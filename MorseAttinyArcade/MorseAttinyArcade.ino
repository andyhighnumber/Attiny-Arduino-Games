/*  2015 / 2016 / 2017
 *  Morse decoder for Attiny85 and the #AttinyArcade by Andy Jackson (M0RCL) - Twitter @andyhighnumber
 *   
 *  Displays morse code coming in on the assigned pin in real time, via a 5-line scrolling display 
 *   
 *  Inspired by, and designed to run on the Attiny Arcade hardware. Check out www.webboggles.com for info on this hardware.
 *   
 *  When running, use:
 *   - The right button to send morse
 *   - The left button to start and stop the software running
 *   
 *  Any code not covered by the licences below can be used freely with attribution. No warranties whatsoever are provided or implied.
 *   
 *  ****************************************************************************************************
 *  * The core morse decoder here is a slightly modified version of WB7FHC's Morse Code Decoder v. 1.1 *
 *  * (c) 2014, Budd Churchward - WB7FHC                                                               *
 *  * This is an Open Source Project                                                                   *
 *  * http://opensource.org/licenses/MIT                                                               *
 *  * Search YouTube for 'WB7FHC' to see several videos of this project as it was developed.           *
 *  * Also see https://github.com/kareiva/wb7fhc-cw-decoder                                            *
 *  ****************************************************************************************************
 *  
 *  This sketch is using the screen control and font functions written by Neven Boyanov for the http://tinusaur.wordpress.com/ project
 *  Source code and font files available at: https://bitbucket.org/tinusaur/ssd1306xled
 * 
 *  Sleep code is based on this blog post by Matthew Little:
 *  http://www.re-innovation.co.uk/web12/index.php/en/blog-75/306-sleep-modes-on-attiny85
 */
 
#include <EEPROM.h>
#include "font6x8AJ.h"
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/interrupt.h> // needed for the additional interrupt

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

#define FIRSTLINE 2     // The first displayed line of decoded morse on the LCD
#define LASTLINE 6      // The last displayed line of decoded morse on the LCD
#define LINELENGTH 20   // The number of characters on one completed line
#define MAXWORD 17      // The longest any single word can be (needed to manage buffer limits)

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
void ssd1306_char_f6x8(uint8_t x, uint8_t y, char ch[]);
void ssd1306_draw_bmp(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t bitmap[]);

// Custom draw functions - allow for extra functionality like inverse display
void sendBlock(byte, bool);
void sendByte(byte, bool);

int stopRunning = 0;

// Other generic functions (both originated in code from webboggles.com and the sleep code is by Matthew Little - see above)
void beep(int,int);
void system_sleep(void);
void doNumber (int,int,int);

// Specific main function to decode morse
void decodeMorse(void);

// Variable declarations for decoder
int inputPin = 2;         // input data comes in here (key or decoder)
int audio = 1;            // will store the value we read on this pin
boolean mute = 0;         // Mute the speaker
int letterCount = 0;
int LCDline = FIRSTLINE;          // keeps track of which line we're printing on
boolean ditOrDah = true;  // We have either a full dit or a full dah
int dit;            

// The following values will auto adjust to the sender's speed
int averageDah;                   // A dah should be 3 times as long as a dit
int averageWordGap;               // will auto adjust

boolean characterDone = true; // A full character has been sent

int downTime = 0;        // How long the tone was on in milliseconds
int upTime = 0;          // How long the tone was off in milliseconds
int myBounce = 2;        // Used as a short delay between key up and down
long startDownTime = 0;  // Arduino's internal timer when tone first comes on
long startUpTime = 0;    // Arduino's internal timer when tone first goes off

int currentWordLength = 0;
int currentWordPos = 0;
char currentWord[MAXWORD];
char screenText[LASTLINE-FIRSTLINE+1][LINELENGTH+1];
int lastWordLength = 0;

boolean justDid = true; // Makes sure we only print one space during long gaps

int myNum = 0;           // We will turn dits and dahs into a binary number stored here
char mySet[] ="##TEMNAIOGKDWRUS##QZYCXBJP#L#FVH09#8###7#####/-61#######2###3#45";
char decodedCharacter = ' ';       // We will store the actual character decoded here

// Interrupt handlers
ISR(PCINT0_vect){ // PB0 pin button interrupt           
  stopRunning = 1; // stop the programme :)
}

void playerIncMorse(){ // PB2 pin button interrupt
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
//  pinMode(2, INPUT_PULLUP);
//  pinMode(1, OUTPUT);
  
  ssd1306_init();
  ssd1306_fillscreen(0x00);

  ssd1306_char_f6x8(0, 0, "  M O R S E");
  ssd1306_char_f6x8(0, 1, "      D E C O D E R");
  ssd1306_char_f6x8(27, 5, "andy jackson"); // see comments above !

  ssd1306_setpos(16,2); 
  for (int incr = 16; incr < 112; incr++) {
    ssd1306_send_data_start();
    ssd1306_send_byte(B00001010);
    ssd1306_send_data_stop();                    
  }
  ssd1306_setpos(27,4); 
  for (int incr = 0; incr < 72; incr++) {
    ssd1306_send_data_start();
    ssd1306_send_byte(B00100000);
    ssd1306_send_data_stop();                    
  }
  ssd1306_setpos(27,6); 
  for (int incr = 0; incr < 72; incr++) {
    ssd1306_send_data_start();
    ssd1306_send_byte(B00000100);
    ssd1306_send_data_stop();                    
  }

  delay(1500);

  long startT = millis();
  long nowT =0;
  boolean sChange = 0;

  while(digitalRead(0) == HIGH) {
    nowT = millis();
    if (nowT - startT > 2000) {
      sChange = 1;     
      if (mute == 0) { mute = 1; ssd1306_char_f6x8(32, 7, "-- MUTE --"); } else { mute = 0; ssd1306_char_f6x8(31, 7, "- SOUND ON -");  }    
      break;
    }
    if (sChange == 1) break;
  }  
  while(digitalRead(0) == HIGH);

  if (sChange == 0) {
    delay(1500);
    
    ssd1306_init();
    ssd1306_fillscreen(0x00);
  
    stopRunning = 0;
  
    decodeMorse(); 
  
    ssd1306_fillscreen(0x00);
    ssd1306_char_f6x8(11, 1, "----------------");
    ssd1306_char_f6x8(11, 2, " S T O P P E D");
    ssd1306_char_f6x8(11, 3, "----------------");
    ssd1306_char_f6x8(0, 6, "          bye..."); 

    delay(1200);    
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

void ssd1306_char_f6x8(uint8_t x, uint8_t y, char ch[]){
  uint8_t c,i,j=0;
  while(ch[j] != '\0')
  {
    c = ch[j] - 32;
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
 *  Main code
 */
void decodeMorse(){
  int wpm = 0;

  averageDah = 240;             // Starting point for auto-adjusting speed
  averageWordGap = averageDah;  // Will auto adjust
  dit = averageDah / 3;         
  lastWordLength = 0;
  letterCount = 0;
  LCDline = FIRSTLINE; 
  currentWordLength = 0;
  currentWordPos = 0;

  for (int i = 700; i>200; i = i - 50){
   beep(30,i);
  }

  ssd1306_char_f6x8(0, 0, "Listening.. WPM:");
  wpm = floor(1200 / (float(averageDah/3.0)) );
  doNumber(96,0,wpm);

  while(stopRunning == 0) {
     audio = digitalRead(inputPin); // What is the tone decoder doing?
  
     if (audio) { 
        keyIsDown();
        while(audio) {
          beep(50,250);
          audio = digitalRead(inputPin);
        }
        }      
     if (!audio) { 
        keyIsUp(); 
      }        
  }
} 


void keyIsDown() {
   // The decoder is detecting our tone
   // The LEDs on the decoder and Arduino will blink on in unison
   
   
   if (startUpTime>0){
     // We only need to do once, when the key first goes down
     startUpTime=0;    // clear the 'Key Up' timer
     }
   // If we haven't already started our timer, do it now
   if (startDownTime == 0){
       startDownTime = millis();  // get Arduino's current clock time
      }

     characterDone=false; // we're still building a character
     ditOrDah=false;      // the key is still down we're not done with the tone
     delay(myBounce);     // Take a short breath here
     
   if (myNum == 0) {      // myNum will equal zero at the beginning of a character
      myNum = 1;          // This is our start bit  - it only does this once per letter
      }
 }
 
  void keyIsUp() {
   // The decoder does not detect our tone
   // The LEDs on the decoder and Arduino will blink off in unison 
   
   // If we haven't already started our timer, do it now
   if (startUpTime == 0){startUpTime = millis();}
   
   // Find out how long we've gone with no tone
   // If it is twice as long as a dah print a space
   upTime = millis() - startUpTime;
   if (upTime<10)return;
   if (upTime > (averageDah*2)) {    
      printSpace();
   }
   
   // Only do this once after the key goes up
   if (startDownTime > 0){
     downTime = millis() - startDownTime;  // how long was the tone on?
     startDownTime=0;      // clear the 'Key Down' timer
   }
 
   if (!ditOrDah) {   
     // We don't know if it was a dit or a dah yet
      shiftBits();    // let's go find out! And do our Magic with the bits
    }

    // If we are still building a character ...
    if (!characterDone) {
       // Are we done yet?
       if (upTime > dit) { 
         // BINGO! we're done with this one  
         printCharacter();       // Go figure out what character it was and print it       
         characterDone=true;     // We got him, we're done here
         myNum=0;                // This sets us up for getting the next start bit
         }
         downTime=0;               // Reset our keyDown counter
       }
   }
   
   
void shiftBits() {
 
  // we know we've got a dit or a dah, let's find out which
  // then we will shift the bits in myNum and then add 1 or not add 1
  
  if (downTime < dit / 3) return;  // ignore my keybounce
  
  myNum = myNum << 1;   // shift bits left
  ditOrDah = true;        // we will know which one in two lines 
  
  
  // If it is a dit we add 1. If it is a dah we do nothing!
  if (downTime < dit) {
     myNum++;           // add one because it is a dit
     } else {
  
    // The next three lines handle the automatic speed adjustment:
    averageDah = (downTime+averageDah) / 2;  // running average of dahs

    if (averageDah > 1000) averageDah = 1000; // limiting slowest speed to about 3wpm !
   
    dit = averageDah / 3;                    // normal dit would be this
    dit = dit * 2;    // double it to get the threshold between dits and dahs
     }
  }


void printCharacter() {           
  justDid = false;         // OK to print a space again after this

  if (myNum == 511) {
    deleteLastWord();
    return;
  }
  
  // Punctuation marks will make a BIG myNum
  if (myNum > 63) {  
    printPunctuation();  // The value we parsed is bigger than our character array
                         // It is probably a punctuation mark so go figure it out.
    return;              
  }
  decodedCharacter = mySet[myNum]; 
  
  if (currentWordLength < MAXWORD) {
    currentWord[currentWordPos] = decodedCharacter;
    currentWordLength++;
    currentWordPos++;
    lastWordLength = currentWordLength; // this will ensure that lastWordLength is always the length of the most recent word
  } else printSpace();
  sendToLCD();      
}

void printSpace() {
  int wpm;

  if (justDid) return;  // only one space, no matter how long the gap
  justDid = true;       // so we don't do this twice

  currentWordLength = 0;
  currentWordPos = 0;

  wpm = floor(1200 / (float(averageDah/3.0)) );
  ssd1306_char_f6x8(96, 0, "   ");
  doNumber(96,0,wpm);
  
  // We keep track of the average gap between words and bump it up 20 milliseconds
  // do avoid false spaces within the word
  averageWordGap = ((averageWordGap + upTime) / 2) + 20;

  decodedCharacter=' ';            // this is going to go to the LCD 
  
  sendToLCD();         // go figure out where to put it on the display
}

void printPunctuation() {
  // Punctuation marks are made up of more dits and dahs than
  // letters and numbers. Rather than extend the character array
  // out to reach these higher numbers we will simply check for
  // them here. This funtion only gets called when myNum is greater than 63
  
  // Thanks to Jack Purdum for the changes in this function
  // The original uses if then statements and only had 3 punctuation
  // marks. Then as I was copying code off of web sites I added
  // characters we don't normally see on the air and the list got
  // a little long. Using 'switch' to handle them is much better.

  lastWordLength = currentWordLength; // this will ensure that lastWordLength is always the length of the most recent word
  
  switch (myNum) {
    case 71:
      decodedCharacter = ':';
      break;
    case 76:
      decodedCharacter = ',';
      break;
    case 84:
      decodedCharacter = '!';
      break;
    case 94:
      decodedCharacter = '-';
      break;
    case 97:
      decodedCharacter = 39;    // Apostrophe
      break;
    case 101:
      decodedCharacter = '@';
      break;
    case 106:
      decodedCharacter = '.';
      break;
    case 115:
      decodedCharacter = '?';
      break;
    case 246:
      decodedCharacter = '$';
      break;
    case 122:
      decodedCharacter = 's';
      sendToLCD();
      decodedCharacter = 'k';
      break;
    default:
      decodedCharacter = '#';    // Should not get here
      break;
  }
  sendToLCD();    // go figure out where to put it on the display
}

void sendToLCD(){
  uint8_t c,i;

  screenText[LCDline-FIRSTLINE][letterCount] = decodedCharacter;
  
  c = decodedCharacter - 32;
  ssd1306_setpos(letterCount*6,LCDline);
  ssd1306_send_data_start();
  for(i=0;i<6;i++)
  {
    ssd1306_send_byte(pgm_read_byte(&ssd1306xled_font6x8[c*6+i]));
  }
  ssd1306_send_data_stop();

  letterCount++; 

  if (letterCount > LINELENGTH) {
    newLine(); 
  }
}

void newLine() {
    int scrollNow = 0;
    int oldLine = LCDline;
    int scrolled = 0;
    
    if (LCDline == LASTLINE) {scrollNow = 1; oldLine = LASTLINE-1; scrolled = 1;} else {LCDline++; scrollNow = 0;}
    
    if (scrollNow == 1) scrollUp();
    blankLine(LCDline);
    
    letterCount = 0;


    if (decodedCharacter > ' ') {   
      for (int i = 0; i < currentWordLength; i++) {
        decodedCharacter = currentWord[i];
        sendToLCD();                        
        screenText[oldLine-FIRSTLINE][LINELENGTH-i] = ' '; 
      }  
      ssd1306_setpos(126-(currentWordLength*6),oldLine);
      ssd1306_send_data_start();
      for(int i=0;i<(currentWordLength*6);i++)
      {
        ssd1306_send_byte(0);
      }
      ssd1306_send_data_stop();
    }
}

void blankLine(int line) {
  uint8_t i;
 
  ssd1306_setpos(0,LCDline);
  ssd1306_send_data_start();
  for(i=0;i<128;i++)
  {
    ssd1306_send_byte(0);
  }
  ssd1306_send_data_stop();
}

void scrollUp() {
  int i,j,k,c;

  for(i = 0; i < LASTLINE-FIRSTLINE; i++) {
    for (j=0;j<LINELENGTH+1;j++) {
      screenText[i][j] = screenText[i+1][j];
      c = screenText[i][j] - 32;
      ssd1306_setpos(j*6,i+FIRSTLINE);
      ssd1306_send_data_start();
      for(k=0;k<6;k++)
      {
        ssd1306_send_byte(pgm_read_byte(&ssd1306xled_font6x8[c*6+k]));
      }
      ssd1306_send_data_stop();      
    }
  }  
}

void deleteLastWord() {

  if (lastWordLength < 1) return; 

  if (screenText[LCDline-FIRSTLINE][letterCount-1] == ' ') {
    letterCount--;
  }
  letterCount--;

  for(int i = 0; i < lastWordLength; i++) {
    ssd1306_setpos(letterCount*6,LCDline);
    ssd1306_send_data_start();
    for(int k=0;k<6;k++)
    {
      ssd1306_send_byte(0);
    }
    ssd1306_send_data_stop();    
    letterCount--;
  }
  letterCount++; // we will have gone one further back than the start of the deleted word
  lastWordLength = 0;
  if (letterCount < 0) letterCount = 0;
  justDid = true; // we don't want to stick another space in after deleting
}


