/* 2015 / 2016 / 2017
 * Stacker game by Andy Jackson @andyhighnumber
 * UFO game by Ilya Titov. Find building instructions on http://webboggles.com/
 * 
 * The code that does not fall under the licenses of sources listed below can be used non commercially with attribution.
 *
 * Modified by andy jackson to allow two games to fit into a single ATTiny85 at the same time.
 * Additional features:
 *  - To play Stacker - press and release left button
 *  - To play UFO - with the right button held, press and release left button 
 *  - To turn sound on and off - press and HOLD left button
 *  - To reset high scores to zero - whilst HOLDING the right button, press and HOLD the left button
 * 
 * If you have problems uploading this sketch, this is probably due to sketch size - you need to update ld.exe in arduino\hardware\tools\avr\avr\bin
 * https://github.com/TCWORLD/ATTinyCore/tree/master/PCREL%20Patch%20for%20GCC
 *
 * This sketch is using the screen control and font functions written by Neven Boyanov for the http://tinusaur.wordpress.com/ project
 * Source code and font files available at: https://bitbucket.org/tinusaur/ssd1306xled
 * **Note that this highly size-optimised version requires modified library functions (which are in this source code file) 
 * and a modified font header
 * 
 * Sleep code is based on this blog post by Matthew Little:
 * http://www.re-innovation.co.uk/web12/index.php/en/blog-75/306-sleep-modes-on-attiny85
*/
#include <EEPROM.h>
#include "font6x8AJ.h"
#include <avr/sleep.h>
#include <avr/interrupt.h> // needed for the additional interrupt

#define DIGITAL_WRITE_HIGH(PORT) PORTB |= (1 << PORT)
#define DIGITAL_WRITE_LOW(PORT) PORTB &= ~(1 << PORT)

// Routines to set and clear bits (used in the sleep code)
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// Defines for OLED output
#define SSD1306XLED_H
#define SSD1306_SCL   PORTB4  // SCL, Pin 3 on SSD1306 Board
#define SSD1306_SDA   PORTB3  // SDA, Pin 4 on SSD1306 Board
#define SSD1306_SA    0x78  // Slave address

// Function prototypes for UFO
void beep(int,int);
void playUFO(void);

// Function prototypes for Stacker
void resetBlocks(void);
void drawRow(int thisRow, int location);
void sendBlock(boolean);
void playStacker(void);
void beep(int,int);
void levelUp(int);

// Other function prototypes - for both
void doNumber (int x, int y, int value);
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

// for UFO
int obstacle[9] = {-50,-50,-50,-50,-50,-50,-50,-50,-50}; // x offset of the obstacle default position, out of view
byte gapOffset[9] = {0,0,0,0,0,0,0,0,0}; // y offset of the fly-through gap
int gapSize[9]; // y height of the gap
byte maxGap = 60; // max height of the gap
int stepsSinceLastObstacle = 0; // so obstacles are not too close
byte gapBlock[9] = {0,0,0,0,0,0,0,0,0}; // if the fly-through gap is closed
int topScoreU = 0;
boolean isStacker = 0;
byte maxObstacles = 1; // this defines the max number of in game obstacles
byte obstacleStep = 2; // pixel step of obstacles per frame
byte blockChance = 0; // this higher value decreases the likelihood of gap being closed 
byte fireCount = 0; // the shot is persistent for several frames
byte playerOffset = 0; // y offset of the top of the player
byte flames = 0; // this is set to 1 when the move up interrupt is triggered
byte flameMask[2]={B00111111,B11111111}; // this is used to only show the flame part of the icon when moving up
void doDrawLS(long, byte);
void doDrawRS(long, byte);


// For Stacker
boolean perfect = 1;
int runCounter = 0;
boolean fireLock = 0;
boolean fireCounterActive = 0;
int fireCounter = 0;
int level = 1; // Game level - incremented every time you clear the screen
boolean blockDirection = 1; // current direction of travel for blocks - 1 is right 0 is left
int stackRow = 0; // which row are we considering
int movecounter = 0; // timer for movement
int matchCount = 6;
boolean row[2][16]; // on-off array of blocks - building up from the bottom

boolean fire = 0;
int topScoreB = 0;
volatile int player = 0; //0 to 128-platformWidth  - this is the position of the player

// For both
boolean stopAnimate = 0; // this is set to 1 when a collision is detected
boolean mute = 0;
boolean newHigh = 0;
int score = 0; // score - this affects the difficulty of the game
int top = 0;

// Interrupt handlers
ISR(PCINT0_vect){ // PB0 pin button interrupt           
}
void playerIncUFO(){ // PB2 pin button interrupt
   fire = 1;
   fireCount = 5; // number of frames the shot will persist
}
void playerIncStacker(){ // PB2 pin button interrupt
  
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
  isStacker = 1;
  
  if (digitalRead(2) == HIGH) isStacker = 0;
  
  //interrupts();
  ssd1306_init();
  ssd1306_fillscreen(0x00);
  
  if (isStacker) {
    for (int j = 0;j<16;j++) {
      row[0][j] = 0;
    }
    for (int i= 14;i<15;i++) {
      row[0][i] = 1;
    }
    drawRow(0,2);
  
    for (int i= 13;i<15;i++) {
      row[0][i] = 1;
    }
    drawRow(0,3);
  
    for (int i= 13;i<16;i++) {
      row[0][i] = 1;
    }
    drawRow(0,4);
  
    for (int i= 13;i<16;i++) {
      row[0][i] = 1;
    }
    drawRow(0,5);
  
    for (int i= 12;i<16;i++) {
      row[0][i] = 1;
    }
    drawRow(0,6);
  
    for (int i= 12;i<16;i++) {
      row[0][i] = 1;
    }
    drawRow(0,7);  

    ssd1306_char_f6x8(0, 2, "S T A C K E R");
    ssd1306_char_f6x8(0, 4, "andh jackson"); // see comments above !
    ssd1306_char_f6x8(0, 6, "inspired bh");  // see comments above !
    ssd1306_char_f6x8(0, 7, "/ebboggles.com");  // see comments above !
  } else {
    for (byte s = 0; s<30; s++){ // generate stars
       ssd1306_setpos((uint8_t)(random(0,127)),(uint8_t)(random(0,7)) );
       ssd1306_send_data_start();
       ssd1306_send_byte(B10000000>>random(0,7)); 
       ssd1306_send_data_stop();
    }
    ssd1306_char_f6x8(14, 2, "U F O  C H A O S");
    // The lower case character set is seriously compromised because I've had to truncate the ASCII table
    // to release space for executable code - hence lower case y and w are remapped to h and / respectively.
    // There is no z in the table (or h!) as these aren't used anywhere in the text here and most of the 
    // symbols are also missing for the same reason (see my hacked version of font6x8.h - font6x8AJ.h for more detail)
    ssd1306_char_f6x8(3, 4, "mods bh andh jackson"); // see comments above !
    ssd1306_char_f6x8(17, 6, "original game bh");  // see comments above !
    ssd1306_char_f6x8(22, 7, "/ebboggles.com");  // see comments above !
  }


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
        EEPROM.write(2,0);
        EEPROM.write(3,0);      
        ssd1306_char_f6x8(8, 0, "-HIGH SCORE RESET-");  
      } else if (mute == 0) { mute = 1; ssd1306_char_f6x8(32, 0, "-- MUTE --"); } else { mute = 0; ssd1306_char_f6x8(23, 0, "-- SOUND ON --");  }    
      break;
    }
    if (sChange == 1) break;
  }  
  while(digitalRead(0) == HIGH);

  if (sChange == 0) {
    delay(2600);
    //noInterrupts();  
    ssd1306_init();
    ssd1306_fillscreen(0x00);
    stopAnimate = 0;
    score = 0;
    
    if (isStacker) { playStacker(); top=topScoreB;} else { playUFO(); top=topScoreU;}
    
    ssd1306_fillscreen(0x00);
    ssd1306_char_f6x8(11, 1, "----------------");
    ssd1306_char_f6x8(11, 2, "G A M E  O V E R");
    ssd1306_char_f6x8(11, 3, "----------------");
    ssd1306_char_f6x8(37, 5, "SCORE:");
    doNumber(75, 5, score);
    if (!newHigh) {
      ssd1306_char_f6x8(21, 7, "HIGH SCORE:");
      doNumber(88, 7, top);
    }
    for (int i = 0; i<1000; i = i+ 50){
      beep(50,i);
    }
    delay(2000);
    if (newHigh) {
      ssd1306_fillscreen(0x00);
      ssd1306_char_f6x8(10, 1, "----------------");
      ssd1306_char_f6x8(10, 3,  " NEW HIGH SCORE ");
      ssd1306_char_f6x8(10, 7, "----------------");
      doNumber(50,5,top);
      for (int i = 700; i>200; i = i - 50){
      beep(30,i);
      }
      newHigh = 0;
      delay(2700);    
    } 
  }
  system_sleep();
  //noInterrupts();  
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
    ssd1306_send_command(0x00);   //low column start address
    ssd1306_send_command(0x10);   //high column start address
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

void system_sleep() {
  ssd1306_fillscreen(0x00);
  ssd1306_send_command(0xAE);
  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  sleep_mode();                        // System actually sleeps here
  sleep_disable();                     // System continues execution here when watchdog timed out 
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON  
  ssd1306_send_command(0xAF);
}

void beep(int bCount,int bDelay){
  if (mute) return;
  for (int i = 0; i<=bCount; i++){digitalWrite(1,HIGH);for(int i2=0; i2<bDelay; i2++){__asm__("nop\n\t");}digitalWrite(1,LOW);for(int i2=0; i2<bDelay; i2++){__asm__("nop\n\t");}}
}

/* ------------------------
 *  UFO Code
 */

void playUFO(void) {
  int fireLock = 0;
  fire = 0;
  maxObstacles = 3;
  obstacleStep = 2;
  for (byte i = 0; i<9; i++){
    obstacle[i] = -50;
    gapOffset[i]=0;
  }
  stepsSinceLastObstacle = 0;
  playerOffset = 0; // y offset of the top of the player
  fireLock = 0;
  
  while (stopAnimate == 0) {

  // deal with inputs
  if(analogRead(0) < 940) { 
    if (fireLock == 0) {
      fire =1; fireCount = 5;
      fireLock = 1;
    } 
  } else fireLock = 0;


  attachInterrupt(0,playerIncUFO,RISING);
     
  //update game vars to make it harder to play
  if (score < 500){blockChance = 11-score/50; if (maxObstacles<5){maxObstacles=(score+40)/70+1;} delayMicroseconds(16000/maxObstacles);}
  
  if (score < 2000){maxGap = 60-score/100;}
  if (fire == 1){score--;}
  if (fireCount>0){fireCount--;}

  for (int boo = 0; boo<3;boo++) {
    if (digitalRead(0)==1){
      if (playerOffset >0){
        playerOffset--; 
        flames = 1; // move player up
        for (int i = 0; i<2; i++){
          beep(1,random(0,i*2));
        }
      }
    }
  }
  
  stepsSinceLastObstacle += obstacleStep;
  for (byte i = 0; i<maxObstacles;i++){ // fly obstacles
  if (obstacle[i] >= 0 && obstacle[i] <= 128){
  obstacle[i] -= obstacleStep;
  if (gapBlock[i]>0 && obstacle[i] < 36  && playerOffset>gapOffset[i] && playerOffset+5<gapOffset[i]+gapSize[i] && fireCount > 0){//
   gapBlock[i] = 0;
   score += 5; 
   for (byte cp = 400; cp>0; cp--){
     beep(1,cp);
   }
  }
  } 
  
  if (obstacle[i]<=4 && stepsSinceLastObstacle>=random(30,100)){ // generate new obstacles
  obstacle[i] = 123;
  gapSize[i] = random(25,maxGap);
  gapOffset[i] = random(0,64-gapSize[i]);
  if (random(0,blockChance)==0){gapBlock[i] = 1;}else {gapBlock[i] = 0;}
  stepsSinceLastObstacle = 0;
  score+=1;
  }
  }
  
  if ( (fireCount == 0) && (playerOffset < 56) ){playerOffset++;} // player gravit
  
  // update what's on the screen
  // erase player
  for (byte r=0; r<8; r++){
  if (r<playerOffset/8 | r >= playerOffset/8+1) {
    ssd1306_setpos(0,r);
    ssd1306_send_data_start();
    sendBlock(0);
    sendBlock(0);
    ssd1306_send_data_stop();
    }
  }
  
  //erase fire
  ssd1306_setpos(16,playerOffset/8);
  ssd1306_send_data_start();
  if (fireCount == 0){
            for (byte f = 0; f<=30; f++){
                ssd1306_send_byte(B00000000);
            }
  }
  ssd1306_send_data_stop();
  
  // Send Obstacle
  for (byte i = 0; i<maxObstacles;i++){
  if (obstacle[i] >= -5 && obstacle[i] <= 128){ // only deal with visible obstacles
  if (obstacle[i] > 8 && obstacle[i] <16){ // look for collision if obstacle is near the player
    if (playerOffset < gapOffset[i] || playerOffset+5 > gapOffset[i]+gapSize[i] || gapBlock[i] != 0){
      // collision - stop looping
      stopAnimate = 1; 
    }
  }                      
  
  for (byte row = 0; row <8; row++){
      ssd1306_setpos(obstacle[i],row);
      ssd1306_send_data_start();
      
      if (obstacle[i]>0&&obstacle[i] < 128){
         
         if ((row+1)*8 - gapOffset[i] <= 8){ // generate obstacle : top and transition
            byte temp = B11111111>>((row+1)*8 - gapOffset[i]); 
            byte tempB = B00000000; 
            if (gapBlock[i]>0){tempB=B10101010;}
            ssd1306_send_byte(temp);
            ssd1306_send_byte(temp|tempB>>1);
            ssd1306_send_byte(temp|tempB);
            ssd1306_send_byte(temp);
            
         }else if (row*8>=gapOffset[i] && (row+1)*8<=gapOffset[i]+gapSize[i]){ // middle gap
            byte tempB = B00000000; 
            if (gapBlock[i]>0){tempB=B10101010;}
            ssd1306_send_byte(B00000000);
            ssd1306_send_byte(B00000000|tempB>>1);
            ssd1306_send_byte(B00000000|tempB);
            ssd1306_send_byte(B00000000);
  
         }else if ((gapOffset[i] +gapSize[i]) >= row*8 && (gapOffset[i] +gapSize[i]) <= (row+1)*8){ // bottom transition
            //}else if ((gapOffset[i] +gapSize[i]) >= row*8 && (gapOffset[i] +gapSize[i]) <= (row+1)*8){ // bottom transition
            //byte temp = B11111111<<((gapOffset[i] + gapSize[i])%8); 
            
            byte temp = B11111111<<((gapOffset[i] + gapSize[i])%8); 
            byte tempB = B00000000; 
            if (gapBlock[i]>0){tempB=B10101010;}
            ssd1306_send_byte(temp);
            ssd1306_send_byte(temp|tempB>>1);
            ssd1306_send_byte(temp|tempB);
            ssd1306_send_byte(temp);
            
         }else { // fill rest of obstacle
            ssd1306_send_byte(B11111111);
            ssd1306_send_byte(B11111111);
            ssd1306_send_byte(B11111111);
            ssd1306_send_byte(B11111111);
         }
         
         ssd1306_send_byte(B00000000);
         ssd1306_send_byte(B00000000);
         
      ssd1306_send_data_stop();
      }
    }
  
  }
  }
   
  if (playerOffset%8!=0){
    ssd1306_setpos(8,playerOffset/8);
    ssd1306_send_data_start();
    doDrawLS(0,playerOffset%8);
    ssd1306_send_data_stop();
    
    ssd1306_setpos(8,playerOffset/8+1);
    ssd1306_send_data_start();
    doDrawRS(0,8-playerOffset%8);
    ssd1306_send_data_stop();
  } else {
    ssd1306_setpos(8,playerOffset/8);
    ssd1306_send_data_start();
    doDrawLS(0,0);
    ssd1306_send_data_stop();
  }

  // Display score in the screen corner
  doNumber(92, 0, score);
  
  flames = 0;
  //interrupts();
  } // while stopanimate == 0

  topScoreU = EEPROM.read(0);
  topScoreU = topScoreU << 8;
  topScoreU = topScoreU |  EEPROM.read(1);
  
  if (score > topScoreU) {
    topScoreU = score;
    EEPROM.write(1,score & 0xFF); 
    EEPROM.write(0,(score>>8) & 0xFF); 
    newHigh = 1;
  }
}

// Drawing routine for the player and fire - with right-shifts
void doDrawRS(long P1, byte P2) {
  ssd1306_send_byte((B00001100&flameMask[flames] | P1)>>P2);
  ssd1306_send_byte((B01011110&flameMask[flames] | P1)>>P2);
  ssd1306_send_byte((B10010111&flameMask[flames] | P1)>>P2);
  ssd1306_send_byte((B01010011&flameMask[flames] | P1)>>P2);
  ssd1306_send_byte((B01010011&flameMask[flames] | P1)>>P2);
  ssd1306_send_byte((B10010111&flameMask[flames] | P1)>>P2);
  ssd1306_send_byte((B01011110&flameMask[flames] | P1)>>P2);
  ssd1306_send_byte((B00001100&flameMask[flames] | P1)>>P2);  
  
  if (fireCount >0) {
        for (byte f = 0; f<=24; f++){
            ssd1306_send_byte(B00000100>>P2);
        }
        ssd1306_send_byte(B00010101>>P2);
        //ssd1306_send_byte(B00001010>>P2);
        ssd1306_send_byte(B00010101>>P2);
        if (fire==1){beep(50,100);}
        fire = 0;
  } 
}

// Drawing routine for the player and fire - with left-shifts
void doDrawLS(long P1, byte P2) {
  ssd1306_send_byte((B00001100&flameMask[flames] | P1)<<P2);
  ssd1306_send_byte((B01011110&flameMask[flames] | P1)<<P2);
  ssd1306_send_byte((B10010111&flameMask[flames] | P1)<<P2);
  ssd1306_send_byte((B01010011&flameMask[flames] | P1)<<P2);
  ssd1306_send_byte((B01010011&flameMask[flames] | P1)<<P2);
  ssd1306_send_byte((B10010111&flameMask[flames] | P1)<<P2);
  ssd1306_send_byte((B01011110&flameMask[flames] | P1)<<P2);
  ssd1306_send_byte((B00001100&flameMask[flames] | P1)<<P2);  
  
  if (fireCount >0) {
        for (byte f = 0; f<=24; f++){
            ssd1306_send_byte(B00000100<<P2);
        }
        ssd1306_send_byte(B00010101<<P2);
        //ssd1306_send_byte(B00001010<<P2);
        ssd1306_send_byte(B00010101<<P2);
        if (fire==1){beep(50,100);}
        fire = 0;
  }   
}

/* ------------------------
 *  Stacker Code
 */
void playStacker() {
  runCounter = 0;
  movecounter = 0;
  level = 1; // Game level - incremented every time you clear the screen
  blockDirection = 1; // current direction of travel for blocks - 1 is right 0 is left
  stackRow = 6; // what's the index of the current active row (0 is the top line, 7 is the bottom)?
  topScoreB = 0; // highscore
  fire = 0; // make sure no fire
  score = 0; // obvious
  matchCount = 6;
  fireLock = 0;
  fireCounterActive = 0;
  fireCounter = 0;
  perfect = 1;

  resetBlocks();
 
  //attachInterrupt(0,playerIncStacker,CHANGE);
  
  // draw base row
  drawRow(1,7);

  while (stopAnimate == 0) {
    while(1) {
    movecounter++;

    doNumber(0,7,score);

    // All this stuff is a complex debounce on the fire/trigger button
    if (fireCounterActive) fireCounter++;
    if ( (fireCounter >= 10) && fireCounterActive ) {
      fireCounterActive = 0;
      fireLock = 0;
      fire = 0;
    }
        
    if(digitalRead(2)==1) {
      while(digitalRead(2)==1);
      fire = 1;
    }

    if(digitalRead(0)==1){
      while(digitalRead(0)==1);
      fire = 1;
    }

    if(analogRead(0) < 940) {
      while(analogRead(0) < 940);
      fire = 1;
    }

    drawRow(0,stackRow);     


    // Fire / stop the blocks!
    if ( (fireLock == 0) && (fire == 1) ) { // fire has been pressed - do stuff - check for alignment, update the array and decrement active row
      int matchTarget = matchCount;
      runCounter = 0;
      matchCount = 0;
      for (int i = 0; i<16;i++) {
        if (row[1][i] == 1) {
          if (row[0][i] == 1) {
            matchCount++;
            row[1][i] = 1;
          } else {
            row[1][i] = 0;
          }
        }
      }
      if (matchCount < matchTarget) perfect = 0;

      // draw stack row - now fixed
      drawRow(1,stackRow);
                                          
      stackRow--;

      if (matchCount>0 ) {
        score+= (7-stackRow);
        beep(50,300);
      }
      fireLock = 1; // lock fire until the button is lifted and the timeout is done
      fire = 0;
      fireCounter = 0;
      fireCounterActive = 1;

      // fill the next row (unless we're already at the top! - arduino hates attempts to access arrays with negative indices
      if (stackRow >=0) {
        for (int i=0;i<matchCount;i++) {
          row[0][i] =1;
        } 
        for (int i=matchCount; i<16;i++) {
          row[0][i] =0;
        }
      }
    } // end of fire routine
   
    // Move the blocks
    int speedNow = 13-level;
    if (speedNow < 4) speedNow = 4;
    if (movecounter >= speedNow) {
        movecounter = 0;
      if(blockDirection) { // Moving right
        if (row[0][15] == 1) { // we've hit the end
          runCounter++;
          if (runCounter > 20) { // runCounter kills the game if no activity for 20 cycles
            stopAnimate = 1;     // this is to prevent the battery being drained if the game is accidentally started
            break;
          }

          blockDirection = 0;
        } else {
          for(int i=15;i>0;i--) { // shuffle right
            row[0][i] = row[0][i-1];
          } 
          row[0][0] = 0;          
        }
      } else { // Moving left

        if (row[0][0] == 1) { // we've hit the end
          blockDirection = 1;
        } else {
          for(int i=0;i<15;i++) { // shuffle left
            row[0][i] = row[0][i+1];
          } 
          row[0][15] = 0;          
        }
      } 
    }

    if(matchCount == 0) { // deal with failure here 
      stopAnimate = 1;
      break;
    }

    // Level up
    if (stackRow < 0) {
      if (level <6) {
        matchCount = 6;
      } else if (level < 9) {
        matchCount = 5;
      } else if (level < 12) {
        matchCount = 4;
      } else if (level < 15) {
        matchCount = 3;
      } else {
        matchCount = 2;
      }
      fireCounter = 0;
      fireLock = 0;
      fireCounterActive = 0;
      movecounter=0;
      stackRow = 6;
      level++;
      levelUp(level);
      blockDirection = 1;
      perfect = 1;
      resetBlocks();
      fire =0;
      
      // draw base row
      drawRow(1,7);

      for (int i = 700; i>100; i = i - 100){
      beep(30,i);
      }
    }
   }
 }
die:
  topScoreB = EEPROM.read(2);
  topScoreB = topScoreB << 8;
  topScoreB = topScoreB |  EEPROM.read(3);

  if (score > topScoreB) { 
    topScoreB = score;
    EEPROM.write(3,score & 0xFF); 
    EEPROM.write(2,(score>>8) & 0xFF); 
    newHigh = 1;
    }
  }

void levelUp(int number) {
  ssd1306_fillscreen(0x00);
  ssd1306_char_f6x8(16, 1, "--------------");
  ssd1306_char_f6x8(16, 2, " L E V E L ");
  ssd1306_char_f6x8(16, 3, "--------------");
  if (perfect) {
     ssd1306_char_f6x8(25, 5, "PERFECT LEVEL");
     ssd1306_char_f6x8(25, 7, "  100 BONUS");
     score += 100;
  }
  doNumber(85,2,number);
  for (int i = 800; i>200; i = i - 200){
  beep(30,i);
  }
  delay(1500);    
  ssd1306_fillscreen(0x00);
}

void resetBlocks(void) {
  for (byte j = 0;j<16;j++) {
    row[0][j] = 0;
  }
  for (byte j = 0;j<16;j++) {
    row[1][j] = 0;
  }

  for (int i=6;i<6+matchCount;i++) {
    row[1][i] = 1;
  }
  
  for (int i=0;i<matchCount;i++) {
    row[0][i] =1;
  }
}


void drawRow(int thisRow, int location) {
  if (thisRow <0) return;
  if (thisRow>1) return;
  ssd1306_setpos(0,location);
  ssd1306_send_data_start();
  for(int j=0;j<16;j++) {
    if (row[thisRow][j] == 1) {
      sendBlock(1);
    } else {
      sendBlock(0);
    }
  }
  ssd1306_send_data_stop();                    
}


void sendBlock(boolean fill){
  if (fill==1){
   ssd1306_send_byte(B00000000);
   ssd1306_send_byte(B01111110);
   ssd1306_send_byte(B01111110);
   ssd1306_send_byte(B01111110);
   ssd1306_send_byte(B01111110);
   ssd1306_send_byte(B01111110);
   ssd1306_send_byte(B01111110);
   ssd1306_send_byte(B00000000);
  }else {
   ssd1306_send_byte(B00000000);
   ssd1306_send_byte(B00000000);
   ssd1306_send_byte(B00000000);
   ssd1306_send_byte(B00000000);
   ssd1306_send_byte(B00000000);
   ssd1306_send_byte(B00000000);
   ssd1306_send_byte(B00000000);
   ssd1306_send_byte(B00000000);
  } 
}



