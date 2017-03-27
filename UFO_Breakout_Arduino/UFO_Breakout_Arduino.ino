/* 2015 / 2016
 * UFO and Breakout games by Ilya Titov. Find building instructions on http://webboggles.com/
 * The code that does not fall under the licenses of sources listed below can be used non commercially with attribution.
 *
 * Modified by andy jackson to allow two games to fit into a single ATTiny85 at the same time.
 * Additional features:
 *  - To play UFO - press and release left button
 *  - To play Breakout - with the right button held, press and release left button 
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
#define SSD1306_SCL   PORTB3  // SCL, Pin 3 on SSD1306 Board
#define SSD1306_SDA   PORTB4  // SDA, Pin 4 on SSD1306 Board
#define SSD1306_SA    0x78  // Slave address

// Function prototypes for UFO
void beep(int,int);
void playUFO(void);

// Function prototypes for Breakout
void collision(void);
void drawPlatform(void);
void sendBlock(boolean);
void playBreakout(void);

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
int topScoreU = 0;
boolean isBreakout = 0;
byte maxObstacles = 1; // this defines the max number of in game obstacles
byte obstacleStep = 2; // pixel step of obstacles per frame
int obstacle[9] = {-50,-50,-50,-50,-50,-50,-50,-50,-50}; // x offset of the obstacle default position, out of view
byte gapOffset[9] = {0,0,0,0,0,0,0,0,0}; // y offset of the fly-through gap
int gapSize[9]; // y height of the gap
byte maxGap = 60; // max height of the gap
int stepsSinceLastObstacle = 0; // so obstacles are not too close
byte gapBlock[9] = {0,0,0,0,0,0,0,0,0}; // if the fly-through gap is closed
byte blockChance = 0; // this higher value decreases the likelihood of gap being closed 
boolean fire = 0; // set to 1 when the fire interrupt is triggered
byte fireCount = 0; // the shot is persistent for several frames
byte playerOffset = 0; // y offset of the top of the player
byte flames = 0; // this is set to 1 when the move up interrupt is triggered
byte flameMask[2]={B00111111,B11111111}; // this is used to only show the flame part of the icon when moving up
void doDrawLS(long, byte);
void doDrawRS(long, byte);

// For Breakout
int topScoreB = 0;
volatile byte player = 0; //0 to 128-platformWidth  - this is the position of the bounce platform
byte platformWidth = 16; 
byte ballx = 62; // coordinate of the ball
byte bally = 50; // coordinate of the ball
int vdir = -1; // vertical direction and step  distance
int hdir = -1; // horizontal direction and step distance
long lastFrame = 0; // time since the screen was updated last
boolean row[3][16]; // on-off array of blocks

// For both
boolean stopAnimate = 0; // this is set to 1 when a collision is detected
boolean mute = 0;
boolean newHigh = 0;
int score = 0; // score - this affects the difficulty of the game
int top = 0;

// Interrupt handlers
ISR(PCINT0_vect){ // PB0 pin button interrupt           
   if ( (player >0) && (isBreakout) ) {player--;} 
   return;
}
void playerIncUFO(){ // PB2 pin button interrupt
   fire = 1;
   fireCount = 5; // number of frames the shot will persist
}
void playerIncBreakOut(){ // PB2 pin button interrupt
  if (player <128-platformWidth){player++;}
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
  isBreakout = 0;
  
  if (digitalRead(2) == HIGH) isBreakout = 1;
  
  interrupts();
  ssd1306_init();
  ssd1306_fillscreen(0x00);
  
  for (byte s = 0; s<30; s++){ // generate stars
     ssd1306_setpos((uint8_t)(random(0,127)),(uint8_t)(random(0,7)) );
     ssd1306_send_data_start();
     ssd1306_send_byte(B10000000>>random(0,7)); 
     ssd1306_send_data_stop();
     //beep(10+s,600);    
  }

  if (isBreakout) {
    ssd1306_char_f6x8(16, 2, "B R E A K O U T");
  } else {
    ssd1306_char_f6x8(14, 2, "U F O  C H A O S");
  }
  // The lower case character set is seriously compromised because I've had to truncate the ASCII table
  // to release space for executable code - hence lower case y and w are remapped to h and / respectively.
  // There is no z in the table (or h!) as these aren't used anywhere in the text here and most of the 
  // symbols are also missing for the same reason (see my hacked version of font6x8.h - font6x8AJ.h for more detail)
  ssd1306_char_f6x8(3, 4, "mods bh andh jackson"); // see comments above !
  ssd1306_char_f6x8(22, 6, "game design bh");  // see comments above !
  ssd1306_char_f6x8(22, 7, "/ebboggles.com");  // see comments above !

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
        ssd1306_char_f6x8(8, 1, "-HIGH SCORE RESET-");  
      } else if (mute == 0) { mute = 1; ssd1306_char_f6x8(32, 1, "-- MUTE --"); } else { mute = 0; ssd1306_char_f6x8(23, 1, "-- SOUND ON --");  }    
      break;
    }
    if (sChange == 1) break;
  }  
  while(digitalRead(0) == HIGH);

  if (sChange == 0) {
    delay(2600);
    noInterrupts();  
    ssd1306_init();
    ssd1306_fillscreen(0x00);
    stopAnimate = 0;
    score = 0;
    
    if (isBreakout) { playBreakout(); top=topScoreB;} else { playUFO(); top=topScoreU;}
    
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
  noInterrupts();  
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
  fire = 0;
  maxObstacles = 3;
  obstacleStep = 2;
  for (byte i = 0; i<9; i++){
    obstacle[i] = -50;
    gapOffset[i]=0;
  }
  stepsSinceLastObstacle = 0;
  playerOffset = 0; // y offset of the top of the player
  
  while (stopAnimate == 0) {

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
  interrupts();
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
 *  Breakout Code
 */
void playBreakout() {

  lastFrame = millis();

  for (byte i =0; i<16;i++){ // reset blocks
    row[0][i]=1; row[1][i]=1; row[2][i]=1;
  } 
  
  platformWidth = 16;
  ballx = 64;
  bally = 50;
  hdir = -1;
  vdir = -1;
  score = 0;
  player = random(0,128-platformWidth);
  ballx = player+platformWidth/2;
  attachInterrupt(0,playerIncBreakOut,CHANGE);

  while (stopAnimate == 0) {
   delay(40);

    while (1==1) {
                // continue moving after the interrupt
                if (digitalRead(2)==1){if (player <128-platformWidth){player++;} if (player <128-platformWidth){player++;} if (player <128-platformWidth){player++;}}
                if (digitalRead(0)==1){if (player >0){player--;} if (player >0){player--;} if (player >0){player--;}}
                
                // bounce off the sides of the screen
                if ((bally+vdir<54&&vdir==1)||(bally-vdir>1&&vdir==-1)){bally+=vdir;}else {vdir = vdir*-1;}
                if ((ballx+hdir<127&&hdir==1)||(ballx-hdir>1&&hdir==-1)){ballx+=hdir;}else {hdir = hdir*-1;}
                
                // frame actions
                if (lastFrame+10<millis()){
                  if(bally>10&&bally+vdir>=54&&(ballx<player||ballx>player+platformWidth)){ // game over if the ball misses the platform
   
                    stopAnimate = 1;
                    break;
                  }else if (ballx<player+platformWidth/2&&bally>10&&bally+vdir>=54){ // if the ball hits left of the platform bounce left
                    hdir=-1; beep(20,600);
                  }else if (ballx>player+platformWidth/2&&bally>10&&bally+vdir>=54){  // if the ball hits right of the platform bounce right
                    hdir=1; beep(20,600);
                  }else if (bally+vdir>=54){
                    hdir=1; beep(20,600);
                  }
                  
                  collisionCheck: // go back to here if a collision was detected to prevent flying through a rigid
                  if (floor((bally+vdir)/8)==2){
                    if (row[2][ballx/8]==1){row[2][ballx/8]=0; score++;  
                        collision(); goto collisionCheck; // check collision for the new direction to prevent flying through a rigid
                    }
                  }else if (floor((bally+vdir)/8)==1){
                    if (row[1][ballx/8]==1){row[1][ballx/8]=0; score++; 
                        collision(); goto collisionCheck;
                    }
                  }else if (floor((bally+vdir)/8)==0){
                    if (row[0][ballx/8]==1){row[0][ballx/8]=0; score++;
                        collision(); goto collisionCheck;
                    }
                  }
                  
                  // reset blocks if all have been hit
                  if (score%48==0){ 
                    for (byte i =0; i<16;i++){
                     row[0][i]=1; row[1][i]=1; row[2][i]=1;
                    } 
                  }
                }
                 
                // update whats on the screen
                noInterrupts();
                
                // blocks
                for (int inc = 0; inc <3; inc ++) {
                  ssd1306_setpos(0,inc);
                  ssd1306_send_data_start();
                  for (int bl = 0; bl <16; bl++){
                    if(row[inc][bl]==1){
                      sendBlock(1);
                    }else {
                      sendBlock(0);
                    }
                   }   
                  ssd1306_send_data_stop();
                }
                
                // clear area below the blocks
                
                for (int kl = 3;kl <8;kl++) {
                  ssd1306_setpos(0,kl);
                  ssd1306_send_data_start();
                  for (byte i =0; i<128; i++){
                     ssd1306_send_byte(B00000000);
                  }
                  ssd1306_send_data_stop();
                }
                // draw ball
                ssd1306_setpos(ballx,bally/8);
                uint8_t temp = B00000001;
                ssd1306_send_data_start();
                temp = temp << bally%8+1;
                ssd1306_send_byte(temp);  
                ssd1306_send_data_stop();
                
                drawPlatform();
            interrupts();
    }
 }
  interrupts();

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

void collision(){ // the collsision check is actually done befor this is called, this code works out where the ball will bounce
  if ((bally+vdir)%8==7&&(ballx+hdir)%8==7){ // bottom right corner
      if (vdir==1){hdir=1;}else if(vdir==-1&&hdir==1){vdir=1;}else {hdir=1;vdir=1;}
    }else if ((bally+vdir)%8==7&&(ballx+hdir)%8==0){ // bottom left corner
      if (vdir==1){hdir=-1;}else if(vdir==-1&&hdir==-1){vdir=1;}else {hdir=-1;vdir=1;}
    }else if ((bally+vdir)%8==0&&(ballx+hdir)%8==0){ // top left corner
      if (vdir==-1){hdir=-1;}else if(vdir==1&&hdir==-1){vdir=-1;}else {hdir=-1;vdir=-1;}
    }else if ((bally+vdir)%8==0&&(ballx+hdir)%8==7){ // top right corner
      if (vdir==-1){hdir=1;}else if(vdir==1&&hdir==1){vdir=-1;}else {hdir=1;vdir=-1;}
    }else if ((bally+vdir)%8==7){ // bottom side
      vdir = 1;
    }else if ((bally+vdir)%8==0){ // top side
      vdir = -1;
    }else if ((ballx+hdir)%8==7){ // right side
      hdir = 1;
    }else if ((ballx+hdir)%8==0){ // left side
      hdir = -1;
    }else {
      hdir = hdir*-1; vdir = vdir*-1;
  }
  beep(30,300);
}

void drawPlatform(){
 noInterrupts();
 ssd1306_setpos(player,7);
 ssd1306_send_data_start();
 for (byte pw = 1; pw <platformWidth; pw++){ssd1306_send_byte(B00000011);}                
 ssd1306_send_data_stop();  
 interrupts(); 
}

void sendBlock(boolean fill){
  if (fill==1){
   ssd1306_send_byte(B00000000);
   for (int inc = 0; inc < 6; inc++) ssd1306_send_byte(B01111110);
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
