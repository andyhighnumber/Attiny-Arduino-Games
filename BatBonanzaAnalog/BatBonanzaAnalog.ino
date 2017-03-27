/* 2015 / 2016 /2017
 * Pong clone by Andy Jackson - Twitter @andyhighnumber
 * Inspired by http://webboggles.com/ and includes some code from the #AttinyArcade games on that site
 * The code that does not fall under the licenses of sources listed below can be used non commercially with attribution.
 * 
 * This code takes its inputs from two paddles - one on pin 7 (the centre of a 10k linear pot between the power rails), the other
 * on the reset pin - this time with a 6k8 resistor between the negative leg of the pot and ground (to stop it resetting the ATTINY85!)
 * You should be able to find the circuit diagram from the folder where these files are (if you got them from my Google Drive) otherwise
 * Tweet @andyhighnumber and I will direct you to the circuit.
 * 
 * There's one button in this design (on pin 5). When the game is running, pressing and releasing the button cycles through modes, including 
 * two-player games and one-player modes with varying degrees of difficulty. 
 * 
 * Also, from standby....
 *  Press and hold the button to reset all settings (good to do when you first flash the chip, since the settings are loaded from EEPROM)
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
#define SSD1306_SCL   PORTB4  // SCL, Pin 4 on SSD1306 Board - for webbogles board
#define SSD1306_SDA   PORTB3  // SDA, Pin 3 on SSD1306 Board - for webbogles board
#define SSD1306_SA    0x78  // Slave address

#define WINSCORE 7

// Function prototypes
void startGame(void);
void drawPlatform(void);
void drawPlatform2(void);
void sendBlock(int);
void playPong(void);
void beep(int,int);
void drawBall(int x, int y);
void blankBall(int x, int y);

void doDrawLS(long, byte);
void doDrawRS(long, byte);
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

int topScoreB = 0;
int player; //0 to 128-platformWidth  - this is the position of the player
int player2; //0 to 128-platformWidth  - this is the position of the player
int lastPlayer;
int lastPlayer2;
int platformWidth = 16; 
boolean stopAnimate = 0; // this is set to 1 when a collision is detected
boolean mute = 0;
int score = 0; // score - this affects the difficulty of the game
int score2 = 0; // score - this affects the difficulty of the game

int ballx = 62*8; // coordinate of the ball
int bally = 50*4; // coordinate of the ball
int vdir = -4; // vertical direction and step  distance
int hdir = -8; // horizontal direction and step distance

int mode = 0;

int perturbation = 0;
int pFactor = 12;

// Interrupt handlers
ISR(PCINT0_vect){ // PB0 pin button interrupt           
}

void playerIncPong(){ // PB2 pin button interrupt
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
  // to release space for executable code - hence lower case y and w are remapped to h and / respectively.
  // There is no z in the table (or h!) as these aren't used anywhere in the text here and most of the 
  // symbols are also missing for the same reason (see my hacked version of font6x8.h - font6x8AJ.h for more detail)
  ssd1306_char_f6x8(0, 1, "   ---------------  ");
  ssd1306_char_f6x8(0, 2, "        B A T       ");
  ssd1306_char_f6x8(0, 4, "    B O N A N Z A   ");
  ssd1306_char_f6x8(0, 5, "   ---------------  ");
  ssd1306_char_f6x8(0, 7, "   bh andh jackson  "); // see comments above !

   long startT = millis();
    long nowT =0;
    boolean sChange = 0;
    while(digitalRead(0) == HIGH) {
      nowT = millis();
      if (nowT - startT > 2000) {
        sChange = 1;     
        EEPROM.write(0,0);
        EEPROM.write(1,0);
        ssd1306_char_f6x8(16, 0, "- SYSTEM RESET -");  
        break;
      }
      if (sChange == 1) break;
    }  
    while(digitalRead(0) == HIGH);

    mute=EEPROM.read(0);
    mode=EEPROM.read(1);

    if (mute != 0 && mute != 1) {
      mute = 0;
       EEPROM.write(0,0);
    }

    if (mode < 0 || mode > 5) {
      mode = 0;
      EEPROM.write(1,0);
    }

    if (sChange != 1) {
    delay(1500);
    ssd1306_init();
    ssd1306_fillscreen(0x00);
    stopAnimate = 0;
    score = 0;
    score2 = 0;

    playPong(); 
    
    delay(3500);
 
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
    if (c>40) c=c-6;
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
 *  Pong Code
 */
void playPong() {
  ballx = 64*8;
  bally = 32*4;
  hdir = -8;
  vdir = -4;
  int actualy, actualx;
  int factor = 0;
  int waitCount = 0;

  int lastx=64*8, lasty=32*4;

  player=64;
  player2=64;
  lastPlayer = 64;
  lastPlayer2 = 64;
  score = 0; // obvious
  score2 = 0; // obvious
  perturbation = 0;
  
  startGame();
  while (stopAnimate == 0) {
    while(1) {
    waitCount++;
    
    if(digitalRead(0)==1) {
      boolean sChange = 0;
      long startT = millis();
      long nowT =0;
      while(digitalRead(0) == HIGH) {
        nowT = millis();
        if (nowT - startT > 1500) {
          sChange = 1;
          if (mute == 0) { mute = 1; ssd1306_char_f6x8(32, 0, "-- MUTE --"); } else { mute = 0; ssd1306_char_f6x8(23, 0, "-- SOUND ON --"); }
          break;
        }
      }
      while(digitalRead(0) == HIGH);
      if (sChange == 1) {
      } else if (mode == 0) { 
          mode = 1; 
          ssd1306_char_f6x8(26, 0, "-- EXPERT --");  
          } else if (mode == 1) { 
            mode = 2; 
            pFactor = 11;
            ssd1306_char_f6x8(32, 0, "-- AUTO --"); 
          } else if (mode == 2) { 
            mode = 3; 
            pFactor = 11;
            ssd1306_char_f6x8(20, 0, "- TOUGH AUTO -");  
          } else if (mode == 3) { 
            mode = 4; 
            pFactor = 10;
            ssd1306_char_f6x8(16, 0, "- EXPERT AUTO -");  
          } else if (mode == 4) { 
            mode = 0; 
            ssd1306_char_f6x8(26, 0, "-- NORMAL --");  
          }
      
      if (sChange == 0) delay(1000);
      ssd1306_fillscreen(0x00);        
      EEPROM.write(0,mute);
      EEPROM.write(1,mode);
    }

    player = ((analogRead(0)-560)/7);
        if (player > 48) player = 48;
    if (player <0) player = 0;


    player2 = (analogRead(1) / 16);
    if (mode == 2 || mode == 3 || mode == 4) {
      if(waitCount >= 3) {
        waitCount = 0;
        perturbation = perturbation - 2 + random(0,5);
        if (perturbation > pFactor) perturbation = pFactor - 2;
        if (perturbation < pFactor*-1) perturbation = (pFactor*-1)+2;
      }
      player2 = (bally/4 -8)+perturbation;
    }
    if (player2 > 48) player2 = 48;
    if (player2 <0) player2 = 0;

    actualy = floor(bally/4);
    actualx = floor(ballx/8);
    
    // bounce off the sides of the screen
    if ((actualy+vdir<63&&vdir>01) || (actualy- vdir>6&&vdir<0)){
      bally+=vdir;
    }else {
        vdir = vdir*-1;
    }
    ballx+=hdir;

    actualy = floor(bally/4);
    actualx = floor(ballx/8);
    
    // check it hits the left pad and deal with bounces and misses
    if (actualx <= 4) {
      if(actualy<player-1||actualy>player+platformWidth+1){ 
        score2++;
    
        ballx = 5*8;
        bally = player*4;

        hdir = 13;
        if (vdir > 0) {
          vdir = 2;
        } else vdir = -2;
        
        ssd1306_fillscreen(0x00);        
        doNumber(46,4,score);
        doNumber(78,4,score2);        
        if (score2 < WINSCORE) {
          for (int i = 0; i<1000; i = i+ 100){
            beep(50,i);
          }
          for (int incr=0;incr<3;incr++) {
              ssd1306_send_data_stop();
              ssd1306_setpos(78,4);
              ssd1306_send_data_start();
              sendBlock(0);
              sendBlock(0);
              ssd1306_send_data_stop();
              delay(350);
              doNumber(78,4,score2);
              delay(350);
            }
            startGame();
        }
        perturbation = 0;
        break;
      }else if (actualy<player+1){        
        vdir = -6;
        hdir = 7;
      }else if (actualy<player+4){        
        vdir = -4;
        hdir = 10;
      }else if (actualy<player+7){        
        vdir = -2;
        hdir = 13;
      }else if (actualy<player+9){        
        vdir = 0;
        hdir = 14;
      }else if (actualy<player+12){        
        vdir = 2;
        hdir = 13;
      }else if (actualy<player+15){        
        vdir = 4;
        hdir = 10;
      }else {   
        vdir = 6;
        hdir = 7;
      }
      beep(20,600);
    }  
    
    // check it hits the right pad and deal with bounces
    if(actualx >= 122) {
      if(actualy<player2-1||actualy>player2+platformWidth+1){
        score++;
  
        ballx = 120*8;
        bally = player2*4;

        hdir = -13;
        if (vdir > 0) {
          vdir = 2;
        } else vdir = -2;

        ssd1306_fillscreen(0x00);        
        doNumber(46,4,score);
        doNumber(78,4,score2);
        if (score < WINSCORE) {
        for (int i = 0; i<1000; i = i+ 100){
            beep(50,i);
          }                
  
          for (int incr=0;incr<3;incr++) {
              ssd1306_setpos(46,4);
              ssd1306_send_data_start();
              sendBlock(0);
              sendBlock(0);
              ssd1306_send_data_stop();
              delay(350);
              doNumber(46,4,score);
              delay(350);
            }
            perturbation = 0;
            startGame();
        }        
        break;
      }else if (actualy<player2+1){        
        vdir = -6;
        hdir = -7;
      }else if (actualy<player2+4){        
        vdir = -4;
        hdir = -10;
      }else if (actualy<player2+7){        
        vdir = -2;
        hdir = -13;
      }else if (actualy<player2+9){        
        vdir = 0;
        hdir = -14;
      }else if (actualy<player2+12){        
        vdir = 2;
        hdir = -13;
      }else if (actualy<player2+15){        
        vdir = 4;
        hdir = -10;
      }else {   
        vdir = 6;
        hdir = -7;
      }
      beep(20,300);
    }      

    
    if (mode == 1 || mode == 3 || mode == 4) {
      factor = 8-floor((score-score2)/2); // expert modes
      if (factor < 2) factor = 2;
    } else {
      factor = 20-floor((score-score2)/2); // normal modes
      if (factor < 10) factor = 10;
    }
    
    delay(factor);
    
    // draw ball
    blankBall(floor(lastx/8),floor(lasty/4));  
    drawPlatform();
    drawPlatform2();
    drawBall(floor(ballx/8),floor(bally/4));
    lastx = ballx;
    lasty = bally;
    
    doNumber(28,0,score);
    doNumber(92,0,score2);
    if (score == WINSCORE || score2 == WINSCORE) {
      stopAnimate = 1;
      break;
    }
    }
 }

blankBall(floor(lastx/8),floor(lasty/4));  
blankBall(floor(ballx/8),floor(bally/4));  

if (score > score2) {
  ssd1306_char_f6x8(27, 3, "P L A Y E R 1"); 
} else {
  ssd1306_char_f6x8(27, 3, "P L A Y E R 2"); 
}
ssd1306_char_f6x8(27, 4, "             ");
ssd1306_char_f6x8(27, 5, "   W I N S   ");

for (int i = 0; i<1000; i = i+ 50){
  beep(50,i);
}


for (int incr=0;incr<6;incr++) {
    ssd1306_setpos(28,0);
    ssd1306_send_data_start();
    sendBlock(0);
    sendBlock(0);
    ssd1306_send_data_stop();
    ssd1306_setpos(92,0);
    ssd1306_send_data_start();
    sendBlock(0);
    sendBlock(0);
    ssd1306_send_data_stop();
    delay(350);
    doNumber(28,0,score);
    doNumber(92,0,score2);
    delay(350);
  }
}

void drawPlatform() {
  if (player != lastPlayer) {
    ssd1306_setpos(0,lastPlayer/8);
    ssd1306_send_data_start();
    ssd1306_send_byte(B00000000);
    ssd1306_send_data_stop();
    ssd1306_setpos(0,lastPlayer/8+1);
    ssd1306_send_data_start();
    ssd1306_send_byte(B00000000);
    ssd1306_send_data_stop(); 
    ssd1306_setpos(0,lastPlayer/8+2);
    ssd1306_send_data_start();
    ssd1306_send_byte(B00000000);
    ssd1306_send_data_stop(); 
  }  

  if (player%8!=0){
    ssd1306_setpos(0,player/8);
    ssd1306_send_data_start();
    ssd1306_send_byte((B11111111)<<player%8);
    ssd1306_send_data_stop();
    ssd1306_setpos(0,player/8+1);
    ssd1306_send_data_start();
    ssd1306_send_byte(B11111111);
    ssd1306_send_data_stop();    
    ssd1306_setpos(0,player/8+2);
    ssd1306_send_data_start();
    ssd1306_send_byte((B01111110)>>8-player%8);
    ssd1306_send_data_stop();
  } else {
    ssd1306_setpos(0,player/8);
    ssd1306_send_data_start();
    ssd1306_send_byte(B11111111);
    ssd1306_send_data_stop();
    ssd1306_setpos(0,player/8+1);
    ssd1306_send_data_start();
    ssd1306_send_byte(B11111111);
    ssd1306_send_data_stop();

  }
  lastPlayer = player;
}

void drawPlatform2() {

  if (player2 != lastPlayer2) {
    ssd1306_setpos(127,lastPlayer2/8);
    ssd1306_send_data_start();
    ssd1306_send_byte(B00000000);
    ssd1306_send_data_stop();
    ssd1306_setpos(127,lastPlayer2/8+1);
    ssd1306_send_data_start();
    ssd1306_send_byte(B00000000);
    ssd1306_send_data_stop(); 
    ssd1306_setpos(127,lastPlayer2/8+2);
    ssd1306_send_data_start();
    ssd1306_send_byte(B00000000);
    ssd1306_send_data_stop(); 
  }
  
  if (player2%8!=0){
    ssd1306_setpos(127,player2/8);
    ssd1306_send_data_start();
    ssd1306_send_byte((B11111111)<<player2%8);
    ssd1306_send_data_stop();
    ssd1306_setpos(127,player2/8+1);
    ssd1306_send_data_start();
    ssd1306_send_byte(B11111111);
    ssd1306_send_data_stop();        
    ssd1306_setpos(127,player2/8+2);
    ssd1306_send_data_start();
    ssd1306_send_byte((B01111110)>>8-player2%8);
    ssd1306_send_data_stop();
  } else {
    ssd1306_setpos(127,player2/8);
    ssd1306_send_data_start();
    ssd1306_send_byte((B11111111)<<0);
    ssd1306_send_data_stop();
    ssd1306_setpos(127,player2/8+1);
    ssd1306_send_data_start();
    ssd1306_send_byte((B11111111)<<0);
    ssd1306_send_data_stop();
  }
  lastPlayer2 = player2;
}



void sendBlock(int fill){
  if (fill == 1) {
   ssd1306_send_byte(B10011000);
   ssd1306_send_byte(B01011100);
   ssd1306_send_byte(B10110110);
   ssd1306_send_byte(B01011111);
   ssd1306_send_byte(B01011111);
   ssd1306_send_byte(B10110110);
   ssd1306_send_byte(B01011100);
   ssd1306_send_byte(B10011000);
  } else  {
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

void blankBall(int x, int y) {
  if (y%8!=0){
    ssd1306_setpos(x,y/8);
    ssd1306_send_data_start();
    ssd1306_send_byte(B00000000);
    ssd1306_send_byte(B00000000);
    ssd1306_send_data_stop();
    
    ssd1306_setpos(x,y/8+1);
    ssd1306_send_data_start();
    ssd1306_send_byte(B00000000);
    ssd1306_send_byte(B00000000);
    ssd1306_send_data_stop();
  } else {
    ssd1306_setpos(x,y/8);
    ssd1306_send_data_start();
    ssd1306_send_byte(B00000000);
    ssd1306_send_byte(B00000000);
    ssd1306_send_data_stop();
  }
}


void drawBall(int x, int y) {
  if (y%8!=0){
    ssd1306_setpos(x,y/8);
    ssd1306_send_data_start();
    doDrawLS(0,y%8);
    ssd1306_send_data_stop();
    
    ssd1306_setpos(x,y/8+1);
    ssd1306_send_data_start();
    doDrawRS(0,8-y%8);
    ssd1306_send_data_stop();
  } else {
    ssd1306_setpos(x,y/8);
    ssd1306_send_data_start();
    doDrawLS(0,0);
    ssd1306_send_data_stop();
  }
}

// Drawing routine for the player and fire - with right-shifts
void doDrawRS(long P1, byte P2) {
  ssd1306_send_byte((B00000011 | P1)>>P2);
  ssd1306_send_byte((B00000011 | P1)>>P2);
}

// Drawing routine for the player and fire - with left-shifts
void doDrawLS(long P1, byte P2) {
  ssd1306_send_byte((B00000011 | P1)<<P2);
  ssd1306_send_byte((B00000011 | P1)<<P2);
}

void startGame(void) {
  
    ssd1306_fillscreen(0x00);

    ssd1306_char_f6x8(16, 3, "-- GET READY --");
    doNumber(60,5,3);
    delay(1000);
    doNumber(60,5,2);
    delay(1000);
    doNumber(60,5,1);
    delay(1000);
    ssd1306_fillscreen(0x00);

    for (int i = 800; i>200; i = i - 200){
    beep(30,i);
    }

}

