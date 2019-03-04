## ATtiny Tetris Gold

This code is designed to run on (very simple and cheap to build) custom hardware and you should find a schematic for the circuit in the folder ATtiny-Tetris-Gold. Because the voltages of the switches are important, it's good to insulate the back of the board from fingers if you can (I use hot melt glue but other methods would work - ideally put it in a case!)
   
Designed originally for the Attiny85 and optimised for the *#AttinyArcade* platform. The source code is less than 8KB and the sketch should run happily with less than 300 bytes of RAM. You can find out more about this platform from http://webboggles.com, buy kits to make it (or get instructions / schematics). This sketch includes some code from the #AttinyArcade games on that site, including interrupt code. 

This game started life as a port but is now essentially a clone of TinyTetris by Anthony Russell, with some additional features. There remain elements of that original codebase, although the vast majority of what's here has been rewritten from scratch (including the screen, text and number rendering code and much of the game engine) in order to optimise for memory, improve responsiveness and allow new features on the limited hardware (added features include Highscore (saved to EEPROM), optional Ghost (or Shadow) Piece, Interrupt Handling and Hard-Mode functionality).  
   
Anthony's source can be found here: https://github.com/AJRussell/Tiny-Tetris and is highly recommended if you'd like a version of Tetris to run on normal Arduino hardware. It has some lovely graphics by Tobozo (one image from which is now inclded the opening screen here!) and it's also possible that some code by Tobozo has made it into this version. Tobozo's repository can be found here; https://github.com/tobozo and is well worth a look. 
There is an Instructables page relating to this project here: https://www.instructables.com/id/Tetris-Clone-With-OLED-SSD1306I2C-for-Arduino-Nano/ 
        
This sketch is using the screen control and font functions written by Neven Boyanov for the http://tinusaur.wordpress.com/ project
Source code and font files available at: https://bitbucket.org/tinusaur/ssd1306xled - hacked about by Andy Jackson to make them render side-on for this game. All the necessary functions are in this file, there's no need to download any additional libraries to compile this game.

The sleep code in this file is based on this blog post by Matthew Little:
http://www.re-innovation.co.uk/web12/index.php/en/blog-75/306-sleep-modes-on-attiny85

### Tetris Multi Button

This interrupt routine relies on having three buttons wired to pin 7 of the Attiny85 - all with a single pull-down resistor (10k) and 
*  One that pulls-up to +Vcc (giving an analog reading of 1023)
*  One that pulls-up via a 1k resistor (analog reading somewhere around 930ish)
*  One that pulls-up via a 2.2 resistor (analog reading around 840)

The key point is that all three buttons provide a voltage that sends the pin to logic-high, thus triggering the interrupt (therefore you could add more buttons, as long as the voltage is always above the threshold that the Attiny85 would class as logic-high). 

In practice, this technique requires a reasonably well made and well insulated board - adding moisture to to board (via damp hands for example) would likely throw the voltages out enough that this technique would become unreliable.

### Before the game starts:
* Hold "DROP" BUTTON - Turns 'ghost' piece on and off
* Hold "DROP" and "ROTATE" together - Turns 'challenge mode' on and off (which fills the screens with random stuff at the start of the game to make it more tricky!)

*ATtiny Tetris Gold Edition* ;) is developed by Jarosław Mazurkiewicz [jm.iq.pl](https://jm.iq.pl/tetris).
The previous version was developed by [Andy Jackson](https://github.com/andyhighnumber/Attiny-Arduino-Games).

The code that does not fall under the licenses of sources listed below can be used non-commercially with or without attribution.
This software is supplied without warranty of any kind.

--

Info on programming the Attiny85 chip can be found online, here's an example: https://create.arduino.cc/projecthub/arjun/programming-attiny85-with-arduino-uno-afb829 - don't forget to burn the bootloader (it's a menu option under 'Tools' on the Arduino IDE) the first time you flash the chip and if you need to change to 8Mhz clock.

For more information please visit: https://jm.iq.pl/tetris
