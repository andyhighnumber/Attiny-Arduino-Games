Games for the AttinyArcade (or hardware inspired by this system)
================================================================

By Andy Jackson - Twitter: @andyhighnumber
------------------------------------------

The games in this folder can mostly be played either on the AttinyArcade, or similar hardware (if the hardware is different, there will be a circuit diagram in the same folder as the game).

Info on programming the Attiny85 chip can be found online, here's an example: https://create.arduino.cc/projecthub/arjun/programming-attiny85-with-arduino-uno-afb829 - don't forget to burn the bootloader (it's a menu option under 'Tools' on the Arduino IDE) the first time you flash the chip and if you need to change between 8Mhz and 16Mhz clocks (all my games are 8Mhz, except Pacman which is 16Mhz).

See www.webboggles.com for details of the AttinyArcade hardware.

These are based on the Attiny85 device running at 8Mhz on its internal clock (don't forget to burn the bootloader when you first flash a new chip- the option to do this is in the "Tools" menu on the Arduino IDE).

In this folder:

For the standard Attiny Arcade Hardware, or enhanced functionality on custom hardware
=====================================================================================
- Pacman_Attiny_Arcade: PacMan clone for the original Attiny. This is the best of the lot in my opinion. Unlike the other games here, this needs to be flashed with a 16Mhz internal clock bootloader (all the others are 8MHz). Send me a message via twitter if this presents any problems.
- SpaceAttackAttiny: Space Invaders clone for the original Attiny (also allows for modified version of hardware with fire button - see circuit diagram in folder)
- Frogger_Attiny_Arcade: Frogger clone for the original Attiny (also allows modified version of hardware with jump button - same hardware config as SpaceAttack above - see schematic in folder)
- MorseAttinyArcade: A morse code decoder (for practicing morse, if you so wish), which displays inputted code via a 5-line scrolling display. When combined with a PLL (or similar) tone detection circuit, this could easily decode off-air morse.

For the standard Attiny Arcade Hardware
=======================================
- WrenRollercoasterAttinyArcade: Inspired by the TinyWings iOS game - fly a wren along a landscape as far as you can! 
- Attiny Tetris: A Tetris clone, evolved from a game for the Arduino by Anthony Russell (https://github.com/AJRussell/Tiny-Tetris)
- UFO_Stacker_Attiny: Two games in one sketch - UFO (from www.webboggles.com) and Stacker (from me).
- UFO_Breakout_Arduino: Again - two games in one sketch, both from www.webboggles.com, UFO and Breakout
- BatBonanzaAttinyArcade: A version of Pong for the standard AttinyHardware - not as playable because you really need analog control for this type of game - see below

For custom hardware - see circuit schematics in the folders with the sketches
=============================================================================
- BatBonanzaAnalog: Pong clone based on custom hardware with analog controls (will work on the same hardware as Space Attack Anlog Version).
- Space_Attack_Analog: Space Invaders clone (sort of) based on custom hardware with analog control (will work on the same hardware as Bat Bonanza Anlog Version).
- BatBonanzaAnalogSinglePot: Pong clone based on custom hardware with single-pot analog control (will work on the same hardware as Space Attack Anlog Version).
- Tetris_Multi_Button: A Tetris cline, as above but designed for multi-button pad (see circuit schematic in folder for details of the hardware)
