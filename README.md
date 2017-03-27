Games for the AttinyArcade (or hardware inspired by this system)
================================================================

By Andy Jackson - Twitter: @andyhighnumber
------------------------------------------

The games in this folder can mostly be played either on the AttinyArcade, or similar hardware (if the hardware is different, there will be a circuit diagram in the same folder as the game).

See www.webboggles.com for details of the AttinyArcade hardware.

These are based on the Attiny85 device running at 8Mhz on its internal clock (don't forget to burn the bootloader when you first flash a new chip- the option to do this is in the "Tools" menu on the Arduino IDE.

In this folder:

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

For the standard Attiny Arcade Hardware, or enhanced functionality on custom hardware
=====================================================================================
- SpaceAttackAttiny: Space Invaders clone for the original Attiny (also allows for modified version of hardware with fire button - see circuit diagram in folder)
