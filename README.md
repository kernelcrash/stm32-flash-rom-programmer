Simple Flash ROM programming rool using an stm32f407 board

 by kernel@kernelcrash.com

- Can erase and program parallel flash ROMs like the AM29F040B
- Based on my old hacked version of meeprommer
- The stm32f407 board appears as a USB serial device to a PC.
- You use a terminal program on a PC to talk to it.
- Wire a AM29F040B parallel flash ROM directly to the stm32f407 board.
- Simple command line based tool to query the flash ROM , and to
  program it using an xmodem transfer (128 byte mode)
- The Command line parser is very simple so when it says you need to type
  a 5 digit hex number, then type a 5 digit hex number. Press reset on the 
  stm32f407 board if you screw up.
- It probably only works on a AM29F040B or similar

Wiring
======

```
  PD8 - PD15    ->   D0 - D7                                                      
  PE0 - PE15    ->   A0 - A15                                                     
  PA2 - PA4    ->   A16 - A18                                                     
  PC0    ->   _CE                                                                 
  PC1    ->   _OE                                                                 
  PC2    ->   _WE      
  GND    ->   GND
  +5V    ->  VCC
```
Commands

 Read from memory
 ================
 r nnnnn mmmmm - show mmmmm bytes at address nnnnn

 eg.  r 00000 00020

      00000  11 00 20 1b 7a b3 20 fb db 99 21 b7 00 0e 80 7e                          
      00010  d3 99 00 79 d3 99 00 23 0c 79 fe 88 20 f1 21 00                          

 Command parsing is terrible, so it really is 'r' 'space' then
 an exactly five digit hex number, then a 'space' then another
 five digit hex number.

 Erase chip
 ==========
 eg. e

     erasing chip
     Erase Complete

 This does a full chip erase (ie. the whole chip goes to FF).  If you don't see
 'Erase Complete' within a minute, it's probably not working, and you may as well
 reset the stm32f407 board.

 Write to chip using an xmodem transfer
 ======================================

 eg.   w 00000

 Use an xmodem transfer to write to the chip at the address specified.
 As soon as you hit ENTER after the 'w' command you'll see a series of 
 'C' chars. You now need to switch your terminal program into 'Send xmodem'
 mode. In minicom I hit 'ctrl-a s', then choose XModem from the menu, then 
 it pops up a file requester. I find my file with the cursor keys, press
 space to select it and hit ENTER. The xmodem transfer should start and 
 the chip will be programmed as it's transferred. It's not that quick,
 but it works well enough.


 Identify chip
 =============

 eg.    i

Identify FLASH chip                                                             
Manufacturer: 20 , Device: e2  

Manufacturer 20 and device e2 is probably not a real AM29F040B. It should
show 01 ... A4 if its a real one.

 MD5SUM
 ======

 eg.   m 00000 08000

 Reads the flash chip and does an md5sum on the bytes read. In the example above
 it reads from 00000 for 32K of data, then prints out the md5. This is so you can
 double check whether what is written to the flash chip matches the file you sent.

 HELP
 ====

```
Flash ROM tool
==============
  r nnnnn mmmmm - show mmmmm bytes at address nnnnn
  w nnnnn - write to flash using xmodem transfer
  e - erase flash rom
  g - show how to connect the GPIO pins to the flash ROM
  i - identify flash rom
  m nnnnn mmmmm - md5sum rom content starting at nnnnn for mmmmmm bytes long
  v - version info
  ? - show this help screen
```
 
