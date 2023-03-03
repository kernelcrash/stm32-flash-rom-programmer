Simple Flash ROM programming rool using an stm32f407 board
modified to only program a MX29F1615 flash ROM

 by kernel@kernelcrash.com

- Can erase and program a MX29F1615 flash ROM
- Based on my old hacked version of meeprommer
- The stm32f407 board appears as a USB serial device to a PC.
- You use a terminal program on a PC to talk to it.
- Wire a MX29F1615 parallel flash ROM directly to the stm32f407 board.
- You need a seperate power supply that can supply 10V.
- Use a jumper or toggle switch to apply the 10V to the _BYTE/VPP pin when writing
  or erasing
- Simple command line based tool to query the flash ROM , and to
  program it using an xmodem transfer (128 byte mode)
- The Command line parser is very simple. Press reset on the 
  stm32f407 board if you screw up.
- It only works on the MX29F1615

Weirdness
=========
Because this is done very cheaply it does not have the features of a proper MX29F1615 programming tool

- Identify doesn't work. Ultimately you need to apply 10V to the _BYTE/VPP pin when applying some magic
  command sequence to the chip, but you need to remove it just before you read the manufacturer and device
  id. Because I cannot do that programmatically, the identify command doesnt work
- Erase  works in that it erases the chip. But its hard to tell if it succeeded or not as the 
  chip is in whats called 'Read SR mode' or 'Read status register mode' after you do the erase command.
  That just means you cannot check if you actually have FF's everywhere until you power cycle.
- Writing works, but it can't properly check that it successfully wrote to parts of the chip. The way
  you are meant to do it is to write 64 x 16bit words to the chip with 10V applied, then check the status
  on the databus and then retry if necessary and so on. At the moment I just write the 64 words, then wait
  about 4ms and 'hope'. Worst case is that you have to write it again. At the end of the whole writing
  process you best off just leaving the 10V connected for a few minutes, then power cycle the flash ROM.
  Basically you wont be able to read back anything you have written until you power cycle.
- I have made this so its specifically for big endian CPUs like the 68000 in the Amiga. That means you
  can take a kickstart ROM image and not have to byte swap it.  I have a md5 function that basically can
  calculate the md5 of some data written to the flash ROM. I've opted to effectively pre-byte-swap it so
  that if you do a md5sum on a PC for a kickstart ROM it will match the one calculated by the tool.


Wiring
======

```

  MX29F1615

  PD0 - PD16    -> Q0 - Q15
  PE0 - PE15    -> A0 - A15
  PA2 - PA5     -> A16 - A19
  PC0           -> _CE
  PC1           -> _OE
  PC2           -> N.C
  GND           -> GND
  +5V           -> VCC
                   Put a toggle switch on _BYTE/VPP. One position is to +5V. The other position is to a 10V source (or maybe +12V then through three 1N4001 diodes
  
```

Using it
========

- Organise a +10V power supply. I have used a LM2596 adjustable DC-DC supply. They generally take +12V in and
  produce some other lower voltage. I just adjusted mine so that it produces +10V. 
- compile the source
- flash the build/stm32-flash-rom-programmer.hex  to an stm32f407 board (vet6 or vgt6 will do)
- test plugging the board in and see if it shows up as a general serial device (on linux it usually shows up as /dev/ttyACM0)
- unplug and wire it all up. I just put the MX29F1615 on a breadboard, then use arduino header wires to connect it up. 
  You can use the +5V on the stm32f407 board to run the +5V of the MX29F1615
- Take the GND from your +10V supply and connect it to the GND on the breadboard, but leave the +10V not
  connected to anything.
- Make sure the _BYTE/VPP pin is easy to get access to. I just run a small breadboard jumper to a blank part of the board. Then another jumper
  from there to the +5V rail.
- Plug it all in to a PC again and run a terminal program. I just use minicom and you should get a menu in your terminal program.
  Press ? a few times to repeat the menu
- You can try reading the MX29F1615 as a test. I usually do this

  r 00000 00100

  So all numbers are in hex. Note the  the 2nd 5 digit number is the length to read.
 
  If you bought an MX29F1615 from China , chances are it is 2nd hand and has some other data on it. So if you
  see a lot of different hex characters that's great. You might also see FFFF's which could be good or bad. Its
  good if your chip has arrived to you erased, but its bad if you're wiring is incorrect and you aren't actually reading anything

- Erase kinda works. Basically do this

  - change the voltage on the _BYTE/VPP pin to +10V 
  - Enter the 'e' command

  It probably is quite quick, but I would just leave it a few minutes as the erase stuff (and the programming stuff) actually
  runs in the background of the chip somehow.

  Don't try and use an 'r' command to read back from the chip. It will be wrong (will probably have 0x80 or similar)

  After waiting a few minutes, move the _BYTE_VPP pin back to +5V.
  Power cycle the MX29F1615 (or just unplug and replug the smt32f407 board if it
  is supplying +5V to the MX29F1615). Now try a 'r' command. eg

   r 00000 00400

 It should be FFFF's. If its not you could try erasing again. There is also a 'b'
 command that can check a long range for FFFF's.

- Write also works, but not like a 'proper' programmer. Here's what I do

  - If I am writing a 512K kickstart I just cut out the first few bytes as a test. eg. to cut out the first 1024bytes

    dd if=kick.rom of=test.rom bs=1 count=1024

 - I plug in the stm32f407 and MX29F1615 connected to it.
 - I double check I have erased it first
 - I am using minicom sudo'd to root so I just put my rom file in /root to make it easier for me to find it
 - Change the voltage on _BYTE/VPP to +10V
 - Enter this to start the writing

    w 00000

   In minicom I then have to go ctrl-a s , then pick xmodem from the list, and then find my test.rom file
   and select it (space in minicom) and the transfer should start. It should be quick since its a small file

   Wait a minute or so then power cycle the stm32f407 board and MX29F1615. Now do an 'r' command

   r 00000 00400

   Try visually seeing if it looks about right. Also use the md5sum command on your PC to first do an md5sum
   on the PC side, then in the terminal

   m 00000 00400

   Hopefully the md5 sums match.

 - Now try flashing the whole 512K rom. Hook up +10V to the _BYTE/VPP pin again and issue the 'w' command

   w 00000

   I will just note you don't need to byte swap. And make sure you start from 00000 again.
   It will just flash over the first 1024 bytes with exactly the same data so its all good. After the whole
   512K is done, again leave it for a few minutes then switch _BYTE/VPP back to +5V and
   power cycle it.

   Do an r command to confirm you can read it back, then try a md5sum of the whole rom. For a 512K rom
   you would enter

   m 00000 80000

   Check the md5sum on your PC against the full file. Hopefully they match. If they don't, just try
   writing the same data again.

~~~~~~~~~~~~~~~~~~~~~~

Commands

 Read from memory
 ================
 r nnnnn mmmmm - show mmmmm bytes at address nnnnn

 eg.  r 00000 00020

    % r 00000 00020
    00000  1114 4ef9 00f8 00d2 0000 ffff 0028 003f 
    00010  0028 000a ffff ffff 0041 4d49 4741 2052 



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

This does not work for the MX29F1615

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
 
