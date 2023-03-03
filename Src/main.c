/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "md5stuff.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VERSIONSTRING "AM29F040 READWRITE (based on MEEPROMMER), CMD:a,w,r,i,e,m,v, version 0.10"

#define MAX_STRING_BUFFER_LENGTH 160

#define SPECIAL_FLASH_ROM_ADDRESS_1 0x5555
#define SPECIAL_FLASH_ROM_ADDRESS_2 0x2aaa

#define NOCOMMAND    0
#define VERSION      1
#define SET_ADDRESS  2
#define CLEAR_RAM    3

#define READ_HEX    10
#define READ_BIN    11
#define READ_ITL    12

#define WRITE_HEX   20

#define WRITE_BIN   21
#define WRITE_ITL   22
#define WRITE_XMODEM 23

#define FLASH_IDENTIFY 30
#define FLASH_ERASE 31

#define MD5SUM 40

#define BLANK_CHECK 45

#define HELP 50
#define GPIO_HELP 51

#define POSITION_SOH  0
#define POSITION_PACKET 1
#define POSITION_PACKETSLEFT 2
#define POSITION_PAYLOAD 3
#define POSITION_CRC 131
#define POSITION_CRC2 132

#define EOT 0x04


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t startAddress, endAddress;
uint32_t lineLength, dataLength;
int loop_cmd = 0; // set to >0 for prompt for command mode
int position; // ptr in xmodem packet
int xmodem_prestart;
int available;
int bytes;  // Make sure this is not defined in 'loop'!
uint8_t crchigh,crclow;
uint32_t total;
uint32_t flashaddress;
uint32_t address_toggle;
uint8_t page_buffer[128];

//a buffer for bytes to burn
#define BUFFERSIZE 133
uint8_t buffer[BUFFERSIZE];
//command buffer for parsing commands
#define COMMANDSIZE 32
char cmdbuf[COMMANDSIZE];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay_us(const uint32_t us)
{
   uint32_t i = us * 60;
   while (i-- > 0) {
      __asm volatile ("nop");
   }
}

void printSerialUSB(char *s) {
	uint32_t i,j;
	uint8_t buffer[MAX_STRING_BUFFER_LENGTH];

	j=0;
	for (i=0;i<MAX_STRING_BUFFER_LENGTH;i++) {
		buffer[i]= (uint8_t) s[j];
		if (buffer[i]==0) {
			break;
		}
		j++;
	}

	usb_write( buffer,i);
}

void printLineSerialUSB(char *s) {
	char buffer[MAX_STRING_BUFFER_LENGTH];
	sprintf(buffer,"%s\r\n",s);
	printSerialUSB(buffer);
}



void printPrompt() {
	printSerialUSB("\r\n> ");
}


void printHelp() {
  printLineSerialUSB("\r\nFlash ROM tool (MX29F1615 version)");
  printLineSerialUSB("=====================================");
  printLineSerialUSB("  r nnnnn mmmmm - show mmmmm bytes at address nnnnn");
  printLineSerialUSB("  w nnnnn - write to flash using xmodem transfer");
  printLineSerialUSB("  e - erase flash rom");
  printLineSerialUSB("  g - show how to connect the GPIO pins to the flash ROM");
  printLineSerialUSB("  i - identify flash rom");
  printLineSerialUSB("  m nnnnn mmmmm - md5sum rom content starting at nnnnn for mmmmm bytes long");
  printLineSerialUSB("  b nnnnn mmmmm - Blank check for FFFFs starting at nnnnn for mmmmm bytes long");
  printLineSerialUSB("  v - version info");
  printLineSerialUSB("  ? - show this help screen");
  printSerialUSB("\r\n");
}

void printGPIOHelp() {
  printSerialUSB("\r\nGPIO Map\r\n");
  printSerialUSB("==============\r\n");
  printSerialUSB("PD0 - PD15    ->   Q0 - Q15\r\n");
  printSerialUSB("PE0 - PE15    ->   A0 - A15\r\n");
  printSerialUSB("PA2 - PA5    ->   A16 - A19\r\n");
  printSerialUSB("PC0    ->   _CE\r\n");
  printSerialUSB("PC1    ->   _OE\r\n");
  printSerialUSB("\r\n");
}

// addr is word address, not a byte address
void writeToFlashROM(uint32_t addr, uint16_t data) {
	uint32_t upper_addr;
	uint32_t t;

	upper_addr = (addr & 0x000f0000) >>16;

	GPIOE->ODR = (addr & 0x0000ffff);
	// add the upper address bits to PA2, PA3, PA4, PA5
	t = GPIOA->IDR;
	GPIOA->ODR = (t & 0xffffffc3) | (upper_addr << 2);
	GPIOD->MODER = 0x55555555;	// port D to outputs

	GPIOD->ODR = data ;

	GPIOC->ODR &= 0xfffffffe;	// PC0 (_CE) low
	delay_us(2);

	GPIOC->ODR |= 0x00000007;	// PC0 (_CE) high

}

// addr is word address, not a byte address
uint16_t readFromFlashROM(uint32_t addr) {
	uint32_t upper_addr;
	uint32_t t;
	uint16_t d;

	upper_addr = (addr & 0x000f0000) >>16;

	GPIOE->ODR = (addr & 0x0000ffff);
	// add the upper address bits to PA2, PA3, PA4
	t = GPIOA->IDR;
	GPIOA->ODR = (t & 0xffffffc3) | (upper_addr << 2);
	GPIOD->MODER = 0x0;	// port D to inputs

	GPIOC->ODR &= 0xfffffffc;	// PC0 (_CE) and PC1 (_OE) low
	delay_us(2);

	d = GPIOD->IDR ;

	GPIOC->ODR |= 0x00000007;	// PC0 (_CE) and PC1 (_OE) high

	return d;


}

uint8_t write_next_word( uint16_t data) {
   int i;
   uint16_t tmp;

   writeToFlashROM(SPECIAL_FLASH_ROM_ADDRESS_1,0x00aa);
   writeToFlashROM(SPECIAL_FLASH_ROM_ADDRESS_2,0x0055);
   writeToFlashROM(SPECIAL_FLASH_ROM_ADDRESS_1,0x00a0);

   // write the word
   writeToFlashROM(flashaddress,data);

   // wait for DQ7 to be set correctly
   for (i=0;i<100;i++) {
      tmp = readFromFlashROM(flashaddress>>1);
      if (tmp & 0x0080)
         break;
   }

   flashaddress+=2;

   return 0; // TODO . return an error if it failed
}

uint8_t write_next_page( uint8_t *buf) {
   int i;
   uint16_t tmp;

   writeToFlashROM(SPECIAL_FLASH_ROM_ADDRESS_1,0x00aa);
   writeToFlashROM(SPECIAL_FLASH_ROM_ADDRESS_2,0x0055);
   writeToFlashROM(SPECIAL_FLASH_ROM_ADDRESS_1,0x00a0);

   // write 64 words
   for (i=0;i<64;i++) {
	// TODO this should probably be the other way around
	// Put the lower address byte in D15 to D8 and the higher address byte in D7 to D0. This means we should not have to byte swap
	tmp = ( (uint16_t) (buf[i*2]<<8) + (uint16_t) (buf[i*2+1]) );
   	writeToFlashROM(flashaddress>>1,tmp);
	flashaddress+=2;
   }
   // pray and hope since we cannot set _BYTE_VPP back to +5V
   delay_us(4000);


   return 0; // TODO . return an error if it failed
}

// Since we only write 16 bytes at a time, write_next_byte will save a byte then write a word  depending on whether flashaddress is even or odd. 
// This also means you can't really  have an uneven number of bytes to write.
//
// We should be writing 64 x 16bit words at a time. That is conveniently the 128 bytes of one block of the xmodem transfer
// So basically accumulate 64 words then write those all at once
uint8_t write_next_byte( uint8_t data) {
	if ((address_toggle & 0x0007f) != 0x0007f) {
		page_buffer[address_toggle & 0x0007f] = data;
		address_toggle++;
		return 0;
	} else {
		page_buffer[address_toggle & 0x0007f] = data;
		address_toggle++;
		return write_next_page(page_buffer);
	}
}

void specialFlashROMReset() {
	writeToFlashROM(SPECIAL_FLASH_ROM_ADDRESS_1,0x00aa);
	writeToFlashROM(SPECIAL_FLASH_ROM_ADDRESS_2,0x0055);
	writeToFlashROM(SPECIAL_FLASH_ROM_ADDRESS_1,0x00f0);
}


void identifyChip() {

	uint8_t manufacturer_code, device_code;

	char buf[80];


	printLineSerialUSB("Identify Flash ROM");
	// reset first
	specialFlashROMReset();


	writeToFlashROM(SPECIAL_FLASH_ROM_ADDRESS_1,0x00aa);
	writeToFlashROM(SPECIAL_FLASH_ROM_ADDRESS_2,0x0055);
	writeToFlashROM(SPECIAL_FLASH_ROM_ADDRESS_1,0x0090);

	manufacturer_code = (uint8_t) readFromFlashROM(0x0000);
	delay_us(2);
	device_code = (uint8_t) readFromFlashROM(0x0001);

	sprintf(buf,"Manufacturer: %02x , Device: %02x\r\n",manufacturer_code,device_code);
	printSerialUSB(buf);


	delay_us(1000);

	// reset again
	specialFlashROMReset();


}

void eraseChip() {
	uint32_t i;
	uint16_t d;

	//uint8_t buf[80];

	printSerialUSB("About to erase MX29F1615. Make sure you have 10V connected to _BYTE/VPP. I will give you 5 seconds\r\n");

	for (i=0;i<5;i++) {
		printSerialUSB(".");
		LL_mDelay(1000);
	}


	printSerialUSB("\r\nerasing chip\r\n");
	// reset first
	specialFlashROMReset();

	writeToFlashROM(SPECIAL_FLASH_ROM_ADDRESS_1,0x00aa);
	writeToFlashROM(SPECIAL_FLASH_ROM_ADDRESS_2,0x0055);
	writeToFlashROM(SPECIAL_FLASH_ROM_ADDRESS_1,0x0080);
	writeToFlashROM(SPECIAL_FLASH_ROM_ADDRESS_1,0x00aa);
	writeToFlashROM(SPECIAL_FLASH_ROM_ADDRESS_2,0x0055);
	writeToFlashROM(SPECIAL_FLASH_ROM_ADDRESS_1,0x0010);

	for (i=0;i<250;i++) {
		d = readFromFlashROM(0x0000);
		if (d & 0x0080) {
			printSerialUSB(" Erase Complete\r\n");
			break;
		}
		LL_mDelay(500);
	}


	// reset again
	specialFlashROMReset();
	printSerialUSB("\r\nNow do the following\r\n");
	printSerialUSB("- Wait a few minutes\r\n");
        printSerialUSB("- Disconnect the +10V from _BYTE/VPP\r\n");
	printSerialUSB("- Reconnect _BYTE/VPP to +5V\r\n");
	printSerialUSB("- Power cycle the MX29F1615 in order to read it\r\n");
	printSerialUSB("- Once you have power cycled, use the b command to check if its actually blank\r\n");

}

// from and to are byte addresses, not word addresses
void read_block(uint32_t from, uint32_t to, int linelength) {
	uint32_t addr;
	uint32_t i;
	uint16_t d;


	char buf[80];

	addr = from;

	while (addr < to ) {
		sprintf((char *)buf,"%05x  ",(unsigned int) (addr));
		printSerialUSB(buf);
		// show 8 words (ie. 16 bytes)
		for (i=0; i<8;i++) {
			d = readFromFlashROM((addr>>1)+i);
			sprintf(buf,"%04x ",d);
			printSerialUSB(buf);
		}
		printSerialUSB("\r\n");
		addr+=16;
	}

}

uint8_t hexDigit(uint8_t c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  else if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  else if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }
  else {
    return 0;
  }
}

// 5 digit hex number
uint32_t hexConvert(char* data) {
  int i = 0;
  uint32_t a,b;

  if (data[0]==0) {
     return 0;
  }
  b=0;
  while ( ((data[i]>='0') && (data[i]<='9')) || ((data[i]>='a') && (data[i]<='f')) || ((data[i]>='A') && (data[i]<='F')) ) {
    a = hexDigit(data[i++]);
    b = (b<<4) + a;
  }
  return b;
}
// 5 digit hex number
uint32_t hexFiveWord(char* data) {
  if (data[0]==0) {
     return 0;
  }
  return ((hexDigit(data[0])*65536L) +
          (hexDigit(data[1]) * 4096L) +
          (hexDigit(data[2]) * 256L) +
          (hexDigit(data[3]) * 16L) +
          (hexDigit(data[4])));
}

uint8_t hexByte(char* a)
{
  return ((hexDigit(a[0]) * 16) + hexDigit(a[1]));
}

uint16_t calccrc(uint8_t *ptr, int count)
{
    uint16_t  crc;
    int i;

    crc = 0;

    while (--count >= 0)
    {
        crc = crc ^ (uint16_t) (*ptr++ << 8);
        i = 8;
        do
        {
            if (crc & 0x8000)
                crc = crc << 1 ^ 0x1021;
            else
                crc = crc << 1;
        } while(--i);
    }
    return (crc);
}

void read_md5(uint32_t startAddress, uint32_t dataLength) {
	uint32_t  bytesleft;
	int bytestoprocess;
	int i;
	char md5buffer[64];
	char md5str[33];
	MD5_CTX context;
	unsigned char digest[16];
	char buf[80];
	uint16_t t;

	flashaddress=startAddress;

	md5str[0] = '\0';
	MD5Init(&context);

	sprintf(buf,"About to md5 from %05x to %05x\r\n",(unsigned int) startAddress,(unsigned int) (startAddress + dataLength - 1));
	printSerialUSB(buf);

	while (flashaddress < (startAddress + dataLength) ) {
	   // Work out how many bytes are left
	   bytesleft = ((startAddress + dataLength) - flashaddress);
	   bytestoprocess=64;
	   if (bytesleft < 64) {
	      bytestoprocess = bytesleft;
	   }
	   // TODO  this is unlikely to work well unless the file is even in length
	   for (i=0;i<bytestoprocess;i+=2) {
	      //md5buffer[i] = (char) read_next_byte();
	      t = readFromFlashROM((flashaddress>>1));
	      // TODO these might need to be swapped around
	      //md5buffer[i] = (char) (t &0xff);
	      //md5buffer[i+1] = (char) ((t &0xff00) >>8);
	      md5buffer[i] = (char) ((t &0xff00) >>8);
	      md5buffer[i+1] = (char) (t &0xff);
	      flashaddress+=2;
	   }
	   MD5Update(&context, md5buffer, bytestoprocess);
	   // No need to bump up flash address as read_next_byte does it.
	}

	MD5Final(digest, &context);

	make_digest(md5str, digest, 16);
	printLineSerialUSB(md5str);
	printSerialUSB("\r\n");

}

void blank_check(uint32_t startAddress, uint32_t dataLength) {
	char buf[80];
	uint16_t t;
	int max_failures=16;
	int failures=0;

	flashaddress=startAddress;

	sprintf(buf,"About to check for FFFFs from %05x to %05x\r\n",(unsigned int) startAddress,(unsigned int) (startAddress + dataLength - 1));
	printSerialUSB(buf);

	while (flashaddress < (startAddress + dataLength) ) {
	   t = readFromFlashROM((flashaddress>>1));
	   if (t != 0xffff) {
		failures++;
		sprintf((char *)buf,"FAIL %05x  = %04x\r\n",(unsigned int) (flashaddress), t);
		printSerialUSB(buf);
		if (failures>max_failures) {
			sprintf(buf,"Excessive failures. Aborting. You need to investigate\r\n");
			printSerialUSB(buf);
			break;
		}
	   }
	   flashaddress+=2;
	}
	if (failures==0) {
		sprintf(buf,"All Blank. Good\r\n");
	} else {
		sprintf(buf,"Some failures. Investigate\r\n");
	}
	printSerialUSB(buf);

}

uint8_t parseCommand() {
  char param[3][16];
  int i,x,y;
  char buf[128];

  char command;

  param[0][0]=0;
  param[1][0]=0;
  param[2][0]=0;

  for (i=0;cmdbuf[i]==' ';i++);
  command = cmdbuf[i++];
  x=0;
  y=0;
  while (cmdbuf[i]) {
    for (;cmdbuf[i]==' ';i++);
    y=0;
    while (cmdbuf[i]>' ') {
      param[x][y++]=cmdbuf[i++];
      param[x][y]=0;
    }
    if (y>0)
      x++;
  }


  startAddress = hexConvert(param[0]);
  dataLength = hexConvert(param[1]);
  lineLength = hexConvert(param[2]);


  //sprintf(buf,"numbers 0x%08lx 0x%08lx 0x%08lx\r\n",startAddress,dataLength, lineLength);
  printSerialUSB(buf);

  uint8_t retval = 0;
  switch (command) {
    case 'a':
      retval = SET_ADDRESS;
      break;
    case 'c':
      retval = CLEAR_RAM;
      break;
    case 'r':
      retval = READ_HEX;
      break;
    case 'w':
      retval = WRITE_XMODEM;
      break;
    case 't':  // write TWO bytes t xxxxx 0aabb
      retval = WRITE_HEX;
      break;
    case 'v':
      retval = VERSION;
      break;
    case '?':
      retval = HELP;
      break;
    case 'g':
      retval = GPIO_HELP;
      break;
    case 'i':
      retval = FLASH_IDENTIFY;
      break;
    case 'e':
      retval = FLASH_ERASE;
      break;
    case 'm':
      retval = MD5SUM;
      break;
    case 'b':
      retval = BLANK_CHECK;
      break;
    default:
      retval = NOCOMMAND;
      break;
  }
  return retval;
}


uint8_t oldParseCommand() {
  //set ',' to '\0' terminator (command string has a fixed strucure)
  //first string is the command character
  cmdbuf[1]  = 0;
  //second string is startaddress (5 bytes)
  cmdbuf[7]  = 0;
  //third string is a data length (orig file said endaddress (5 bytes))
  cmdbuf[13] = 0;
  //fourth string is length (2 bytes)
  cmdbuf[16] = 0;
  startAddress = hexFiveWord((cmdbuf + 2));
  dataLength = hexFiveWord((cmdbuf + 8));
  lineLength = hexByte(cmdbuf + 14);
  uint8_t retval = 0;
  switch (cmdbuf[0]) {
    case 'a':
      retval = SET_ADDRESS;
      break;
    case 'c':
      retval = CLEAR_RAM;
      break;
    case 'r':
      retval = READ_HEX;
      break;
    case 'w':
      retval = WRITE_XMODEM;
      break;
    case 't':  // write TWO bytes t xxxxx 0aabb
      retval = WRITE_HEX;
      break;
    case 'v':
      retval = VERSION;
      break;
    case '?':
      retval = HELP;
      break;
    case 'g':
      retval = GPIO_HELP;
      break;
    case 'i':
      retval = FLASH_IDENTIFY;
      break;
    case 'e':
      retval = FLASH_ERASE;
      break;
    case 'm':
      retval = MD5SUM;
      break;
    default:
      retval = NOCOMMAND;
      break;
  }
  return retval;
}

void readCommand() {
  //first clear command buffer
  for (int i = 0; i < COMMANDSIZE; i++) cmdbuf[i] = 0;
  //initialize variables
  char c = ' ';
  int idx = 0;
  //now read serial data until linebreak or buffer is full
  while (1) {
    if (idx >= COMMANDSIZE) {
      cmdbuf[idx - 1] = 0;
      break;
    }
    uint8_t data_rx = usb_check_buffer();

    if (data_rx) {
      c = usb_get_byte();
      if ((c == '\n') || (c == '\r')) {
        cmdbuf[idx] = 0;

        break;
      }
      // check for backspace
      if (c == 0x08) {
        cmdbuf[idx]=0;
        idx--;
        usb_write( (uint8_t *)&c,1); // echo back
      } else {
        usb_write( (uint8_t *)&c,1); // echo back
        cmdbuf[idx++] = c;
      }
    }
  }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  //uint8_t key;
  //char buf[160];
  uint8_t xmodem_byte;
  uint32_t crc;
  int i;


  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  LL_mDelay(1000);
  printHelp();


  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_1);
	//send_buffer[0] = x++;
	//CDC_Transmit_FS(send_buffer,3);
	//LL_mDelay(1000);

	if (loop_cmd == NOCOMMAND) {
		printSerialUSB("\r% "); // show the prompt
		readCommand();
		printSerialUSB("\r\n"); // only required in ECHO mode
		uint8_t cmd = parseCommand();

		switch (cmd) {
      			case FLASH_IDENTIFY:
				identifyChip();

				break;
			case FLASH_ERASE:
				eraseChip();
				break;

			case READ_HEX:
				specialFlashROMReset(); // read-reset
				//set a default if needed to prevent infinite loop
				if (lineLength == 0) lineLength = 16;
				if (dataLength == 0) {
					dataLength = 0x80;
				}
				endAddress = startAddress + dataLength - 1;
				read_block(startAddress, endAddress, lineLength);
				break;
			case WRITE_XMODEM:
				printSerialUSB("\r\nAbout to start flashing the MX29F1615 with a file you will transfer via xmodem\r\n");
				printSerialUSB("\r\nI am giving you 5 seconds to connect +10V to the _BYTE/VPP pin\r\n");
				for (i=0;i<5;i++) {
					printSerialUSB(".");
					LL_mDelay(1000);
				}
				printLineSerialUSB("\r\nWhen the xmodem transfer is finished, wait a few minutes, then put _BYTE/VPP back to +5V, then power cycle the MX29F1615\r\n");
				printLineSerialUSB("\r\n\r\nPlease start an xmodem transfer in your terminal ...");
				position = 0;
				bytes = 0;
				total=0;
				address_toggle = 0; 
				flashaddress=startAddress;
				specialFlashROMReset(); // read-reset
				//set_address_bus(startAddress);
				xmodem_prestart = 1;

				loop_cmd = WRITE_XMODEM;
				break;

			case MD5SUM:
 				//set a default if needed to prevent infinite loop

				if (dataLength == 0) {
					dataLength = 0x80;
				}
				endAddress = startAddress + dataLength - 1;
				printLineSerialUSB("Reading md5");
				read_md5(startAddress, dataLength);
				break;

			case BLANK_CHECK:
 				//set a default if needed to prevent infinite loop

				if (dataLength == 0) {
					dataLength = 0x80;
				}
				endAddress = startAddress + dataLength - 1;
				printLineSerialUSB("Checking for all FFFFs");
				blank_check(startAddress, dataLength);
				break;

			case VERSION:
				printLineSerialUSB(VERSIONSTRING);
				break;
			case HELP:
				printHelp();
				break;
			case GPIO_HELP:
				printGPIOHelp();
				break;
			default:
				break;
		}
	} else if (loop_cmd == WRITE_XMODEM) {
		if (xmodem_prestart) {  // In prestart mode we are just waiting for the sending device to start sending.
			printSerialUSB("C");
			LL_mDelay(1000);
			if (usb_check_buffer()) {
				xmodem_prestart = 0;
			}
		} else {
			//should be a byte read to read.
			while (usb_check_buffer()) {
				xmodem_byte = usb_get_byte();
				switch(position) {
					case POSITION_SOH:
						bytes=0;
						if (xmodem_byte == EOT) {
							printSerialUSB("\x06");  // send ACK
							loop_cmd = NOCOMMAND;
							//Serial.print("Wrote ");
							//Serial.println(total,DEC);
						}
						break;
					case POSITION_CRC:
						crchigh = xmodem_byte;
						break;
					case POSITION_CRC2:
						crc = calccrc(buffer,128);
						if ( (((crc >> 8) &0xff) == crchigh) && ( (crc & 0xff) == xmodem_byte) ) {
							for (int i=0;i<128;i++) {
								//write_next_byte2(buffer[i]);
								write_next_byte(buffer[i]);
								total++;
							}
							printSerialUSB("\x06");  // send ACK
						} else {
							printSerialUSB("\x15"); // CRC failed. Send NAK, and because we never wrote anything to SRAM, the address counter wont have moved
						}
						position = -1;
						break;
					default:
						if ((position >=POSITION_PAYLOAD) && (position <POSITION_CRC)) {
							buffer[bytes++] = xmodem_byte;
						}

						break;

         			}
         			position++;
      			}
		}
	}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_5)
  {
  Error_Handler();  
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 336, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 336, LL_RCC_PLLQ_DIV_7);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_SetSystemCoreClock(168000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();  
  };
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

  /**/
  LL_GPIO_ResetOutputPin(GPIOE, LL_GPIO_PIN_2|LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5 
                          |LL_GPIO_PIN_6|LL_GPIO_PIN_7|LL_GPIO_PIN_8|LL_GPIO_PIN_9 
                          |LL_GPIO_PIN_10|LL_GPIO_PIN_11|LL_GPIO_PIN_12|LL_GPIO_PIN_13 
                          |LL_GPIO_PIN_14|LL_GPIO_PIN_15|LL_GPIO_PIN_0|LL_GPIO_PIN_1);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3|LL_GPIO_PIN_4);

  /**/
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5 
                          |LL_GPIO_PIN_6|LL_GPIO_PIN_7|LL_GPIO_PIN_8|LL_GPIO_PIN_9 
                          |LL_GPIO_PIN_10|LL_GPIO_PIN_11|LL_GPIO_PIN_12|LL_GPIO_PIN_13 
                          |LL_GPIO_PIN_14|LL_GPIO_PIN_15|LL_GPIO_PIN_0|LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7
	  		|LL_GPIO_PIN_8|LL_GPIO_PIN_9|LL_GPIO_PIN_10|LL_GPIO_PIN_11 
                          |LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
