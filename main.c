/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
/*
	Takes an VGA image of OV9655 camera and encodes it in realtime to JPEG.
	The image has a capture time of 264ms.
	For sampling a picture PA0 must be pulled high. (User button on STM32F429 discovery board)
	The picture is then transmitted by serial interface on PD5 (38k2 8n1).
	PG13 is blinking (Green LED on STM32F429 discovery board) when the uC is
	ideling. So when the picture has been transmitted completely PG13 is
	blinking.

	This example does not need any external SDRAM. It just uses it's internal
	SRAM. So this example should also work on the STM32F407 discovery board.

	The inital code came from http://mikrocontroller.bplaced.net/wordpress/?page_id=1115

	The camera must be connected to

	Camera Data pin		STM32 designator	Possible Hardware Pins for DCMI		Pin used in this implementation
	HREF				HSYNC				PA4 PH8								PA4
	PCLK				PIXCLK				PA6									PA6
	VSYNC				VSYNC				PB7 PG9 PI5							PB7
	D2					D0					PA9 PC6 PH9							PC6
	D3					D1					PA10 PC7 PH10						PC7
	D4					D2					PC8 PE0 PG10 PH11					PC8
	D5					D3					PC9 PE1 PG11 PH12					PE1
	D6					D4					PC11 PE4 PH14						PE4
	D7					D5					PB6 PD3 PI4							PB6
	D8					D6					PB8 PE5 PI6							PE5
	D9					D7					PB9 PE6 PI7							PE6
	XCLK				none				PA8 (clockout, not DCMI specific)	PA8

	RET and PWDN can be left unconnected
*/

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dcmi.h"

#include "encoder/arch.h"
#include "encoder/dct.h"
#include "encoder/jpegenc.h"

#include <string.h>

#define QVGA

#ifdef QVGA
#define OV9655_MAXX			320
#define OV9655_MAXY			240
#else
#define OV9655_MAXX			640
#define OV9655_MAXY			480
#endif

#define OV9655_BUFFER_SIZE	150*1024

#define OV9655_DCMI_BASE_ADR		((uint32_t)0x50050000)
#define OV9655_DCMI_REG_DR_OFFSET	0x28
#define OV9655_DCMI_REG_DR_ADDRESS	(OV9655_DCMI_BASE_ADR | OV9655_DCMI_REG_DR_OFFSET)

#define  OV9655_I2C_ADR        0x30  /* Slave-address vom OV9655 */


uint8_t ov9655_ram_buffer[OV9655_BUFFER_SIZE];
volatile uint32_t buffNum = 0;

uint64_t subsclk = 0;
uint64_t readclk = 0;
uint32_t jpeg_pos = 0;
bool samplingFinished;

uint8_t rxbuf[1];

// I2C interface
static const I2CConfig i2cfg2 = {
    OPMODE_I2C,
    200000,
    FAST_DUTY_CYCLE_2,
};

// UART interface
static UARTConfig uart_cfg_1 = {
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	115200,
	0,
	USART_CR2_LINEN,
	0
};

// I2C camera configuration for QVGA resolution
static const uint8_t OV9655_CONFIG[] =
{

     0xFF, 0x01,
     0x12, 0x80,

0xff, 0x00,
0x2c, 0xff,
0x2e, 0xdf,
0xff, 0x01,
0x3c, 0x32,
     //{0x11, 0x30,  // Divide clock by 49
0x11, 0x02,   // Divide clock by 3
0x09, 0x02,
0x04, 0x28,
0x13, 0xe5,
0x14, 0x48,
0x2c, 0x0c,
0x33, 0x78,
0x3a, 0x33,
0x3b, 0xfB,
0x3e, 0x00,
0x43, 0x11,
0x16, 0x10,
0x39, 0x92,
0x35, 0xda,
0x22, 0x1a,
0x37, 0xc3,
0x23, 0x00,
0x34, 0xc0,
0x36, 0x1a,
0x06, 0x88,
0x07, 0xc0,
0x0d, 0x87,
0x0e, 0x41,
0x4c, 0x00,
0x48, 0x00,
0x5B, 0x00,
0x42, 0x03,
0x4a, 0x81,
0x21, 0x99,
0x24, 0x40,
0x25, 0x38,
0x26, 0x82,
0x5c, 0x00,
0x63, 0x00,
0x61, 0x70,
0x62, 0x80,
0x7c, 0x05,
0x20, 0x80,
0x28, 0x30,
0x6c, 0x00,
0x6d, 0x80,
0x6e, 0x00,
0x70, 0x02,
0x71, 0x94,
0x73, 0xc1,
0x12, 0x40,
0x17, 0x11,
0x18, 0x43,
0x19, 0x00,
0x1a, 0x4b,
0x32, 0x09,
0x37, 0xc0,
0x4f, 0x60,
0x50, 0xa8,
0x6d, 0x00,
0x3d, 0x38,
0x46, 0x3f,
0x4f, 0x60,
0x0c, 0x3c,
0xff, 0x00,
0xe5, 0x7f,
0xf9, 0xc0,
0x41, 0x24,
0xe0, 0x14,
0x76, 0xff,
0x33, 0xa0,
0x42, 0x20,
0x43, 0x18,
0x4c, 0x00,
0x87, 0xd5,
0x88, 0x3f,
0xd7, 0x03,
0xd9, 0x10,
0xd3, 0x82,
0xc8, 0x08,
0xc9, 0x80,
0x7c, 0x00,
0x7d, 0x00,
0x7c, 0x03,
0x7d, 0x48,
0x7d, 0x48,
0x7c, 0x08,
0x7d, 0x20,
0x7d, 0x10,
0x7d, 0x0e,
0x90, 0x00,
0x91, 0x0e,
0x91, 0x1a,
0x91, 0x31,
0x91, 0x5a,
0x91, 0x69,
0x91, 0x75,
0x91, 0x7e,
0x91, 0x88,
0x91, 0x8f,
0x91, 0x96,
0x91, 0xa3,
0x91, 0xaf,
0x91, 0xc4,
0x91, 0xd7,
0x91, 0xe8,
0x91, 0x20,
0x92, 0x00,
0x93, 0x06,
0x93, 0xe3,
0x93, 0x05,
0x93, 0x05,
0x93, 0x00,
0x93, 0x04,
0x93, 0x00,
0x93, 0x00,
0x93, 0x00,
0x93, 0x00,
0x93, 0x00,
0x93, 0x00,
0x93, 0x00,
0x96, 0x00,
0x97, 0x08,
0x97, 0x19,
0x97, 0x02,
0x97, 0x0c,
0x97, 0x24,
0x97, 0x30,
0x97, 0x28,
0x97, 0x26,
0x97, 0x02,
0x97, 0x98,
0x97, 0x80,
0x97, 0x00,
0x97, 0x00,
0xc3, 0xed,
0xa4, 0x00,
0xa8, 0x00,
0xc5, 0x11,
0xc6, 0x51,
0xbf, 0x80,
0xc7, 0x10,
0xb6, 0x66,
0xb8, 0xA5,
0xb7, 0x64,
0xb9, 0x7C,
0xb3, 0xaf,
0xb4, 0x97,
0xb5, 0xFF,
0xb0, 0xC5,
0xb1, 0x94,
0xb2, 0x0f,
0xc4, 0x5c,
0xc0, 0x64,
0xc1, 0x4B,
0x8c, 0x00,
0x86, 0x3D,
0x50, 0x00,
0x51, 0xC8,
0x52, 0x96,
0x53, 0x00,
0x54, 0x00,
0x55, 0x00,
0x5a, 0xC8,
0x5b, 0x96,
0x5c, 0x00,
0xd3, 0x7f,
0xc3, 0xed,
0x7f, 0x00,
0xda, 0x00,
0xe5, 0x1f,
0xe1, 0x67,
0xe0, 0x00,
0xdd, 0x7f,
0x05, 0x00,
0x12, 0x40,
0xd3, 0x7f,
0xc0, 0x16,
0xC1, 0x12,
0x8c, 0x00,
0x86, 0x3d,
0x50, 0x00,
0x51, 0x2C,
0x52, 0x24,
0x53, 0x00,
0x54, 0x00,
0x55, 0x00,
0x5A, 0x2c,
0x5b, 0x24,
0x5c, 0x00,
0xff, 0xff,

0xFF, 0x00,
0x05, 0x00,
0xDA, 0x10,
0xD7, 0x03,
0xDF, 0x00,
0x33, 0x80,
0x3C, 0x40,
0xe1, 0x77,
0x00, 0x00,
0xff, 0xff,

0xFF, 0x01,
0x15, 0x00,

0xe0, 0x14,
0xe1, 0x77,
0xe5, 0x1f,
0xd7, 0x03,
0xda, 0x10,
0xe0, 0x00,
0xFF, 0x01,
0x04, 0x08,
0xff, 0xff,






	0xff, 0x01,
	0x11, 0x01,
	0x12, 0x00, // Bit[6:4]: Resolution selection//0x02为彩条
	0x17, 0x11, // HREFST[10:3]
	0x18, 0x75, // HREFEND[10:3]
	0x32, 0x36, // Bit[5:3]: HREFEND[2:0]; Bit[2:0]: HREFST[2:0]
	0x19, 0x01, // VSTRT[9:2]
	0x1a, 0x97, // VEND[9:2]
	0x03, 0x0f, // Bit[3:2]: VEND[1:0]; Bit[1:0]: VSTRT[1:0]
	0x37, 0x40,
	0x4f, 0xbb,
	0x50, 0x9c,
	0x5a, 0x57,
	0x6d, 0x80,
	0x3d, 0x34,
	0x39, 0x02,
	0x35, 0x88,
	0x22, 0x0a,
	0x37, 0x40,
	0x34, 0xa0,
	0x06, 0x02,
	0x0d, 0xb7,
	0x0e, 0x01,
	
	0xff, 0x00,        	                              
	0xe0, 0x04,                                   
	0xc0, 0xc8,                                   
	0xc1, 0x96,                                   
	0x86, 0x3d,                                   
	0x50, 0x00,                                   
	0x51, 0x90,                                   
	0x52, 0x2c,                                   
	0x53, 0x00,                                   
	0x54, 0x00,                                   
	0x55, 0x88,                                   
	0x57, 0x00,                                   
	0x5a, 0x90,                                   
	0x5b, 0x2C,                                   
	0x5c, 0x05,              //bit2->1;bit[1:0]->1
	0xd3, 0x02,                                   
	0xe0, 0x00,                                   
                      
  	0xff, 0xff,





/*
// 1024*768
0xff, 0x01,
0x11, 0x00,
0x12, 0x00,
0x17, 0x11,
0x18, 0x75,
0x32, 0x36,
0x19, 0x01,
0x1a, 0x97,
0x03, 0x0f,
0x37, 0x40,
0x4f, 0xbb,
0x50, 0x9c,
0x5a, 0x57,
0x6d, 0x80,
0x3d, 0x34,
0x39, 0x02,
0x35, 0x88,
0x22, 0x0a,
0x37, 0x40,
0x34, 0xa0,
0x06, 0x02,
0x0d, 0xb7,
0x0e, 0x01,
0xff, 0x00,
0xc0, 0xc8,
0xc1, 0x96,
0x8c, 0x00,
0x86, 0x3d,
0x50, 0x00,
0x51, 0x90,
0x52, 0x2c,
0x53, 0x00,
0x54, 0x00,
0x55, 0x88,
0x5a, 0x00,
0x5b, 0xc0,
0x5c, 0x01,
0xd3, 0x02,
0xff, 0xff,*/






0xe0, 0x14,
0xe1, 0x77,
0xe5, 0x1f,
0xd7, 0x03,
0xda, 0x10,
0xe0, 0x00,
0xFF, 0x01,
0x04, 0x08,
0xff, 0xff



};


void OV9655_RAM2SD(void);
bool OV9655_Snapshot2RAM(void);
void OV9655_InitDMA(void);
void OV9655_InitDCMI(void);
void OV9655_InitGPIO(void);
void dma_avail(uint32_t flags);

/**
  * Captures an image from the camera.
  * Returns false in case of an error.
  */
bool OV9655_Snapshot2RAM(void)
{
	palSetPad(GPIOG, 14);

	// DCMI init
	OV9655_InitDCMI();

	// Encode JPEG data
	samplingFinished = false;
	systime_t timeout = chVTGetSystemTimeX() + MS2ST(3000); // Timeout 1sec
	while(!samplingFinished && chVTGetSystemTimeX() < timeout)
		chThdSleepMilliseconds(1);

	palClearPad(GPIOG, 14);

	return true;
}

/**
  * Transmits JPEG encoded buffer by serial
  */
void OV9655_RAM2SD(void)
{
	uint32_t i;
	for(i=0; i<150; i++)
	{
		uartStartSend(&UARTD2, 1024, &ov9655_ram_buffer[i*1024]);
		chThdSleepMilliseconds(100);
	}
}

/**
  * ISR callback called when DCMI buffer moved into RAM by DMA completly
  * Buffer size: OV9655_BUFFER_SIZE = OV9655_MAXX * 16 * bytes
  * Double buffer mode is activated.
  */
void dma_avail(uint32_t flags)
{
	(void)flags;
	samplingFinished = true;
	dmaStreamDisable(STM32_DMA2_STREAM1);
}

/**
  * Initializes DMA
  */
void OV9655_InitDMA(void)
{
    const stm32_dma_stream_t *stream = STM32_DMA2_STREAM1;
    dmaStreamAllocate(stream, 10, (stm32_dmaisr_t)dma_avail, NULL);
    dmaStreamSetPeripheral(stream, ((uint32_t*)OV9655_DCMI_REG_DR_ADDRESS));
    dmaStreamSetMemory0(stream, (uint32_t)ov9655_ram_buffer);
    dmaStreamSetTransactionSize(stream, OV9655_BUFFER_SIZE);
    dmaStreamSetMode(stream, STM32_DMA_CR_CHSEL(1) | STM32_DMA_CR_DIR_P2M |
							STM32_DMA_CR_MINC | STM32_DMA_CR_PSIZE_WORD |
							STM32_DMA_CR_MSIZE_WORD | STM32_DMA_CR_MBURST_SINGLE |
							STM32_DMA_CR_PBURST_SINGLE | STM32_DMA_CR_TCIE);
    dmaStreamSetFIFO(stream, STM32_DMA_FCR_FTH_HALF | STM32_DMA_FCR_DMDIS);
    dmaStreamEnable(stream);
}

void OV9655_DeinitDMA(void)
{
    const stm32_dma_stream_t *stream = STM32_DMA2_STREAM1;
    dmaStreamDisable(stream);
}


/**
  * Initializes DCMI
  */
void OV9655_InitDCMI(void)
{
	// Clock enable
	RCC->AHB2ENR |= RCC_AHB2Periph_DCMI;

	// Configure DCMI
	DCMI->CR = DCMI_CaptureMode_SnapShot | DCMI_CR_JPEG | DCMI_CR_PCKPOL;

	// DCMI enable
	DCMI->CR |= (uint32_t)DCMI_CR_ENABLE;
	// Capture enable
	DCMI->CR |= (uint32_t)DCMI_CR_CAPTURE;
}

void OV9655_DeinitDCMI(void)
{
	// Clock enable
	RCC->AHB2ENR &= ~RCC_AHB2Periph_DCMI;
}

/**
  * Initializes GPIO (for DCMI)
  * The high speed clock supports communication by I2C (XCLK = 16MHz)
  */
void OV9655_InitGPIO(void)
{
	palSetPadMode(GPIOA, 4, PAL_MODE_ALTERNATE(13)); // HSYNC -> PA4
	palSetPadMode(GPIOA, 6, PAL_MODE_ALTERNATE(13)); // PCLK  -> PA6
	palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(13)); // VSYNC -> PB7
	palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(13)); // D0    -> PC6
	palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(13)); // D1    -> PC7
	palSetPadMode(GPIOC, 8, PAL_MODE_ALTERNATE(13)); // D2    -> PC8
	palSetPadMode(GPIOE, 1, PAL_MODE_ALTERNATE(13)); // D3    -> PE1
	palSetPadMode(GPIOE, 4, PAL_MODE_ALTERNATE(13)); // D4    -> PE4
	palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(13)); // D5    -> PB6
	palSetPadMode(GPIOE, 5, PAL_MODE_ALTERNATE(13)); // D6    -> PE5
	palSetPadMode(GPIOE, 6, PAL_MODE_ALTERNATE(13)); // D7    -> PE6
}

/**
  * Setup a CLOCKOUT pin (PA8) which is needed by the camera (XCLK pin)
  */
void OV9655_InitClockout(void)
{
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(0)); // PA8             -> XCLK
}


void OV9655_TransmitConfig(void) {
	for(uint32_t i=0; i<sizeof(OV9655_CONFIG); i+=2) {
		i2cAcquireBus(&I2CD2);
		i2cMasterTransmitTimeout(&I2CD2, OV9655_I2C_ADR, &OV9655_CONFIG[i], 2, rxbuf, 0, MS2ST(100));
		i2cReleaseBus(&I2CD2);
		chThdSleepMilliseconds(10);
	}
}

void OV9655_init(void) {
	OV9655_InitClockout();
	OV9655_InitGPIO();

	// Send settings to OV9655
	OV9655_TransmitConfig();

	// DCMI DMA
	OV9655_InitDMA();

	// DCMI Init
	OV9655_InitDCMI();
}

void OV9655_deinit(void) {
	// DCMI Init
	OV9655_DeinitDCMI();

	// DCMI DMA
	OV9655_DeinitDMA();
}

/**
  * Entry point of program
  */
int main(void) {
	halInit();
	chSysInit();

	uint32_t i;
	for(i=0; i<OV9655_BUFFER_SIZE; i++)
		ov9655_ram_buffer[i] = 0;

	// Init LEDs
	palSetPadMode(GPIOG, 13, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPadMode(GPIOG, 14, PAL_MODE_OUTPUT_PUSHPULL);

	// Init I2C
	i2cStart(&I2CD2, &i2cfg2);

	// Init UART (2Mbit 8n1 TXD=PD5 RXD=PD6)
	palSetPadMode(GPIOD, 5, PAL_MODE_ALTERNATE(7));
	palSetPadMode(GPIOD, 6, PAL_MODE_ALTERNATE(7));
	uartStart(&UARTD2, &uart_cfg_1);

	uartStartSend(&UARTD2, 5, ">>>>>");

	OV9655_init();
	OV9655_Snapshot2RAM();	// Sample data from DCMI through DMA to RAM
	OV9655_RAM2SD();		// Transmit image (JPEG-encoded) by serial to computer

	while (true) {
		palTogglePad(GPIOG, 13); // Toogle LED (blinking)
		chThdSleepMilliseconds(100);
	}
}














