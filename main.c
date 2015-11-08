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
   Concepts and parts of this file have been contributed by Uladzimir Pylinsky
   aka barthess.
 */

/**
 * This demo acquire data from accelerometer and prints it in shell.
 */
#include <stdlib.h>

#include "ch.h"
#include "hal.h"
//#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dcmi.h"

static uint8_t rxbuf[4];

// Missing stm32f4xx.h def
#define DMA_SxFCR_FTH                        ((uint32_t)0x00000003)
#define DMA_SxFCR_DMDIS                      ((uint32_t)0x00000004)

// Missing from stm32f4xx_dma.c
#define DMA_Stream0_IT_MASK     (uint32_t)(DMA_LISR_FEIF0 | DMA_LISR_DMEIF0 | \
                                           DMA_LISR_TEIF0 | DMA_LISR_HTIF0 | \
                                           DMA_LISR_TCIF0)
#define DMA_Stream1_IT_MASK     (uint32_t)(DMA_Stream0_IT_MASK << 6)

void UB_OV9655_RAM2BMP(void);
void UB_OV9655_Snapshot2RAM(void);
void P_OV9655_InitDMA(void);
void P_OV9655_InitDCMI(void);
void P_OV9655_InitIO(void);

#define OV9655_MAXX   160
#define OV9655_MAXY   120
#define OV9655_PIXEL  OV9655_MAXX*OV9655_MAXY

#define OV9655_DCMI_DMA_STREAM            DMA2_Stream1
#define OV9655_DCMI_DMA_CHANNEL           DMA_Channel_1
#define OV9655_DCMI_BASE_ADR        ((uint32_t)0x50050000)
#define OV9655_DCMI_REG_DR_OFFSET   0x28  // DR-Register
#define OV9655_DCMI_REG_DR_ADDRESS  (OV9655_DCMI_BASE_ADR | OV9655_DCMI_REG_DR_OFFSET)

#define   BMP_HEADER_LEN    54


char buffer[32];
uint16_t ov9655_ram_buffer[OV9655_PIXEL];

const uint8_t BMP_HEADER_320x240[BMP_HEADER_LEN]={
0x42,0x4D,0x36,0xE1,0x00,0x00,0x00,0x00,0x00,0x00, // ID=BM, Filsize=(120x160x3+54)
0x36,0x00,0x00,0x00,0x28,0x00,0x00,0x00,           // Offset=54d, Headerlen=40d
0xA0,0x00,0x00,0x00,0x78,0x00,0x00,0x00,0x01,0x00, // W=160d, H=120d (landscape)
0x18,0x00,0x00,0x00,0x00,0x00,0x00,0xE1,0x00,0x00, // 24bpp, uncompressed, Data=(120x160x3)
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,           // nc
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00            // nc
/*
0x42,0x4D,0x36,0x84,0x03,0x00,0x00,0x00,0x00,0x00, // ID=BM, Filsize=(240x320x3+54)
0x36,0x00,0x00,0x00,0x28,0x00,0x00,0x00,           // Offset=54d, Headerlen=40d
0x40,0x01,0x00,0x00,0xF0,0x00,0x00,0x00,0x01,0x00, // W=320d, H=240d (landscape)
0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x84,0x03,0x00, // 24bpp, uncompressed, Data=(240x320x3)
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,           // nc
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00            // nc
*/
};

/**
 *
 */
static void print(char *p) {

  while (*p) {
    sdPut(&SD2, *p++);
  }
}



/* I2C interface #2 */
static const I2CConfig i2cfg2 = {
    OPMODE_I2C,
    400000,
    FAST_DUTY_CYCLE_2,
};

static uint8_t OV9655_QVGA_TAB[]=
{
0x00,0x00,0x01,0x80,0x02,0x80,0x03,0x02,0x04,0x03,0x09,0x01, // <= I2C camera for 160*120
0x0b,0x57,0x0e,0x61,0x0f,0x40,0x11,0x01,0x12,0x62,0x13,0xc7,
0x14,0x3a,0x16,0x24,0x17,0x18,0x18,0x04,0x19,0x01,0x1a,0x81,
0x1e,0x00,0x24,0x3c,0x25,0x36,0x26,0x72,0x27,0x08,0x28,0x08,
0x29,0x15,0x2a,0x00,0x2b,0x00,0x2c,0x08,0x32,0xa4,0x33,0x00,
0x34,0x3f,0x35,0x00,0x36,0x3a,0x38,0x72,0x39,0x57,0x3a,0xcc,
0x3b,0x04,0x3d,0x99,0x3e,0x0e,0x3f,0xc1,0x40,0xc0,0x41,0x41,
0x42,0xc0,0x43,0x0a,0x44,0xf0,0x45,0x46,0x46,0x62,0x47,0x2a,
0x48,0x3c,0x4a,0xfc,0x4b,0xfc,0x4c,0x7f,0x4d,0x7f,0x4e,0x7f,
0x4f,0x98,0x50,0x98,0x51,0x00,0x52,0x28,0x53,0x70,0x54,0x98,
0x58,0x1a,0x59,0x85,0x5a,0xa9,0x5b,0x64,0x5c,0x84,0x5d,0x53,
0x5e,0x0e,0x5f,0xf0,0x60,0xf0,0x61,0xf0,0x62,0x00,0x63,0x00,
0x64,0x02,0x65,0x20,0x66,0x00,0x69,0x0a,0x6b,0x5a,0x6c,0x04,
0x6d,0x55,0x6e,0x00,0x6f,0x9d,0x70,0x21,0x71,0x78,0x72,0x22,
0x73,0x02,0x74,0x10,0x75,0x10,0x76,0x01,0x77,0x02,0x7A,0x12,
0x7B,0x08,0x7C,0x16,0x7D,0x30,0x7E,0x5e,0x7F,0x72,0x80,0x82,
0x81,0x8e,0x82,0x9a,0x83,0xa4,0x84,0xac,0x85,0xb8,0x86,0xc3,
0x87,0xd6,0x88,0xe6,0x89,0xf2,0x8a,0x24,0x8c,0x80,0x90,0x7d,
0x91,0x7b,0x9d,0x02,0x9e,0x02,0x9f,0x7a,0xa0,0x79,0xa1,0x40,
0xa4,0x50,0xa5,0x68,0xa6,0x4a,0xa8,0xc1,0xa9,0xef,0xaa,0x92,
0xab,0x04,0xac,0x80,0xad,0x80,0xae,0x80,0xaf,0x80,0xb2,0xf2,
0xb3,0x20,0xb4,0x20,0xb5,0x00,0xb6,0xaf,0xb6,0xaf,0xbb,0xae,
0xbc,0x7f,0xbd,0x7f,0xbe,0x7f,0xbf,0x7f,0xbf,0x7f,0xc0,0xaa,
0xc1,0xc0,0xc2,0x01,0xc3,0x4e,0xc6,0x05,0xc7,0x82,0xc9,0xe0,
0xca,0xe8,0xcb,0xf0,0xcc,0xd8,0xcd,0x93,0x12,0x63,0x40,0x10,
0x15,0x08
/*0x00,0x00,0x01,0x80,0x02,0x80,0x03,0x02,0x04,0x03,0x09,0x01, // <= I2C camera for QVGA (only possible when external RAM configured)
0x0b,0x57,0x0e,0x61,0x0f,0x40,0x11,0x01,0x12,0x62,0x13,0xc7,
0x14,0x3a,0x16,0x24,0x17,0x18,0x18,0x04,0x19,0x01,0x1a,0x81,
0x1e,0x00,0x24,0x3c,0x25,0x36,0x26,0x72,0x27,0x08,0x28,0x08,
0x29,0x15,0x2a,0x00,0x2b,0x00,0x2c,0x08,0x32,0x12,0x33,0x00,
0x34,0x3f,0x35,0x00,0x36,0x3a,0x38,0x72,0x39,0x57,0x3a,0xcc,
0x3b,0x04,0x3d,0x99,0x3e,0x02,0x3f,0xc1,0x40,0xc0,0x41,0x41,
0x42,0xc0,0x43,0x0a,0x44,0xf0,0x45,0x46,0x46,0x62,0x47,0x2a,
0x48,0x3c,0x4a,0xfc,0x4b,0xfc,0x4c,0x7f,0x4d,0x7f,0x4e,0x7f,
0x4f,0x98,0x50,0x98,0x51,0x00,0x52,0x28,0x53,0x70,0x54,0x98,
0x58,0x1a,0x59,0x85,0x5a,0xa9,0x5b,0x64,0x5c,0x84,0x5d,0x53,
0x5e,0x0e,0x5f,0xf0,0x60,0xf0,0x61,0xf0,0x62,0x00,0x63,0x00,
0x64,0x02,0x65,0x20,0x66,0x00,0x69,0x0a,0x6b,0x5a,0x6c,0x04,
0x6d,0x55,0x6e,0x00,0x6f,0x9d,0x70,0x21,0x71,0x78,0x72,0x11,
0x73,0x01,0x74,0x10,0x75,0x10,0x76,0x01,0x77,0x02,0x7A,0x12,
0x7B,0x08,0x7C,0x16,0x7D,0x30,0x7E,0x5e,0x7F,0x72,0x80,0x82,
0x81,0x8e,0x82,0x9a,0x83,0xa4,0x84,0xac,0x85,0xb8,0x86,0xc3,
0x87,0xd6,0x88,0xe6,0x89,0xf2,0x8a,0x24,0x8c,0x80,0x90,0x7d,
0x91,0x7b,0x9d,0x02,0x9e,0x02,0x9f,0x7a,0xa0,0x79,0xa1,0x40,
0xa4,0x50,0xa5,0x68,0xa6,0x4a,0xa8,0xc1,0xa9,0xef,0xaa,0x92,
0xab,0x04,0xac,0x80,0xad,0x80,0xae,0x80,0xaf,0x80,0xb2,0xf2,
0xb3,0x20,0xb4,0x20,0xb5,0x00,0xb6,0xaf,0xb6,0xaf,0xbb,0xae,
0xbc,0x7f,0xbd,0x7f,0xbe,0x7f,0xbf,0x7f,0xbf,0x7f,0xc0,0xaa,
0xc1,0xc0,0xc2,0x01,0xc3,0x4e,0xc6,0x05,0xc7,0x81,0xc9,0xe0,
0xca,0xe8,0xcb,0xf0,0xcc,0xd8,0xcd,0x93,0x12,0x63,0x40,0x10,
0x15,0x08*/
};

#define  OV9655_I2C_ADR        0x30  // Slave-Adresse vom OV9655

/*
 * Application entry point.
 */
int main(void) {
	systime_t tmo = MS2ST(100);

	/*
	* System initializations.
	* - HAL initialization, this also initializes the configured device drivers
	*   and performs the board-specific initializations.
	* - Kernel initialization, the main() function becomes a thread and the
	*   RTOS is active.
	*/
	halInit();
	chSysInit();

	palSetPadMode(GPIOG, 13, PAL_MODE_OUTPUT_PUSHPULL);

	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(0));
	RCC->CFGR = (RCC->CFGR & (uint32_t)0xF0FFFFFF) | (uint32_t)0x08000000; 

	/*
	* Starts I2C
	*/
	i2cStart(&I2CD2, &i2cfg2);

	/*
	* Prepares the Serial driver 2
	*/
	sdStart(&SD2, NULL);          /* Default is 38400-8-N-1.*/
	palSetPadMode(GPIOD, 5, PAL_MODE_ALTERNATE(7));
	palSetPadMode(GPIOD, 6, PAL_MODE_ALTERNATE(7));

	// QVGA = 320x240 Pixel
	uint32_t anzahl = sizeof(OV9655_QVGA_TAB);
	uint32_t i;
	for(i=0; i<anzahl; i+=2) {
		i2cAcquireBus(&I2CD2);
		i2cMasterTransmitTimeout(&I2CD2, OV9655_I2C_ADR, &OV9655_QVGA_TAB[i], 2, rxbuf, 0, tmo);
		i2cReleaseBus(&I2CD2);
		chThdSleepMilliseconds(3);
	}

	UB_OV9655_Snapshot2RAM();
	UB_OV9655_RAM2BMP();

	while (true) {
		if (palReadPad(GPIOA, 0)) { // User button pressed
			UB_OV9655_Snapshot2RAM(); // Sample data from DCMI through DMA to RAM
			UB_OV9655_RAM2BMP(); // Transmit image (BMP-encoded) by serial to computer
		}
		palTogglePad(GPIOG, 13); // Toogle LED
		chThdSleepMilliseconds(100);
	}
}

void UB_OV9655_Snapshot2RAM(void)
{
	P_OV9655_InitIO();
	static bool dma_mode = 0;
	if(dma_mode==0) {
		// Init GPIO
		P_OV9655_InitIO();
		// Init DMA-Mode
		P_OV9655_InitDMA();
		dma_mode=1;
	}
	// DMA enable
	DMA2_Stream1->CR |= (uint32_t)DMA_SxCR_EN;
	// DCMI init
	P_OV9655_InitDCMI();
	// DCMI enable
	DCMI->CR |= (uint32_t)DCMI_CR_ENABLE;
	// Capture enable
	DCMI->CR |= (uint32_t)DCMI_CR_CAPTURE;
}


void UB_OV9655_RAM2BMP(void)
{
  uint32_t n,adr=0;
  uint16_t color,x,y;
  uint8_t r,g,b;

  char newbuffer[2] = {0,0};
  print(newbuffer);

  // Send BMP-Header
  for(n=0;n<BMP_HEADER_LEN;n++) {
		sdPut(&SD2, BMP_HEADER_320x240[n]);
  }

  // Send color data
  adr=0;
  for(x=0;x<OV9655_MAXX;x++) {
    for(y=0;y<OV9655_MAXY;y++) {
      color=ov9655_ram_buffer[adr];
      adr++;
      r=((color&0xF800)>>8);  // 5bit red
      g=((color&0x07E0)>>3);  // 6bit green
      b=((color&0x001F)<<3);  // 5bit blue

		sdPut(&SD2, b);
		sdPut(&SD2, g);
		sdPut(&SD2, r);
    }
  }

}

void P_OV9655_InitDMA()
{
	//DMA_InitTypeDef  DMA_InitStructure;

	// Clock enable
	RCC->AHB1ENR |= RCC_AHB1Periph_DMA2;

	// DMA deinit
	DMA2_Stream1->CR &= ~(uint32_t)DMA_SxCR_EN;

	//DMA_DeInit(OV9655_DCMI_DMA_STREAM);

	/* Disable the selected DMAy Streamx */
	DMA2_Stream1->CR &= ~((uint32_t)DMA_SxCR_EN);
	/* Reset DMAy Streamx control register */
	DMA2_Stream1->CR  = 0;
	/* Reset DMAy Streamx Number of Data to Transfer register */
	DMA2_Stream1->NDTR = 0;
	/* Reset DMAy Streamx peripheral address register */
	DMA2_Stream1->PAR  = 0;
	/* Reset DMAy Streamx memory 0 address register */
	DMA2_Stream1->M0AR = 0;
	/* Reset DMAy Streamx memory 1 address register */
	DMA2_Stream1->M1AR = 0;
	/* Reset DMAy Streamx FIFO control register */
	DMA2_Stream1->FCR = (uint32_t)0x00000021; 
	/* Reset interrupt pending bits for DMA2 Stream1 */
	DMA2->LIFCR = DMA_Stream1_IT_MASK;












	/*DMA_InitStructure.DMA_Channel = OV9655_DCMI_DMA_CHANNEL;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(OV9655_DCMI_DMA_STREAM, &DMA_InitStructure);*/

	DMA2_Stream1->CR &=	((uint32_t)~(DMA_SxCR_CHSEL | DMA_SxCR_MBURST | DMA_SxCR_PBURST |
						DMA_SxCR_PL | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE |
						DMA_SxCR_MINC | DMA_SxCR_PINC | DMA_SxCR_CIRC |
						DMA_SxCR_DIR));
	DMA2_Stream1->CR |=	OV9655_DCMI_DMA_CHANNEL | DMA_DIR_PeripheralToMemory | DMA_PeripheralInc_Disable |
						DMA_MemoryInc_Enable | DMA_PeripheralDataSize_Word | DMA_MemoryDataSize_HalfWord |
						DMA_Mode_Circular | DMA_Priority_High | DMA_MemoryBurst_Single | DMA_PeripheralBurst_Single;


	/*DMA_InitStructure.DMA_PeripheralBaseAddr = OV9655_DCMI_REG_DR_ADDRESS;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ov9655_ram_buffer;
	DMA_InitStructure.DMA_BufferSize = OV9655_PIXEL;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;*/

	DMA2_Stream1->FCR &= (uint32_t)~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);
	DMA2_Stream1->FCR |= DMA_FIFOMode_Enable | DMA_FIFOThreshold_Full;

	DMA2_Stream1->NDTR = OV9655_PIXEL;
	DMA2_Stream1->PAR = OV9655_DCMI_REG_DR_ADDRESS;
	DMA2_Stream1->M0AR = (uint32_t)&ov9655_ram_buffer;


	//DMA_Init(OV9655_DCMI_DMA_STREAM, &DMA_InitStructure);

	
}

void P_OV9655_InitDCMI()
{
	//DCMI_InitTypeDef DCMI_InitStructure;

	// Clock enable
	RCC->AHB2ENR |= RCC_AHB2Periph_DCMI;

	// DCMI init
	/*DCMI_InitStructure.DCMI_CaptureMode = DCMI_CaptureMode_SnapShot;
	DCMI_InitStructure.DCMI_SynchroMode = DCMI_SynchroMode_Hardware;
	DCMI_InitStructure.DCMI_PCKPolarity = DCMI_PCKPolarity_Falling;
	DCMI_InitStructure.DCMI_VSPolarity = DCMI_VSPolarity_High;
	DCMI_InitStructure.DCMI_HSPolarity = DCMI_HSPolarity_High;
	DCMI_InitStructure.DCMI_CaptureRate = DCMI_CaptureRate_All_Frame;
	DCMI_InitStructure.DCMI_ExtendedDataMode = DCMI_ExtendedDataMode_8b; 
	DCMI_Init(&DCMI_InitStructure);*/

	DCMI->CR &=	~((uint32_t)DCMI_CR_CM     | DCMI_CR_ESS   | DCMI_CR_PCKPOL |
				DCMI_CR_HSPOL  | DCMI_CR_VSPOL | DCMI_CR_FCRC_0 | 
				DCMI_CR_FCRC_1 | DCMI_CR_EDM_0 | DCMI_CR_EDM_1); 
	DCMI->CR =	DCMI_CaptureMode_SnapShot | DCMI_SynchroMode_Hardware | DCMI_PCKPolarity_Falling |
				DCMI_VSPolarity_High | DCMI_HSPolarity_High | DCMI_CaptureRate_All_Frame | DCMI_ExtendedDataMode_8b;
}

void P_OV9655_InitIO()
{
	palSetPadMode(GPIOA, 4, PAL_MODE_ALTERNATE(13));
	palSetPadMode(GPIOA, 6, PAL_MODE_ALTERNATE(13));
	palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(13));
	palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(13));
	palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(13));
	palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(13));
	palSetPadMode(GPIOC, 8, PAL_MODE_ALTERNATE(13));
	palSetPadMode(GPIOC, 9, PAL_MODE_ALTERNATE(13));
	palSetPadMode(GPIOE, 4, PAL_MODE_ALTERNATE(13));
	palSetPadMode(GPIOE, 5, PAL_MODE_ALTERNATE(13));
	palSetPadMode(GPIOE, 6, PAL_MODE_ALTERNATE(13));

	/*GPIO_InitTypeDef GPIO_InitStructure;

	//-----------------------------------------
	// Clock Enable Port-A, Port-B, Port-C and Port-E
	//-----------------------------------------
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);

	//-----------------------------------------
	// Init all pins at port A
	//-----------------------------------------
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_DCMI); // PA4=DCMI_HSYNC -> HSNYC
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_DCMI); // PA6=DCMI_PCLK  -> PCLK  

	// Structur fuer Port-A
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	// Config von Port-A
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//-----------------------------------------
	// Init all pins at port B
	//-----------------------------------------
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_DCMI); // PB6=DCMI_D5    -> D5
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_DCMI); // PB7=DCMI_VSYNC -> VSYNC

	// Structur fuer Port-B
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	// Config von Port-B
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//-----------------------------------------
	// Init all pins at port C
	//-----------------------------------------
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_DCMI); // PC6=DCMI_D0    -> D0
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_DCMI); // PC7=DCMI_D1    -> D1
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_DCMI); // PC8=DCMI_D2    -> D2
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_DCMI); // PC9=DCMI_D3    -> D3

	// Structur fuer Port-C
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	// Config von Port-C
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//-----------------------------------------
	// Init all pins at port E
	//-----------------------------------------
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_DCMI); // PE4=DCMI_D4    -> D4
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_DCMI); // PE5=DCMI_D6    -> D6
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_DCMI); // PE6=DCMI_D7    -> D7

	// Structur fuer Port-E
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	// Config von Port-E
	GPIO_Init(GPIOE, &GPIO_InitStructure);*/
}


