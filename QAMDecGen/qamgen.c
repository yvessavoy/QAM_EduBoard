/*
* qamgen.c
*
* Created: 05.05.2020 16:24:59
*  Author: Chaos
*/ 
#include "avr_compiler.h"
#include "pmic_driver.h"
#include "TC_driver.h"
#include "clksys_driver.h"
#include "sleepConfig.h"
#include "port_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"
#include "stack_macros.h"

#include "mem_check.h"

#include "qaminit.h"
#include "qamgen.h"


const int16_t sinLookup100[NR_OF_SAMPLES*2] = {0x0,0x18F,0x30F,0x471,0x5A7,0x6A6,0x763,0x7D8,
	0x7FF,0x7D8,0x763,0x6A6,0x5A7,0x471,0x30F,0x18F,
	0x0,0xFE71,0xFCF1,0xFB8F,0xFA59,0xF95A,0xF89D,0xF828,
	0xF801,0xF828,0xF89D,0xF95A,0xFA59,0xFB8F,0xFCF1,0xFE71,
	0x0,0x18F,0x30F,0x471,0x5A7,0x6A6,0x763,0x7D8,
	0x7FF,0x7D8,0x763,0x6A6,0x5A7,0x471,0x30F,0x18F,
	0x0,0xFE71,0xFCF1,0xFB8F,0xFA59,0xF95A,0xF89D,0xF828,
0xF801,0xF828,0xF89D,0xF95A,0xFA59,0xFB8F,0xFCF1,0xFE71};

const int16_t sinLookup50[NR_OF_SAMPLES*2] = {0x0,0xC8,0x187,0x238,0x2D3,0x353,0x3B1,0x3EB,
	0x3FF,0x3EB,0x3B1,0x353,0x2D3,0x238,0x187,0xC8,
	0x0,0xFF38,0xFE79,0xFDC8,0xFD2D,0xFCAD,0xFC4F,0xFC15,
	0xFC01,0xFC15,0xFC4F,0xFCAD,0xFD2D,0xFDC8,0xFE79,0xFF38,
	0x0,0xC8,0x187,0x238,0x2D3,0x353,0x3B1,0x3EB,
	0x3FF,0x3EB,0x3B1,0x353,0x2D3,0x238,0x187,0xC8,
	0x0,0xFF38,0xFE79,0xFDC8,0xFD2D,0xFCAD,0xFC4F,0xFC15,
0xFC01,0xFC15,0xFC4F,0xFCAD,0xFD2D,0xFDC8,0xFE79,0xFF38,};

#define SENDBUFFER_SIZE 40
//													5		H	     E		  L		  L		   O      Checksumme = 34
uint8_t sendbuffer[SENDBUFFER_SIZE] = {1,2,1,3, 0,0,1,1, 1,0,2,0, 1,2,1,1, 1,2,3,0, 1,2,3,0, 1,2,3,3, 0,0,0,0, 0,2,0,2, 1,2,1,2};
//#define SENDBUFFER_SIZE 68
//uint8_t sendbuffer[SENDBUFFER_SIZE] = {1,2,1,3, 0,0,2,3, 1,1,0,1, 1,0,0,1, 1,0,3,1, 0,2,0,0, 1,0,1,0, 1,2,1,1, 1,2,0,3, 1,2,3,3, 1,2,1,0, 1,2,1,1, 1,3,0,2, 0,0,0,0, 0,3,3,0, 1,2,1,2, 1,2,1,2};

void vQuamGen(void *pvParameters) {
	(void) pvParameters;
	
	while(evDMAState == NULL) {
		vTaskDelay(3/portTICK_RATE_MS);
	}
	xEventGroupWaitBits(evDMAState, DMAGENREADY, false, true, portMAX_DELAY);
	for(;;) {
		vTaskDelay(10/portTICK_RATE_MS);
	}
}

int pSendbuffer = 0;
void fillBuffer(uint16_t buffer[NR_OF_SAMPLES]) {	
	for(int i = 0; i < NR_OF_SAMPLES;i++) {
		switch(sendbuffer[pSendbuffer]) {
			case 0:
			buffer[i] = 0x800 + (sinLookup100[i]);
			break;
			case 1:
			buffer[i] = 0x800 + (sinLookup100[i+16]);
			break;
			case 2:
			buffer[i] = 0x800 + (sinLookup50[i]);
			break;
			case 3:
			buffer[i] = 0x800 + (sinLookup50[i+16]);
			break;
		}
	}
	if(pSendbuffer < SENDBUFFER_SIZE-1) {
		pSendbuffer++;
	} else {
		pSendbuffer = 0;
	}
}

ISR(DMA_CH0_vect)
{
	//static signed BaseType_t test;
	
	DMA.CH0.CTRLB|=0x10;
	fillBuffer(&dacBuffer0[0]);
}

ISR(DMA_CH1_vect)
{
	DMA.CH1.CTRLB|=0x10;
	fillBuffer(&dacBuffer1[0]);
}