/*
* qamdec.c
*
* Created: 05.05.2020 16:38:25
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
#include "errorHandler.h"

#include "qaminit.h"
#include "qamdec.h"

#define BUFFER_SIZE NR_OF_SAMPLES * 4
#define AMPLITUDE_HALF_MAX 3072
#define AMPLITUDE_HALF_MIN 1024
#define AMPLITUDE_MAX 4096
#define AMPLITUDE_MIN 0

QueueHandle_t decoderQueue;
QueueHandle_t symbolQueue;

uint32_t decoderIdx;
uint32_t newDataIdx;

typedef enum {
	P_Idle1,
	P_Idle2,
	P_Sync,
	P_Length,
	P_Data,
	P_Checksum,
} State_Protocol;

uint16_t buffer[BUFFER_SIZE];

uint16_t toBufferIdx(uint32_t idx) {
	return idx % 128;
}

uint16_t uGetNextMaxIdx(uint16_t startIdx) {
	if (startIdx == 0) {
		startIdx = 1;
	}
	
	for (int i = startIdx; i < NR_OF_SAMPLES + startIdx && i < BUFFER_SIZE - 1; i++) {
		if (buffer[i - 1] < buffer[i] && buffer[i + 1] < buffer[i]) {
			return i;
		}
	}
	
	return 0;
}

uint16_t uGetNextMinIdx(uint16_t startIdx) {
	if (startIdx == 0) {
		startIdx = 1;
	}
	
	for (int i = startIdx; i < NR_OF_SAMPLES + startIdx && i < BUFFER_SIZE - 1; i++) {
		if (buffer[i - 1] > buffer[i] && buffer[i + 1] > buffer[i]) {
			return i;
		}
	}
	
	return 0;
}

uint16_t uGetNullPointIdx(uint16_t startIdx) {
	uint16_t nextMinIdx = uGetNextMinIdx(startIdx);
	uint16_t nextMaxIdx = uGetNextMaxIdx(startIdx);
	uint16_t nullPointIdx;
	
	if ((int16_t) nextMinIdx - (NR_OF_SAMPLES / 4) >= 0) {
		nullPointIdx  = nextMinIdx - (NR_OF_SAMPLES / 4);
	} else {
		nullPointIdx  = nextMaxIdx - (NR_OF_SAMPLES / 4);
	}
	
	uint16_t nullPointVal = buffer[nullPointIdx];
	
	while(1) {
		if (nullPointVal == buffer[nextMinIdx] || nullPointVal == buffer[nextMaxIdx] || // Phase adjustment detected
		   (nullPointVal * 2) - buffer[nextMinIdx] != buffer[nextMaxIdx]) {             // Amplitude adjustment detected
			nullPointIdx += NR_OF_SAMPLES / 2;
			nullPointVal = buffer[nullPointIdx];
			nextMinIdx = uGetNextMinIdx(nullPointIdx);
			nextMaxIdx = uGetNextMaxIdx(nullPointIdx);
		} else {
			// Neither do we have an amplitude nor a phase adjustment, so we found a correct nullpoint
			return nullPointIdx;
		}
		
		if (nullPointIdx + (NR_OF_SAMPLES / 2) > BUFFER_SIZE - 1) {
			nextMinIdx = uGetNextMinIdx(0);
			nextMaxIdx = uGetNextMaxIdx(0);
			
			if ((int16_t) nextMinIdx - (NR_OF_SAMPLES / 4) >= 0) {
				nullPointIdx  = nextMinIdx - (NR_OF_SAMPLES / 4);
			} else {
				nullPointIdx  = nextMaxIdx - (NR_OF_SAMPLES / 4);
			}
			
			nullPointVal = buffer[nullPointIdx];
		}
	}
}

// This task receives symbols from the Symbol-Queue (vTaskDetectSymbol),
// applies the defined protocol and sends data bytes into the data queue.
void vTaskProtocol(void *pvParameters) {
	(void) pvParameters;
	uint8_t queueItem;
	State_Protocol state = P_Idle1;
	
	for(;;) {
		while(uxQueueMessagesWaiting(symbolQueue) > 0) {
			if(xQueueReceive(symbolQueue, &queueItem, portMAX_DELAY) == pdTRUE) {
				// TODO: Process item
			}
		}
		
		vTaskDelay(2 / portTICK_RATE_MS);
	}
}

// This task is responsible for analyzing the
// current data held in buffer and finding symbols
// in it. For every symbol that is found, a new
// message in the Symbol-Queue is queued
void vTaskDetectSymbols(void *pvParameters) {
	(void) pvParameters;
	
	decoderIdx = 0;
	uint16_t nullPoint;
	uint16_t periodMinIdx;
	uint16_t periodMaxIdx;
	uint8_t symbol;
	
	for(;;) {
		// Don't get ahead of the new data pointer
		if (decoderIdx >= newDataIdx) {
			vTaskDelay(1);
			continue;
		}
		
		nullPoint = uGetNullPointIdx(toBufferIdx(decoderIdx));
		decoderIdx = nullPoint + NR_OF_SAMPLES;
		periodMinIdx = uGetNextMinIdx(nullPoint);
		periodMaxIdx = uGetNextMaxIdx(nullPoint);
		symbol = 0;
		
		// Check for amplitude adjustment
		if (buffer[periodMinIdx] == AMPLITUDE_HALF_MIN && buffer[periodMaxIdx] == AMPLITUDE_HALF_MAX) {
			symbol |= (1 << 1);
		}
		
		// Check for phase adjustment
		if (periodMinIdx < periodMaxIdx) {
			symbol |= (1 << 0);
		}
		
		xQueueSend(symbolQueue, &symbol, 0);
		vTaskDelay(1 / portTICK_RATE_MS);
	}
}

// This task is responsible for keeping the buffer-array up to date
// When new data is received, it writes it at newDataIdx from left
// to right until the end is reached. It then starts at 0 again
void vTaskFillBuffer(void *pvParameters) {
	(void) pvParameters;
	
	newDataIdx = 0;
	uint16_t bufferIdx;
	
	uint16_t bufferelement[NR_OF_SAMPLES];
	xEventGroupWaitBits(evDMAState, DMADECREADY, false, true, portMAX_DELAY);

	for(;;) {
		while(uxQueueMessagesWaiting(decoderQueue) > 0) {
			if(xQueueReceive(decoderQueue, &bufferelement[0], portMAX_DELAY) == pdTRUE) {
				for (int i = 0; i < NR_OF_SAMPLES; i++) {
					bufferIdx = toBufferIdx(newDataIdx);
					buffer[bufferIdx] = bufferelement[i];
					newDataIdx++;
				}
			}
		}
		vTaskDelay(2 / portTICK_RATE_MS);
	}
}

void vQuamDec()
{
	decoderQueue = xQueueCreate(4, NR_OF_SAMPLES * sizeof(int16_t));
	symbolQueue = xQueueCreate(5, sizeof(uint8_t));
	
	while(evDMAState == NULL) {
		vTaskDelay(3/portTICK_RATE_MS);
	}
		
	xTaskCreate(vTaskFillBuffer, "fillBuffer", configMINIMAL_STACK_SIZE + 20, NULL, 1, NULL);
	//xTaskCreate(vTaskDetectSymbols, "detectSymbols", configMINIMAL_STACK_SIZE + 10, NULL, 1, NULL);
	//xTaskCreate(vTaskProtocol, "protocol", configMINIMAL_STACK_SIZE + 10, NULL, 1, NULL);
}

void fillDecoderQueue(uint16_t buffer[NR_OF_SAMPLES])
{
	BaseType_t xTaskWokenByReceive = pdFALSE;

	xQueueSendFromISR( decoderQueue, &buffer[0], &xTaskWokenByReceive );
}

ISR(DMA_CH2_vect)
{
	DMA.CH2.CTRLB|=0x10;

	fillDecoderQueue( &adcBuffer0[0] );
}

ISR(DMA_CH3_vect)
{
	DMA.CH3.CTRLB |= 0x10;

	fillDecoderQueue( &adcBuffer1[0] );
}