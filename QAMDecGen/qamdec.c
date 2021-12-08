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
#define TOLERANCE 50

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
	P_Error
} State_Protocol;

uint16_t buffer[BUFFER_SIZE];

uint8_t toBufferIdx(uint32_t idx) {
	return idx % 128;
}

uint8_t inTolerance(uint16_t val, uint16_t target) {
	if (target - TOLERANCE < val && val < target + TOLERANCE) {
		return 1;
	} else {
		return 0;
	}
}

uint16_t uGetMaxInBuffer() {
	uint16_t idx = 0;
	for (int i = 0; i < BUFFER_SIZE; i++) {
		if (buffer[i] > buffer[idx]) {
			idx = i;
		}
	}
	
	return buffer[idx];
}

uint8_t uGetMaxIdxInSample(uint8_t startIdx) {
	for (int i = startIdx; i < startIdx + NR_OF_SAMPLES; i++) {
		if (buffer[i - 1] < buffer[i] && buffer[i + 1] < buffer[i]) {
			if (buffer[i - 1] < buffer[i] - 5 || buffer[i + 1] < buffer[i] - 5) {
				return i;
			}
		}
	}
}

uint8_t uGetMinIdxInSample(uint8_t startIdx) {
	for (int i = startIdx; i < startIdx + NR_OF_SAMPLES; i++) {
		if (buffer[i - 1] > buffer[i] && buffer[i + 1] > buffer[i]) {
			if (buffer[i - 1] > buffer[i] + 5 || buffer[i + 1] > buffer[i]) {
				return i;
			}
		}
	}
}

uint8_t uGetMinMaxInSample(uint8_t startIdx) {
	for (int i = startIdx; i < startIdx + NR_OF_SAMPLES; i++) {
		if ((buffer[i - 1] < buffer[i] && buffer[i + 1] < buffer[i]) || (buffer[i - 1] > buffer[i] && buffer[i + 1] > buffer[i])) {
			return i;
		}
	}
}

int8_t uGetNullPointIdx(uint8_t startIdx) {
	uint8_t nextMinMaxIdx;
	uint8_t nullPointIdx;
	uint16_t nullPointVal;
	uint8_t nextMinIdx;
	uint8_t nextMaxIdx;
	uint8_t i;
	
	// First we get the next min or max within a full sin wave
	// and within bounds of the array
	do {
		startIdx++;
		nextMinMaxIdx = uGetMinMaxInSample(startIdx);
	} while(nextMinMaxIdx < NR_OF_SAMPLES / 4);
	
	do {
		// The nullpoint is 1/4 of a curve before the min/max
		nullPointIdx = nextMinMaxIdx - (NR_OF_SAMPLES / 4);
		
		// Get a min and a max that are at least 2 samples
		// away from each other, as everything else doesn't
		// really make sense
		i = 1;
		do {
			nextMinIdx = uGetMinIdxInSample(nullPointIdx + i);
			i++;
		} while (nextMinIdx - nullPointIdx < 2);
		
		i = 1;
		do {
			nextMaxIdx = uGetMaxIdxInSample(nullPointIdx + i);
			i++;
		} while (nextMaxIdx - nullPointIdx < 2);
		
		// Check if the nullpoint index matches up with the min and max index.
		// It could be that there are more or less samples than we expect between
		// the min/max and the nullpoint, so we need to adjust it if necessary
		if (nullPointIdx != nextMinIdx - (NR_OF_SAMPLES / 4) || nullPointIdx != nextMaxIdx - (NR_OF_SAMPLES / 4 * 3)) {
			if (nullPointIdx + 1 == nextMaxIdx - (NR_OF_SAMPLES / 4 * 3)) {
				nullPointIdx++;
			} else if (nullPointIdx - 1 == nextMaxIdx - (NR_OF_SAMPLES / 4 * 3)) {
				nullPointIdx--;
			}
		}
	
		nullPointVal = buffer[nullPointIdx];
		
		// This if-statement should never be true as we have a buffer that can hold
		// 4 full sin-waves, so we should at least find one valid nullpoint
		if (nullPointIdx + (NR_OF_SAMPLES / 2) > BUFFER_SIZE - 1) {
			return -1;
		} else {
			// Advance the search for a valid point
			nullPointIdx += (NR_OF_SAMPLES / 2);
			nextMinMaxIdx = uGetMinMaxInSample(nullPointIdx);
		}
	} while (inTolerance(nullPointVal, buffer[nextMinIdx]) ||							 // Phase adjustment detected
	         inTolerance(nullPointVal, buffer[nextMaxIdx]) ||							 // Phase adjustment detected
	         !inTolerance((nullPointVal * 2) - buffer[nextMinIdx], buffer[nextMaxIdx])); // Amplitude adjustment detected
			 
	// Subtract NR_OF_SAMPLES / 2 because we added it in the do-while but didn't
	// need to add it as the nullpoint matched
	return nullPointIdx - (NR_OF_SAMPLES / 2);
}

// This task receives symbols from the Symbol-Queue (vTaskDetectSymbol),
// applies the defined protocol and sends data bytes into the data queue.
void vTaskProtocol(void *pvParameters) {
	(void) pvParameters;
	uint8_t queueItem;
	uint8_t idx;
	uint8_t dataLength;
	State_Protocol state = P_Idle1;
	
	for(;;) {
		vDisplayClear();
		vDisplayWriteStringAtPos(0,0,"Decoder");
		vDisplayWriteStringAtPos(1,0,"Depth: 1");
		while(uxQueueMessagesWaiting(symbolQueue) > 0) {
			if(xQueueReceive(symbolQueue, &queueItem, portMAX_DELAY) == pdTRUE) {
				switch (state) {
					case P_Idle1:
						// Idle 1 has 2 possible targets:
						// - Idle 2
						// - Sync
						if (queueItem == 2) {
							state = P_Idle2;
						} else if (queueItem == 3) {
							state = P_Sync;
						} else {
							state = P_Error;
						}
						
						break;
						
					case P_Idle2:
						// Idle 2 has 2 possible targets:
						// - Idle 1
						// - Sync
						if (queueItem == 1) {
							state = P_Idle1;
						} else if (queueItem == 3) {
							state = P_Sync;
						} else {
							state = P_Error;
						}
						
						break;
						
					case P_Sync:
						// Sync is only 1 symbol, so immediately after,
						// we can enter the Length-State
						state = P_Length;
						idx = 0;
					
						break;
						
					case P_Length:
						// Length has 4 symbols, so after 4,
						// we can enter the Data-State
						if (idx == 3) {
							state = P_Data;
							idx = 0;
						} else {
							dataLength |= (queueItem << 6 - (2 * idx));
							idx++;
						}
					
						break;
						
					case P_Data:
						// We already know the length, so we just
						// sit in this state for as long as Length is
						// and glue the symbols to bytes
						idx++;
						if (idx == dataLength) {
							state = P_Checksum;
						}
					
						break;
						
					case P_Checksum:
						
						
						break;
						
					default:
						// If we get in here, there was an unrecognized state
						// This should never happen, but for now, we just show
						// it on the display
						vDisplayWriteStringAtPos(1,0,"Unknown State");
						break;
				}
				vTaskDelay(1 / portTICK_RATE_MS);
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
	int8_t nullPointIdx;
	uint8_t periodMinIdx;
	uint16_t periodMaxIdx;
	uint8_t symbol;
	uint16_t maxAmplitude100 = 0;
	uint16_t maxAmplitude50 = 0;
	uint16_t minAmplitude50 = 0;
	
	for(;;) {
		// Don't get ahead of the new data pointer
		while (decoderIdx + NR_OF_SAMPLES >= newDataIdx) {
			vTaskDelay(1 / portTICK_RATE_MS);
		}
		
		nullPointIdx = uGetNullPointIdx(toBufferIdx(decoderIdx));
		// If we didn't find a valid one, we get -1 returned
		// so we start at the beginning of the buffer, just behind
		// newDataIdx and continue with the next iteration
		if (nullPointIdx == -1) {
			decoderIdx = newDataIdx - toBufferIdx(newDataIdx);
			vTaskDelay(1 / portTICK_RATE_MS);
			continue;
		} else {
			decoderIdx = nullPointIdx + NR_OF_SAMPLES;
		}
		
		periodMinIdx = uGetMinIdxInSample(nullPointIdx + 1);
		periodMaxIdx = uGetMaxIdxInSample(nullPointIdx + 1); 
		symbol = 0;
		
		// Continously check and adjust min and max amplitude
		maxAmplitude100 = uGetMaxInBuffer();
		uint16_t diff = (maxAmplitude100 - buffer[nullPointIdx]) / 2;
		maxAmplitude50 = buffer[nullPointIdx] + diff;
		minAmplitude50 = buffer[nullPointIdx] - diff;
		
		// Check for amplitude adjustment
		if (inTolerance(buffer[periodMinIdx], minAmplitude50) && inTolerance(buffer[periodMaxIdx], maxAmplitude50)) {
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
	uint8_t bufferIdx;
	
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
		
	xTaskCreate(vTaskFillBuffer, "fillBuffer", configMINIMAL_STACK_SIZE + 100, NULL, 1, NULL);
	xTaskCreate(vTaskDetectSymbols, "detectSymbols", configMINIMAL_STACK_SIZE + 200, NULL, 1, NULL);
	xTaskCreate(vTaskProtocol, "protocol", configMINIMAL_STACK_SIZE + 200, NULL, 1, NULL);
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