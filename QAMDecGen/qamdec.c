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

#define BUFFER_SIZE NR_OF_SAMPLES * 12
#define TOLERANCE 60

QueueHandle_t decoderQueue;
QueueHandle_t symbolQueue;

uint32_t decoderIdx;
uint32_t newDataIdx;
uint16_t npMedian;

typedef enum {
	P_Idle1,
	P_Idle2,
	P_Length,
	P_Data,
	P_Checksum
} State_Protocol;

typedef struct {
	uint16_t len;
	char *data;
} DataMessage;

uint16_t buffer[BUFFER_SIZE];

uint16_t toBufferIdx(uint32_t idx) {
	return idx % 384;
}

uint16_t inTolerance(uint16_t val, uint16_t target) {
	if (target - TOLERANCE <= val && val <= target + TOLERANCE) {
		return 1;
	} else {
		return 0;
	}
}

uint16_t uGetMaxIdxInSample(uint16_t startIdx) {
	for (int i = startIdx; i < startIdx + NR_OF_SAMPLES; i++) {
		if (buffer[i - 1] < buffer[i] && buffer[i + 1] < buffer[i]) {
			return i;
		}
	}
}

uint16_t uGetMinIdxInSample(uint16_t startIdx) {
	for (int i = startIdx; i < startIdx + NR_OF_SAMPLES - 1; i++) {
		if (buffer[i - 1] > buffer[i] && buffer[i + 1] > buffer[i]) {
			return i;
		}
		
		if (buffer[i - 1] > buffer[i] && buffer[i + 1] == buffer[i]) {
			return i + 1;
		}
	}
}

uint16_t uGetMinMaxInSample(uint16_t startIdx) {
	uint16_t min = uGetMinIdxInSample(startIdx);
	uint16_t max = uGetMaxIdxInSample(startIdx);
	
	if (min < max) {
		return min;
	} else {
		return max;
	}
}
uint16_t b, c, d, e, f;
int32_t uGetNullPointIdx(uint32_t startIdx) {	
	uint16_t bufferIdx = toBufferIdx(startIdx);
	uint16_t minMaxIdx = uGetMinMaxInSample(bufferIdx + 5);
	
	if (minMaxIdx < NR_OF_SAMPLES / 4) {
		return -1;
	}
	
	uint16_t nextMinMaxIdx = uGetMinMaxInSample(minMaxIdx + 5);
	uint16_t npIdx = minMaxIdx - (NR_OF_SAMPLES / 4);
	
	// Check if the value of the nullpoint is near the average, and if
	// not, check if the previous or next one is, because it could be
	// that the min or max are shifted by 1
	if (!inTolerance(buffer[npIdx], npMedian)) {
		if (inTolerance(buffer[npIdx - 1], npMedian)) {
			npIdx--;
		} else if (inTolerance(buffer[npIdx + 1], npMedian)) {
			npIdx++;
		}
		minMaxIdx = npIdx + (NR_OF_SAMPLES / 4);
		nextMinMaxIdx = npIdx + (NR_OF_SAMPLES / 4 * 3);
	}
	
	// Check for phase or amplitude adjustment in the current sample; This would mean we are
	// in the middle and not at the start of a sample, so move the start half a period forward
	if (inTolerance(buffer[npIdx], buffer[minMaxIdx]) || inTolerance(buffer[npIdx], buffer[nextMinMaxIdx]) || !inTolerance((buffer[npIdx] * 2) - buffer[minMaxIdx], buffer[nextMinMaxIdx])) {
		b = 1;
		c = !inTolerance((buffer[npIdx] * 2) - buffer[minMaxIdx], buffer[nextMinMaxIdx]);
		d = buffer[npIdx] * 2;
		e = buffer[minMaxIdx];
		f = buffer[nextMinMaxIdx];
		npIdx += (NR_OF_SAMPLES / 2);
	} else {
		b = 0;
	}
	
	return (384 * (startIdx / 384)) + npIdx;
}

// This task receives symbols from the Symbol-Queue (vTaskDetectSymbol),
// applies the defined protocol and sends data bytes into the data queue.
char data[255];
char uartData[271];
void vTaskProtocol(void *pvParameters) {
	(void) pvParameters;
	uint16_t queueItem;
	uint16_t idx = 0;
	uint16_t dataLength;
	uint16_t checksum;
	uint16_t rxChecksum;
	State_Protocol state = P_Idle1;
	
	for(;;) {
		while(uxQueueMessagesWaiting(symbolQueue) > 0) {
			if(xQueueReceive(symbolQueue, &queueItem, portMAX_DELAY) == pdTRUE) {				
				switch (state) {
					case P_Idle1:
						if (queueItem == 2) {
							state = P_Idle2;
						} else if (queueItem == 3) {
							// We only have one sync symbol, so as soon as
							// we receive it, we can directly go to the
							// length state
							state = P_Length;
							dataLength = 0;
							idx = 0;
						}
						
						break;
						
					case P_Idle2:
						if (queueItem == 1) {
							state = P_Idle1;
						} else if (queueItem == 3) {
							// We only have one sync symbol, so as soon as
							// we receive it, we can directly go to the
							// length state
							state = P_Length;
							dataLength = 0;
							idx = 0;
						}
						
						break;
						
					case P_Length:
						// The length is 4 symbols or 1 byte
						dataLength |= (queueItem << (6 - (2 * idx)));
						
						if (idx == 3) {
							state = P_Data;
							idx = 0;
							for (int i = 0; i < 256; i++) {
								data[i] = 0;
							}
							checksum = dataLength;
						} else {
							idx++;
						}
						
						break;
						
					case P_Data:
						// Int division gives us the correct index
						data[idx / 4] |= (queueItem << (6 - (2 * (idx % 4))));
						checksum += queueItem;
						
						idx++;
						
						if (idx / 4 == dataLength) {
							state = P_Checksum;
							rxChecksum = 0;
							idx = 0;
						}
					
						break;
						
					case P_Checksum:
						// The checksum is 16 bit long, so 8 symbols will
						// build the checksum
						idx++;
						rxChecksum |= (queueItem << (16 - (idx * 2)));
						
						if (idx == 8) {
							// Verify that the sent and calculated checksums match
							if (rxChecksum == checksum) {
								sprintf(uartData, "Received: %s\r\n", data);
							/*} else {
								sprintf(uartData, "Checksum Error, calculated %d, received %d\r\n", checksum, rxChecksum);
							}*/
							
							for (int i = 0; i < 271; i++) {
								if (uartData[i] == 0) {
									break;
								}
								
								while(!(USARTC0.STATUS & USART_DREIF_bm));
								USARTC0.DATA = uartData[i];
							}}
							
							state = P_Idle1;
						}
						
						break;
				}
			}
		}

		vTaskDelay(5 / portTICK_RATE_MS);
	}
}

// This task is responsible for analyzing the
// current data held in buffer and finding symbols
// in it. For every symbol that is found, a new
// message in the Symbol-Queue is queued
uint16_t nps[250];
uint16_t npsx = 0;
void vTaskDetectSymbols(void *pvParameters) {
	(void) pvParameters;
	
	decoderIdx = 0;
	npMedian = 1190;
	uint16_t symbol;
	int32_t nullPointIdx;
	uint16_t bufferNullPointIdx;
	uint16_t periodMinIdx;
	uint16_t periodMaxIdx;
	uint16_t maxAmplitude100 = 0;
	uint16_t maxAmplitude50 = 0;
	uint16_t minAmplitude50 = 0;
	uint16_t diff = 0;
	
	maxAmplitude100 = 2192;
	
	for(;;) {
		while (decoderIdx + NR_OF_SAMPLES < newDataIdx) {
			nullPointIdx = uGetNullPointIdx(decoderIdx);
			if (nullPointIdx == -1) {
				decoderIdx += NR_OF_SAMPLES;
				continue;
			} else {
				decoderIdx = nullPointIdx + NR_OF_SAMPLES;
			}
			
			bufferNullPointIdx = toBufferIdx(nullPointIdx);
			nps[npsx] = bufferNullPointIdx;
			npsx++;
			if (npsx == 250) {
				npsx = 0;
			}
			
			if (buffer[bufferNullPointIdx + 8] < buffer[bufferNullPointIdx + 24]) {
				periodMinIdx = bufferNullPointIdx + 8;
				periodMaxIdx = bufferNullPointIdx + 24;
			} else {
				periodMinIdx = bufferNullPointIdx + 24;
				periodMaxIdx = bufferNullPointIdx + 8;
			}

			diff = (maxAmplitude100 - buffer[bufferNullPointIdx]) / 2;
			maxAmplitude50 = buffer[bufferNullPointIdx] + diff;
			minAmplitude50 = buffer[bufferNullPointIdx] - diff;
			
			symbol = 0;
			// Check for amplitude adjustment
			if (inTolerance(buffer[periodMinIdx], minAmplitude50) && inTolerance(buffer[periodMaxIdx], maxAmplitude50)) {
				symbol |= (1 << 1);
			}
			
			// Check for phase adjustment
			if (periodMinIdx < periodMaxIdx) {
				symbol |= (1 << 0);
			}
			
			xQueueSend(symbolQueue, &symbol, 0); // 1,2,1,3, 0,0,1,1, 1,0,2,0, 1,2,1,1, 1,2,3,0, 1,2,3,0, 1,2,3,3, 0,0,0,0, 0,2,0,2, 1,2,1,2
		} 
		
		vTaskDelay(5 / portTICK_RATE_MS);
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
		vTaskDelay(1 / portTICK_RATE_MS);
	}
}

void vInitUart(){
	// Baudrate 9600
	USARTC0.BAUDCTRLA = 0xD0 & 0xFF;
	USARTC0.BAUDCTRLB |= ((0 & 0x0F) << 0x04);
	USARTC0.CTRLA = USART_RXCINTLVL_LO_gc;
	USARTC0.STATUS |= USART_RXCIF_bm;
	USARTC0.CTRLB = USART_TXEN_bm;
	USARTC0.CTRLC = USART_CHSIZE_8BIT_gc;
	USARTC0.CTRLC &= ~(USART_PMODE0_bm | USART_PMODE1_bm | USART_SBMODE_bm);
	PORTC.DIR = 0x08;
	
	char* data = "Decoder-UART Init\n";
	while(*data)
	{
		while(!(USARTC0.STATUS & USART_DREIF_bm));
		USARTC0.DATA = *data++;
	}
}

void vQuamDec()
{
	decoderQueue = xQueueCreate(20, NR_OF_SAMPLES * sizeof(int16_t));
	symbolQueue = xQueueCreate(250, sizeof(uint16_t));
	
	vInitUart();
	
	/*while(evDMAState == NULL) {
		vTaskDelay(3/portTICK_RATE_MS);
	}*/
		
	xTaskCreate(vTaskFillBuffer, "fillBuffer", configMINIMAL_STACK_SIZE + 150, NULL, 3, NULL);
	xTaskCreate(vTaskDetectSymbols, "detectSymbols", configMINIMAL_STACK_SIZE + 150, NULL, 2, NULL);
	xTaskCreate(vTaskProtocol, "protocol", configMINIMAL_STACK_SIZE + 150, NULL, 2, NULL);
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