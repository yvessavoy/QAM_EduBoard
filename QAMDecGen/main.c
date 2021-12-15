/*
 * QAMDecGen.c
 *
 * Created: 20.03.2018 18:32:07
 * Author : chaos
 */ 

//#include <avr/io.h>
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
#include "stack_macros.h"

#include "mem_check.h"

#include "init.h"
#include "utils.h"
#include "errorHandler.h"
#include "NHD0420Driver.h"

#include "qaminit.h"
#include "qamgen.h"
#include "qamdec.h"


extern void vApplicationIdleHook( void );
void vLedBlink(void *pvParameters);

TaskHandle_t ledTask;

void vApplicationIdleHook( void )
{	
	
}

int main(void)
{
    resetReason_t reason = getResetReason();

	vInitClock();
	vInitDisplay();
	
	initDAC();
	initDACTimer();
	initGenDMA();
	initADC();
	initADCTimer();
	initDecDMA();
	
	xTaskCreate(vQuamGen, "quamGen", configMINIMAL_STACK_SIZE + 500, NULL, 4, NULL);
	vQuamDec();

	vDisplayClear();
	vDisplayWriteStringAtPos(0,0,"Newest Message:");
	
	vTaskStartScheduler();
	return 0;
}
