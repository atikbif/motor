#include "rs485.h"

#include "stm32f0xx_conf.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

void RS485Task( void *pvParameters )
{
    portTickType xLastExecutionTime;
    xLastExecutionTime = xTaskGetTickCount();
    while(1)
    { // backgroung loop
        vTaskDelayUntil( &xLastExecutionTime, RS485_DELAY);
        //GPIOA->ODR ^= GPIO_Pin_11;
    }
}
