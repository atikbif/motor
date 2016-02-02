#ifndef RS485_H_INCLUDED
#define RS485_H_INCLUDED

#define RS485_TASK_PRIORITY                  ( tskIDLE_PRIORITY + 1 )
#define RS485_DELAY						( ( portTickType ) 1 / portTICK_RATE_MS )

void RS485Task( void *pvParameters );

#endif /* RS485_H_INCLUDED */
