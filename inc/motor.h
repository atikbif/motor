#ifndef MOTOR_H_INCLUDED
#define MOTOR_H_INCLUDED

#define Motor_TASK_PRIORITY                  ( tskIDLE_PRIORITY + 1 )
#define Motor_DELAY						( ( portTickType ) 1 / portTICK_RATE_MS )

void MotorTask( void *pvParameters );

#endif /* MOTOR_H_INCLUDED */
