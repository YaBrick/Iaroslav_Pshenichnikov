#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define IR_VERY_VERY_LEFT_FLAG  BIT0
#define IR_VERY_LEFT_FLAG  BIT1
#define IR_LEFT_FLAG  BIT2
#define IR_LEFT_MIDDLE_FLAG  BIT3
#define IR_MIDDLE_FLAG  BIT4
#define IR_RIGHT_FLAG  BIT5
#define IR_RIGHT_MIDDLE_FLAG  BIT6
#define IR_VERY_RIGHT_FLAG  BIT7
#define IR_VERY_VERY_RIGHT_FLAG  BIT8
#define STOP_FLAG BIT9


extern EventGroupHandle_t xEvents;

portTASK_FUNCTION(msctrl_task, arg);

	