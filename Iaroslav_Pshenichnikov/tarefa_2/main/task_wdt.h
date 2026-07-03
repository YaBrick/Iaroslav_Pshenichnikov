#include "wheel.h"

void esp_task_wdt_isr_user_handler(void){
    wheel_Init(); //сомнительно, но попробуй вдруг сработает из isr
    wheel_GoForward();
    right_wheel_GoBackward();
	wheel_SetVel(100, 0);
}