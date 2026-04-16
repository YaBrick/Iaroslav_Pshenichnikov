#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
//#include "led_strip.h"
#include "sdkconfig.h"
#include "soc/gpio_reg.h"
#include "freertos/event_groups.h"
#include "mission_control.h"

EventGroupHandle_t xEvents;

static const char *TAG = "mission_control";


portTASK_FUNCTION(msctrl_task, arg) 
{
    //vTaskDelay( 2000 / portTICK_PERIOD_MS);

    vTaskDelete(NULL);
}
