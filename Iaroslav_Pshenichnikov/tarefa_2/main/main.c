/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "driver/mcpwm_cap.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "hal/gpio_types.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "wheel.h"
#include "treeeyes_task.h"
#include "wheel_task.h"
#include "imu_task.h"
#include "ir_line_task.h"
#include "speed_ctrl_task.h"
#include <inttypes.h>

#define TREE_EYES_TASK
//#define IMU_TASK
#define WHEEL_CTRL_TASK
#define IR_LINE_CTRL_TASK
#define SPEED_CTRL_TASK

void app_main(void)
{

/* xEvents structure
   BIT  Descirption
   -------------------------
    0 - Most left IR sensor
    1 - Left IR sensor
    2 - Middle IR sensor
    3 - Right IR sensor
    4 - Most Right IR sensor
    5 - Stop flag (from treeeyes_task, sonar < 10 cm)
    rest - unused
*/    
EventGroupHandle_t xEvents = xEventGroupCreate();
TaskHandle_t wheel_handle = NULL;

static handlers_t multiple_handlers;

#ifdef IR_LINE_CTRL_TASK
    xTaskCreate(ir_line_ctrl,
                "ircontrol",
                configMINIMAL_STACK_SIZE*3,
                xEvents,
                5,
                NULL);
#endif

#ifdef TREE_EYES_TASK
    xTaskCreate(Treeeyes,
                "treeeyes",
                configMINIMAL_STACK_SIZE*3,
                xEvents,
                6,
                NULL);
#endif

#ifdef IMU_TASK
    xTaskCreate(IMU_Task,
                "imu",
                configMINIMAL_STACK_SIZE*3,
                NULL,
                5,
                NULL);
#endif

#ifdef WHEEL_CTRL_TASK
    xTaskCreate(wheel_ctrl,
                "wheel",
                configMINIMAL_STACK_SIZE*3,
                xEvents,
                5,
                &wheel_handle);
#endif

#ifdef SPEED_CTRL_TASK
    multiple_handlers.events = xEvents;
    multiple_handlers.wheel = wheel_handle;
    xTaskCreate(speed_ctrl,
                "speed",
                configMINIMAL_STACK_SIZE*3,
                &multiple_handlers,
                5,
                NULL);
#endif

}