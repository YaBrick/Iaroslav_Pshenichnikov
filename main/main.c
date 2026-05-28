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
#include "mission_control.h"
#include <inttypes.h>

#define TREE_EYES_TASK
//#define IMU_TASK
//#define WHEEL_CTRL_TASK
#define IR_LINE_CTRL_TASK
//define MISSION_CTRL_TASK


void app_main(void)
{
//xEvents = xEventGroupCreate();

TaskHandle_t ir_line_handle = NULL;

#ifdef IR_LINE_CTRL_TASK
    xTaskCreate(ir_line_ctrl,
                "ircontrol",
                configMINIMAL_STACK_SIZE*3,
                NULL,
                5,
                &ir_line_handle);
#endif

#ifdef TREE_EYES_TASK
    xTaskCreate(Treeeyes,
                "treeeyes",
                configMINIMAL_STACK_SIZE*3,
                ir_line_handle,
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
                NULL,
                5,
                NULL);
#endif


#ifdef MISSION_CTRL_TASK

xTaskCreate(msctrl_task,
                "mscontrol",
                configMINIMAL_STACK_SIZE*3,
                NULL,
                5,
                NULL);
#endif

}