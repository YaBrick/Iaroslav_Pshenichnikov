#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "wheel.h" 
#include "speed_ctrl_task.h"
#include "pid_ctrl.h"

const static char *TAG = "speed_ctrl";

speed_t speed_estimator(void){
    int pL = 0, pR = 0;
    const int estimated_increment = 3;
    wheel_GetEndoderPulses(&pL, &pR);

    speed_t s = {
        .L = pL * estimated_increment,
        .R = pR * estimated_increment,
    };
    return s;
}

pid_ctrl_config_t pid_config = {
    .init_param = {
    .kp = 20,
    .ki = 1,
    .kd = 10,
    .max_output = 350,
    .min_output = 0,
    .max_integral = 100,
    .min_integral = 5,
    .cal_type = PID_CAL_TYPE_INCREMENTAL
    }
}

portTASK_FUNCTION(speed_ctrl, args)
{
    handlers_t *ctx = (handlers_t *)args;
    EventGroupHandle_t evt          = ctx->events;
    TaskHandle_t       wheel_handle = ctx->wheel;
    speed_t speed;
    
    int eventBits;

    volatile bool L_big, L_med; 
    volatile bool R_big, R_med; 
    volatile int16_t L_mult, R_mult;
    voltatile int16_t L_pid, R_pid; //values after applying PID computing
  
    volatile bool middle, error_state;

    pid_ctrl_block_handle_t L_pid_block;
    pid_ctrl_block_handle_t R_pid_block;
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &L_pid_block));    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid));
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &R_pid_block));

    while(1){
        
      eventBits = xEventGroupGetBits(evt);
      speed = speed_estimator();
      
              /* Invert: bit = 1 means line detected under that sensor */
      uint8_t sens = ~eventBits & 0x3F;

      bool L_big   = (sens & BIT0) != 0;   /* very-left  */
      bool L_med   = (sens & BIT1) != 0;   /* left       */
      bool middle  = (sens & BIT2) != 0;   /* middle     */
      bool R_med   = (sens & BIT3) != 0;   /* right      */
      bool R_big   = (sens & BIT4) != 0;   /* very-right */

      bool error_state = (sens == 0x00);

      bool sonar_stop = (sens & BIT5) != 0;

        L_mult = 5 *  L_big
                +  8 * (L_med  & !L_big)
                +  7 * (middle & !L_big  & !L_med)
                +  6 * (!L_big & !L_med  & !middle & R_med)
                +  7 *  error_state
                -  12 * (R_big);
                      
        R_mult = 5 *  R_big
                +  8 * (R_med  & !R_big)
                +  7 * (middle & !R_big  & !R_med)
                +  6 * (!R_big & !R_med  & !middle & L_med)
                +  7 *  error_state
                -  12 * (L_big);

        pid_compute(L_pid_block, R_mult, &L_pid);
        pid_compute(L_pid_block, R_mult, &L_pid);

        uint16_t L_pkt = (uint16_t)(L_pid * (uint16_t)sonar_stop + 1024);          // + 1024 just to be able pass "signed" value as an unsigned
        uint16_t R_pkt = (uint16_t)(R_pid * (uint16_t)sonar_stop + 1024);
        //xTaskNotifyIndexed(wheel_handle, 0, (uint32_t)L_pkt | ((uint32_t)R_pkt << 16), eSetValueWithOverwrite);

        //ESP_LOGI(TAG, "Left encoder: %d\tRight encoder: %d\r\n", pL, pR);
        ESP_LOGI(TAG, "L = %d R = %d", speed.L, speed.R);
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
	
}