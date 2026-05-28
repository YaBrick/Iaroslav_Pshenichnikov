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
#include "mission_control.h"



const static char *TAG = "wheels";

portTASK_FUNCTION(wheel_ctrl, arg)
{
	wheel_Init();
	wheel_SetVel(100, 100);

	uint32_t power_left_wheel, power_right_wheel; 

  EventBits_t eventBits = xEventGroupGetBits(xEvents);

  bool L_big, L_med; 
  uint8_t L_speed_increment;

  bool R_big, R_med; 
  uint8_t R_speed_increment;
  
  bool middle, error_state;
  uint16_t common_speed_increment = 27;

	wheel_GoForward();
          
          
          //wheel_GetPower(&power_left_wheel, &power_right_wheel);
          //printf("Left ADC: %" PRIu32 "; \t Right ADC: %" PRIu32 ".\n", power_left_wheel, power_right_wheel);

		  //printf("Left ADC: %d\n", adc_left_raw[1][0]);
    while(1){
      /*
        eventBits = xEventGroupWaitBits(
        xEvents,
        IR_VERY_VERY_LEFT_FLAG | IR_VERY_LEFT_FLAG | IR_VERY_RIGHT_FLAG | IR_VERY_VERY_RIGHT_FLAG | STOP_FLAG,
        pdTRUE,   // clear bits after receiving
        pdFALSE,  // wait for any (OR)
        portMAX_DELAY
    ); */
      eventBits = xEventGroupGetBits(xEvents);
      
      eventBits = ~eventBits & 0x1F;

      L_big   = (eventBits & BIT0) != 0;
      L_med   = (eventBits & BIT1) != 0;
     

      R_big   = (eventBits & BIT4) != 0;
      R_med   = (eventBits & BIT3) != 0;

      middle = (eventBits & BIT2) != 0;
      
      error_state = (eventBits == 0x0);

      L_speed_increment = 11 * L_big + 8 *(L_med * !(L_big)) + 7 * middle * (!(L_big)) * (!(L_med)) + 6 * (!(L_big) * !(L_med) * !(middle) * (R_med)) + 6 * error_state;
      R_speed_increment = 11 * R_big + 8 *(R_med * !(R_big)) + 7 * middle * (!(R_big)) * (!(R_med)) + 6 * (!(R_big) * !(R_med) * !(middle) * (L_med)) + 6 * error_state;

      wheel_SetVel(L_speed_increment * common_speed_increment, R_speed_increment * common_speed_increment);
      ESP_LOGI(TAG, "%d %d", L_speed_increment, R_speed_increment);

      vTaskDelay(pdMS_TO_TICKS(50));
    }
}
