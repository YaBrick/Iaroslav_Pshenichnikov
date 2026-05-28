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


const static char *TAG = "wheels";

portTASK_FUNCTION(wheel_ctrl, arg)
{
	wheel_Init();
	wheel_SetVel(100, 100);
  uint32_t L_speed = 0, R_speed = 0, packed = 0;

	uint32_t power_left_wheel, power_right_wheel; 

    //wheel_GetPower(&power_left_wheel, &power_right_wheel);
    //printf("Left ADC: %" PRIu32 "; \t Right ADC: %" PRIu32 ".\n", power_left_wheel, power_right_wheel);
	  //printf("Left ADC: %d\n", adc_left_raw[1][0]);

    while(1){

      xTaskNotifyWaitIndexed(
            0,                  // индекс ящика
            0,                  // не сбрасываем биты на входе
            ULONG_MAX,          // полностью забираем значение на выходе
            &packed,
            portMAX_DELAY);

      L_speed = packed & 0xFFFF; R_speed = packed >> 16;
      wheel_GoForward();
      if(L_speed < 1024){
        L_speed = 1024 - L_speed;
        left_wheel_GoBackward();
      }
      else{
        L_speed = L_speed - 1024;
      }
      
      if(R_speed < 1024){
        R_speed = 1024 - R_speed;
        right_wheel_GoBackward();
      }
      else{
        R_speed = R_speed - 1024;
      }
      wheel_SetVel(L_speed, R_speed);

      ESP_LOGI(TAG, "L=%lu  R=%lu", L_speed, R_speed);

      vTaskDelay(pdMS_TO_TICKS(50));
    }
}
