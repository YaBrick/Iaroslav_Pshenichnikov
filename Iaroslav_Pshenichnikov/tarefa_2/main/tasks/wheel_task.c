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
  uint32_t packed = 0;

	uint32_t power_left_wheel, power_right_wheel; 

    //wheel_GetPower(&power_left_wheel, &power_right_wheel);
    //printf("Left ADC: %" PRIu32 "; \t Right ADC: %" PRIu32 ".\n", power_left_wheel, power_right_wheel);
	  //printf("Left ADC: %d\n", adc_left_raw[1][0]);

    while(1){

      xTaskNotifyWaitIndexed(
            0,                  // índice do slot de notificação
            0,                  // não limpa bits na entrada
            ULONG_MAX,          // pega o valor inteiro na saída
            &packed,
            portMAX_DELAY);

      /* Recebido do speed_ctrl: duty de cada roda em ticks de PWM (com sinal, offset 1024) */
      int L_duty = (int)(packed & 0xFFFF) - 1024;
      int R_duty = (int)(packed >> 16)    - 1024;

      /* Limita à faixa do motor (±400 ticks) */
      if(L_duty >  400){ L_duty =  400; }   if(L_duty < -400){ L_duty = -400; }
      if(R_duty >  400){ R_duty =  400; }   if(R_duty < -400){ R_duty = -400; }

      /* Valores com sinal, em ticks, como serão aplicados (para a telemetria) */
      int L_signed = L_duty, R_signed = R_duty;

      /* O sinal define o sentido; a magnitude vai para o PWM */
      wheel_GoForward();
      if(L_duty < 0){ left_wheel_GoBackward();  L_duty = -L_duty; }
      if(R_duty < 0){ right_wheel_GoBackward(); R_duty = -R_duty; }
      wheel_SetVel((uint32_t)L_duty, (uint32_t)R_duty);

      //ESP_LOGI(TAG, "L=%d  R=%d", L_signed, R_signed);

      /* Telemetria para a GUI Python: valores já decodificados (com sinal),
       * em ticks de PWM, exatamente como aplicados aos motores.
       * Formato: >WHL:<L_signed>,<R_signed>\n */
      printf(">WHL:%d,%d\n", L_signed, R_signed);

      vTaskDelay(pdMS_TO_TICKS(50));
    }
}
