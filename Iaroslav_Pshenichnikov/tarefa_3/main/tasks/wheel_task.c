#include "wheel_task.h"

#include <stdio.h>
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "wheel.h"
#include "wcet.h"

const static char *TAG = "wheels";

portTASK_FUNCTION(wheel_ctrl, arg)
{
  //wcet_init(46, 47);
  esp_task_wdt_add(NULL);
  /* wheel_Init() agora e feito no app_main, antes da criacao dos tasks */
	
    /* Neutro (duty 0 nas duas rodas, offset 1024): sem a espera bloqueante,
   * o primeiro ciclo pode rodar antes da primeira notificacao chegar. */
  wheel_SetVel(0, 0);
  uint32_t packed = 1024u | (1024u << 16);

    /* Periodo fixo (RMS): vTaskDelayUntil mantem T constante, sem drift */
    TickType_t last_wake = xTaskGetTickCount();

    while(1){
      //wcet_begin(46, 47);
      /* Poll sem bloquear (timeout 0): task estritamente periodico para o RMS.
       * Sem notificacao nova, mantem o ultimo comando recebido em `packed`. */
      uint32_t incoming;
      if (xTaskNotifyWaitIndexed(
            0,                  // indice do slot de notificacao
            0,                  // nao limpa bits na entrada
            ULONG_MAX,          // pega o valor inteiro na saida
            &incoming,
            0) == pdTRUE){
          packed = incoming;
      }

      /* Recebido do speed_ctrl: duty de cada roda em ticks de PWM (com sinal, offset 1024) */
      int L_duty = (int)(packed & 0xFFFF) - 1024;
      int R_duty = (int)(packed >> 16)    - 1024;

      /* Limita a faixa do motor (+-400 ticks) */
      if(L_duty >  400){ L_duty =  400; }   if(L_duty < -400){ L_duty = -400; }
      if(R_duty >  400){ R_duty =  400; }   if(R_duty < -400){ R_duty = -400; }

      /* Valores com sinal, em ticks, como serao aplicados (para a telemetria) */
      int L_signed = L_duty, R_signed = R_duty;

      /* O sinal define o sentido; a magnitude vai para o PWM */
      wheel_GoForward();
      if(L_duty < 0){ left_wheel_GoBackward();  L_duty = -L_duty; }
      if(R_duty < 0){ right_wheel_GoBackward(); R_duty = -R_duty; }
      wheel_SetVel((uint32_t)L_duty, (uint32_t)R_duty);

      //ESP_LOGI(TAG, "L=%d  R=%d", L_signed, R_signed);

      /* Telemetria para a GUI Python: valores ja decodificados (com sinal),
       * em ticks de PWM, exatamente como aplicados aos motores.
       * Formato: >WHL:<L_signed>,<R_signed>\n */
      printf(">WHL:%d,%d\n", L_signed, R_signed);

      //wcet_end(47);
      esp_task_wdt_reset(); 
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(60));
    }
}
