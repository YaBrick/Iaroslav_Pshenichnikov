#include "wdt_handler_task.h"

#include "wheel.h"

/* Handle do proprio handler task: alvo da notificacao vinda da ISR do TWDT. */
static TaskHandle_t s_handler_task = NULL;

/* Chamado da ISR do TWDT (contexto de interrupcao). Substitui o weak-hook do
 * ESP-IDF. Nao faz o trabalho pesado aqui (suspender tasks nao e permitido em
 * ISR); apenas acorda o handler task via notificacao. */
void esp_task_wdt_isr_user_handler(void){
    if (s_handler_task != NULL){
        BaseType_t higher_prio_woken = pdFALSE;
        vTaskNotifyGiveFromISR(s_handler_task, &higher_prio_woken);
        portYIELD_FROM_ISR(higher_prio_woken);
    }
}

portTASK_FUNCTION(wdt_handler, arg)
{
    wdt_targets_t *targets = (wdt_targets_t *)arg;

    /* Registra o proprio handle para a ISR conseguir notificar */
    s_handler_task = xTaskGetCurrentTaskHandle();

    while(1){
        /* Bloqueia ate a ISR do TWDT notificar (nao consome CPU enquanto espera) */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Congela o controle: sem isso, speed_ctrl e wheel continuariam rodando
         * e sobrescreveriam os motores no ciclo seguinte. */
        if (targets->speed != NULL) vTaskSuspend(targets->speed);
        if (targets->wheel != NULL) vTaskSuspend(targets->wheel);

        /* Estado de panico: gira o robo em torno do proprio eixo */
        wheel_GoForward();
        right_wheel_GoBackward();
        wheel_SetVel(300, 300);
    }
}
