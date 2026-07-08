#include "wheel.h"

/* Chamado do contexto de INTERRUPÇÃO do TWDT: apenas ações mínimas e não
 * bloqueantes (escritas de registrador). wheel_Init() já foi feito no app_main —
 * refazê-lo aqui (mallocs/mutexes em ISR) causaria crash. Sinalização de pânico:
 * o robô gira em torno do próprio eixo. */
void esp_task_wdt_isr_user_handler(void){
    wheel_GoForward();
    right_wheel_GoBackward();
	wheel_SetVel(100, 100);
}