#include "wheel.h"

/* Chamado do contexto de INTERRUPCAO do TWDT: apenas acoes minimas e nao
 * bloqueantes (escritas de registrador). wheel_Init() ja foi feito no app_main -
 * refaze-lo aqui (mallocs/mutexes em ISR) causaria crash. Sinalizacao de panico:
 * o robo gira em torno do proprio eixo. */
void esp_task_wdt_isr_user_handler(void){
    wheel_GoForward();
    right_wheel_GoBackward();
	wheel_SetVel(100, 100);
}