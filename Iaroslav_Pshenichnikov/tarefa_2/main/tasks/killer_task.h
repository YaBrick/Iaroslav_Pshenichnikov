#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Task de teste do TWDT: entra em busy-loop e nunca cede a CPU,
 * fazendo o watchdog estourar (para validar o handler). */
portTASK_FUNCTION_PROTO(killer, arg);
