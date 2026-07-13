#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Tasks que o handler suspende quando o TWDT dispara (para que nao voltem a
 * comandar os motores por cima do estado de panico). */
typedef struct {
    TaskHandle_t speed;
    TaskHandle_t wheel;
} wdt_targets_t;

portTASK_FUNCTION_PROTO(wdt_handler, arg);
