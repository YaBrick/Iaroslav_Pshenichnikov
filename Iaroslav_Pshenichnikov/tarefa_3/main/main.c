#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include <inttypes.h>

#include "treeeyes_task.h"
#include "wheel_task.h"
#include "imu_task.h"
#include "ir_line_task.h"
#include "speed_ctrl_task.h"
#include "wdt_handler_task.h"
#include "wheel.h"

#define TREE_EYES_TASK
//#define IMU_TASK
#define WHEEL_CTRL_TASK
#define IR_LINE_CTRL_TASK
#define SPEED_CTRL_TASK


void app_main(void)
{

/* xEvents structure
   BIT  Descirption
   -------------------------
    0 - Most left IR sensor
    1 - Left IR sensor
    2 - Middle IR sensor
    3 - Right IR sensor
    4 - Most Right IR sensor
    5 - Stop flag (from treeeyes_task, sonar < 10 cm)
    rest - unused
*/    
EventGroupHandle_t xEvents = xEventGroupCreate();
TaskHandle_t wheel_handle = NULL;
TaskHandle_t speed_handle = NULL;

static handlers_t multiple_handlers;
/* static: lido pelo wdt_handler depois que app_main retorna */
static wdt_targets_t wdt_targets;

/* Hardware das rodas (MCPWM/PCNT/ADC) inicializado ANTES dos tasks:
 * - speed_ctrl le encoders ja no primeiro ciclo (sem corrida com o wheel_task);
 * - o handler do TWDT pode comandar os motores sem precisar de init no ISR. */
wheel_Init();

#ifdef IR_LINE_CTRL_TASK
    xTaskCreate(ir_line_ctrl,
                "ircontrol",
                configMINIMAL_STACK_SIZE*3,
                xEvents,
                20,
                NULL);
#endif

#ifdef TREE_EYES_TASK
    xTaskCreate(Treeeyes,
                "treeeyes",
                configMINIMAL_STACK_SIZE*3,
                xEvents,
                5,
                NULL);
#endif

#ifdef IMU_TASK
    xTaskCreate(IMU_Task,
                "imu",
                configMINIMAL_STACK_SIZE*3,
                NULL,
                5,
                NULL);
#endif

#ifdef WHEEL_CTRL_TASK
    xTaskCreate(wheel_ctrl,
                "wheel",
                configMINIMAL_STACK_SIZE*3,
                xEvents,
                10,
                &wheel_handle);
#endif

#ifdef SPEED_CTRL_TASK
    multiple_handlers.events = xEvents;
    multiple_handlers.wheel = wheel_handle;
    xTaskCreate(speed_ctrl,
                "speed",
                configMINIMAL_STACK_SIZE*3,
                &multiple_handlers,
                15,
                &speed_handle);
#endif

    /* Handler do TWDT: prioridade alta (fica bloqueado ate a ISR notificar).
     * Ao disparar, suspende speed_ctrl/wheel e gira o robo em torno do eixo. */
    wdt_targets.speed = speed_handle;
    wdt_targets.wheel = wheel_handle;
    xTaskCreate(wdt_handler,
                "wdt_handler",
                configMINIMAL_STACK_SIZE*3,
                &wdt_targets,
                24,
                NULL);

}