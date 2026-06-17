#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "wheel.h"
#include "esp_timer.h"
#include "speed_ctrl_task.h"
#include "pid_ctrl.h"

const static char *TAG = "speed_ctrl";

speed_t speed_estimator(void){
    int pL = 0, pR = 0;
    static int last_pL = 0, last_pR = 0;
    const float estimated_increment = 11.63f; // got it empirically
    wheel_GetEndoderPulses(&pL, &pR);

    int64_t timestamp_ms = esp_timer_get_time() / 1000;
    static int64_t last_timestamp = 0;

    int64_t dt = timestamp_ms - last_timestamp;  

    speed_t s = {0};
    if ((dt > 0)) {                                 
        s.L = ((pL - last_pL) * estimated_increment) / (int)dt;
        s.R =  ((pR - last_pR) * estimated_increment) / (int)dt;
    }

    last_timestamp = timestamp_ms;
    last_pL = pL;
    last_pR = pR;
    return s;
}



portTASK_FUNCTION(speed_ctrl, args)
{
    pid_ctrl_config_t pid_config = {
        .init_param = {
        .kp = 1,
        .ki = 0.0,
        .kd = 0.0,
        .max_output = 16,
        .min_output = -16,
        .max_integral = 3,
        .min_integral = -3,
        .cal_type = PID_CAL_TYPE_INCREMENTAL
        }
    };

    handlers_t *ctx = (handlers_t *)args;
    EventGroupHandle_t evt          = ctx->events;
    TaskHandle_t       wheel_handle = ctx->wheel;
    speed_t speed;


    int eventBits;

    volatile bool L_big, L_med;
    volatile bool R_big, R_med;
    volatile int16_t L_mult, R_mult;
    const int speed_scale       = 1;    // setpoint: peso L_mult (máx ~8) → cm/s (máx ~16)
    const int common_speed_mult = 25;   // saída do PID (cm/s, ±16) → ticks de PWM (±400): 400/16 ≈ 25

    volatile float L_pid, R_pid; //values after applying PID computing

    volatile bool middle, error_state;

    pid_ctrl_block_handle_t L_pid_block;
    pid_ctrl_block_handle_t R_pid_block;
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &L_pid_block));
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &R_pid_block));

    while(1){

      eventBits = xEventGroupGetBits(evt);
      speed = speed_estimator();

              /* Invert: bit = 1 means line detected under that sensor */
      uint8_t sens = ~eventBits & 0x3F;

      bool L_big   = (sens & BIT0) != 0;   /* very-left  */
      bool L_med   = (sens & BIT1) != 0;   /* left       */
      bool middle  = (sens & BIT2) != 0;   /* middle     */
      bool R_med   = (sens & BIT3) != 0;   /* right      */
      bool R_big   = (sens & BIT4) != 0;   /* very-right */

      bool error_state = (sens == 0x00);

      bool sonar_stop = (sens & BIT5) != 0;

        L_mult = 5 *  R_big
                +  8 * (R_med  & !R_big)
                +  7 * (middle & !R_big  & !R_med)
                +  6 * (!R_big & !R_med  & !middle & L_med)
                +  7 *  error_state
                -  12 * (L_big);

        R_mult = 5 *  L_big
                +  8 * (L_med  & !L_big)
                +  7 * (middle & !L_big  & !L_med)
                +  6 * (!L_big & !L_med  & !middle & R_med)
                +  7 *  error_state
                -  12 * (R_big);


        /* Setpoint em cm/s, erro = alvo − medido (ambos em cm/s) */
        int target_L = L_mult * speed_scale;
        int target_R = R_mult * speed_scale;
        pid_compute(L_pid_block, (target_L - speed.L), &L_pid);
        pid_compute(R_pid_block, (target_R - speed.R), &R_pid);

        /* Saída do PID (cm/s) → ticks de PWM. Calculamos o valor com sinal em int,
         * limitamos à faixa do motor (±400 ticks), e só então somamos 1024
         * e convertemos para uint16_t — assim o cast sempre recebe um valor não negativo. */
        int L_val = (int)(common_speed_mult * L_pid) * (int)sonar_stop;
        int R_val = (int)(common_speed_mult * R_pid) * (int)sonar_stop;
        if (L_val >  400){L_val =  400;}   if (L_val < -400){L_val = -400;}
        if (R_val >  400){R_val =  400;}   if (R_val < -400){R_val = -400;}
        uint16_t L_pkt = (uint16_t)(L_val + 1024);   // + 1024: transmite o sinal como unsigned
        uint16_t R_pkt = (uint16_t)(R_val + 1024);
        xTaskNotifyIndexed(wheel_handle, 0, (uint32_t)L_pkt | ((uint32_t)R_pkt << 16), eSetValueWithOverwrite);

        //ESP_LOGI(TAG, "Left encoder: %d\tRight encoder: %d\r\n", pL, pR);
        //ESP_LOGI(TAG, "L = %d R = %d", speed.L, speed.R);

        /* Telemetria para a GUI Python via UART do console (115200 baud).
         * Formato: >DATA:<sens>,<L_mult>,<R_mult>,<L_pid>,<R_pid>,<speed.L>,<speed.R>\n
         *   sens  - 6 bits dos sensores (bit=1 => linha detectada), bit5 = parada (sonar).
         *   speed - velocidade estimada das rodas (cm/s). */
        printf(">DATA:%u,%d,%d,%.1f,%.1f,%d,%d\n",
               (unsigned)sens, L_mult, R_mult, L_pid, R_pid, speed.L, speed.R);

        vTaskDelay(pdMS_TO_TICKS(30));
        }
}
