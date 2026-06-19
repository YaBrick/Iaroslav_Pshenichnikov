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
    const float estimated_increment = 11.63f; // obtido empiricamente
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



/* Cinemática inversa: velocidades do corpo (v [cm/s], w [rad/s]) → velocidades
 * lineares das rodas [cm/s]. d = WHEEL_HALF_TRACK_CM (meia-bitola).
 *   v_L = v + w*d ; v_R = v - w*d
 * (w positivo => roda esquerda mais rápida => giro à direita) */
cinematic_t inverse_cinematic_converter(float _target_lin_speed, float _target_ang_speed){
    cinematic_t Cinematic;
    Cinematic.l_wheel = _target_lin_speed + (_target_ang_speed * WHEEL_HALF_TRACK_CM);
    Cinematic.r_wheel = _target_lin_speed - (_target_ang_speed * WHEEL_HALF_TRACK_CM);
    return Cinematic;
}

/* Cinemática direta: velocidades lineares das rodas [cm/s] → velocidades do corpo.
 * É exatamente a inversa de inverse_cinematic_converter():
 *   v = (v_L + v_R) / 2          [cm/s]
 *   w = (v_L - v_R) / (2*d)      [rad/s]   (rad = cm/cm, adimensional) */
body_speed_t forward_cinematic_converter(float l_wheel, float r_wheel){
    body_speed_t body;
    body.linear  = (l_wheel + r_wheel) / 2.0f;
    body.angular = (l_wheel - r_wheel) / (2.0f * WHEEL_HALF_TRACK_CM);
    return body;
}

/* Converte os 5 bits dos sensores de linha num único estado exclusivo.
 * Reconhece as 5 posições simples e as 4 intermediárias (dois sensores
 * adjacentes). Qualquer outro padrão (0, não adjacentes, 3+ sensores) → LINE_LOST. */
line_state_t read_line_state(uint8_t sens){
    uint8_t s = sens & 0x1F;          // apenas os 5 sensores de linha (BIT0..BIT4)

    if (s == 0) return LINE_LOST;     // só "perdido" quando NENHUM sensor vê a linha

    /* Centróide dos sensores ativos: posição média em [0..4] (0=esq, 4=dir).
     * Robusto a 3+ sensores (linha grossa) e a padrões fora dos 9 casos exatos. */
    int sum = 0, count = 0;
    for (int i = 0; i < 5; i++){
        if (s & (1 << i)){ sum += i; count++; }
    }

    /* idx em [0..8] (passos de 0.5 do centróide) → estados LINE_L_DISTANT..LINE_R_DISTANT */
    int idx = (int)(((float)sum / count) * 2.0f + 0.5f);   // arredonda
    if (idx > 8) idx = 8;
    return (line_state_t)(LINE_L_DISTANT + idx);
}


portTASK_FUNCTION(speed_ctrl, args)
{
    /* Nível alto: velocidade linear do corpo. Saída em cm/s (máx ~16 cm/s) */
    pid_ctrl_config_t pid_linear_speed_config = {
        .init_param = {
        .kp = 0.3,
        .ki = 0.3,
        .kd = 0.0,
        .max_output = 16,
        .min_output = -16,
        .max_integral = 3,
        .min_integral = -3,
        .cal_type = PID_CAL_TYPE_INCREMENTAL
        }
    };

    /* Nível alto: velocidade angular do corpo. Saída em rad/s.
     * Com v_roda máx ~16 cm/s e d=10 cm: w_máx ≈ 32/(2*10) ≈ 1.6 rad/s. */
    pid_ctrl_config_t pid_angular_speed_config = {
        .init_param = {
        .kp = 0.4,
        .ki = 0.4,
        .kd = 0.0,
        .max_output = 2,
        .min_output = -2,
        .max_integral = 1,
        .min_integral = -1,
        .cal_type = PID_CAL_TYPE_INCREMENTAL
        }
    };

    /* Nível baixo: PID por velocidade de cada roda (mesma config para L e R) */
    pid_ctrl_config_t pid_wheel_config = {
        .init_param = {
        .kp = 0.1,
        .ki = 0.4,
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
    volatile float target_lin_speed, target_ang_speed = 0;
    volatile float L_pid, R_pid; //values after applying PID computing

    /* Nível alto (corpo): velocidade linear/angular */
    pid_ctrl_block_handle_t linear_pid_block;
    pid_ctrl_block_handle_t angular_pid_block;
    /* Nível baixo (rodas): velocidade de cada roda */
    pid_ctrl_block_handle_t L_pid_block;
    pid_ctrl_block_handle_t R_pid_block;

    ESP_ERROR_CHECK(pid_new_control_block(&pid_linear_speed_config,  &linear_pid_block));
    ESP_ERROR_CHECK(pid_new_control_block(&pid_angular_speed_config, &angular_pid_block));
    ESP_ERROR_CHECK(pid_new_control_block(&pid_wheel_config,         &L_pid_block));
    ESP_ERROR_CHECK(pid_new_control_block(&pid_wheel_config,         &R_pid_block));

    while(1){

      eventBits = xEventGroupGetBits(evt);
      speed = speed_estimator();

      /* Inverte só os 5 sensores de linha (ativos em baixo => bit=1 = linha detectada).
       * O BIT5 (flag de parada do sonar) é ativo em alto, então é mantido sem inversão. */
      uint8_t sens = (~eventBits & 0x1F) | (eventBits & BIT5);

      /* Estado da linha: exclusivo, inclui posições intermediárias (2 sensores) */
      line_state_t line = read_line_state(sens);

      bool sonar_stop = (sens & BIT5) != 0;

        /* === Setpoint do corpo a partir do estado (exclusivo) da linha ===
         * Linha à esquerda => virar à esquerda (w < 0, pois w > 0 gira à direita).
         * Quanto mais longe do centro, maior |w| e menor a velocidade linear. */
        const float V_MAX  = 12.0f;   // velocidade linear máxima (cm/s)
        const float W_STEP = 0.4f;    // passo de velocidade angular por nível (rad/s)

        if (sonar_stop){    // LINE_LOST cai no default do switch (pos=0) => segue reto como no centro
            target_lin_speed = 0;
            target_ang_speed = 0;
        }
        else{
            /* posição discreta: -4 (extrema esq) .. 0 (centro) .. +4 (extrema dir) */
            int pos;
            switch (line){
                case LINE_L_DISTANT:     pos = -4; break;
                case LINE_L_DISTANT_MID: pos = -3; break;
                case LINE_L_MID:         pos = -2; break;
                case LINE_L_MID_CENTER:  pos = -1; break;
                case LINE_CENTER:        pos =  0; break;
                case LINE_CENTER_R_MID:  pos =  1; break;
                case LINE_R_MID:         pos =  2; break;
                case LINE_R_MID_DISTANT: pos =  3; break;
                case LINE_R_DISTANT:     pos =  4; break;
                default:                 pos =  0; break;
            }
            int dist = (pos < 0) ? -pos : pos;
            target_ang_speed = pos * W_STEP;                  // gira na direção da linha
            target_lin_speed = V_MAX * (1.0f - 0.15f * dist); // mais devagar nas curvas
        }


        /* === Nível alto: PID de velocidade linear/angular do corpo ===
         * Medimos (lin, ang) a partir das velocidades das rodas (cinemática direta);
         * a saída do PID é o comando (lin, ang). Requer Ki != 0 para manter o comando. */
        body_speed_t measured = forward_cinematic_converter(speed.L, speed.R);
        float lin_cmd = 0, ang_cmd = 0;
        pid_compute(linear_pid_block,  (target_lin_speed - measured.linear),  &lin_cmd);
        pid_compute(angular_pid_block, (target_ang_speed - measured.angular), &ang_cmd);

        /* Cinemática inversa: comando (lin, ang) → velocidades-alvo das rodas */
        cinematic_t wheel_target = inverse_cinematic_converter(lin_cmd, ang_cmd);


        Выпили второй пид (по ШИМу) - он наху не нужен, первы пид делает именно то что нужно

        также оверушут в том числе из за того чтопроизводной части нет - добавь

        дада, как бы интегральная часть должна эта решать но нихуя, глянь графики в инете, правда

        подбери интегральную часть и производную, наверняка надо будет уменьшить линейный коэфф

        Сделай замер WCET - поднять гпио высоко под конец исполнения, замерить осликом. Оформить в график 
        желательно

        
        /* === Nível baixo: PID por velocidade de cada roda ===
         * erro = alvo da roda − velocidade medida da roda */
        pid_compute(L_pid_block, (wheel_target.l_wheel - speed.L), &L_pid);
        pid_compute(R_pid_block, (wheel_target.r_wheel - speed.R), &R_pid);


        ///Preparando sinal para enviar no wheel_task.c
        /* Converte cm/s → ticks de PWM AQUI, multiplicando ainda em float antes de
         * truncar, para não perder resolução (duty fino em vez de passos de 25).
         * !sonar_stop zera na parada. O clamp em ±400 fica no wheel_task. */
        const int common_speed_mult = 25;   // cm/s → ticks de PWM: ±16 cm/s → ±400
        int L_val = (int)(common_speed_mult * L_pid) * (!sonar_stop);
        int R_val = (int)(common_speed_mult * R_pid) * (!sonar_stop);
        uint16_t L_pkt = (uint16_t)(L_val + 1024);   // + 1024: transmite o sinal como unsigned
        uint16_t R_pkt = (uint16_t)(R_val + 1024);
        xTaskNotifyIndexed(wheel_handle, 0, (uint32_t)L_pkt | ((uint32_t)R_pkt << 16), eSetValueWithOverwrite);

        /* Telemetria para a GUI Python via UART do console (115200 baud).
         * Formato: >DATA:<sens>,<alvo_roda_L>,<alvo_roda_R>,<L_pid>,<R_pid>,<speed.L>,<speed.R>\n
         *   sens       - 6 bits dos sensores (bit=1 => linha detectada), bit5 = parada (sonar).
         *   alvo_roda  - velocidade-alvo de cada roda (cm/s), saída da cinemática inversa.
         *   speed      - velocidade estimada das rodas (cm/s). */
        printf(">DATA:%u,%d,%d,%.1f,%.1f,%d,%d\n",
               (unsigned)sens, wheel_target.l_wheel, wheel_target.r_wheel, L_pid, R_pid, speed.L, speed.R);

        vTaskDelay(pdMS_TO_TICKS(30));
        }
}
