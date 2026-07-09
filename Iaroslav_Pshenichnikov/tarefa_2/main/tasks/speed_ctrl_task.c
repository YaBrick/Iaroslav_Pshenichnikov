#include "speed_ctrl_task.h"

#include <stdio.h>
#include "driver/gpio.h"    // macros BIT0..BIT5
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "pid_ctrl.h"
#include "wheel.h"
#include "wcet.h"

const static char *TAG = "speed_ctrl";

speed_t speed_estimator(void){
    int pL = 0, pR = 0;
    static int last_pL = 0, last_pR = 0;
    const float estimated_increment = 11.63f; // obtido empiricamente
    wheel_GetEndoderPulses(&pL, &pR);

    /* dt em microssegundos (sem arredondar para ms) e calculo em float:
     * elimina a quantizacao da velocidade. A constante continua valida pois
     * deltap*inc*1000/dt_us == deltap*inc/dt_ms. */
    int64_t now_us = esp_timer_get_time();
    static int64_t last_us = 0;
    int64_t dt_us = now_us - last_us;

    speed_t s = {0};
    if (dt_us > 0) {
        s.L = (pL - last_pL) * estimated_increment * 1000.0f / dt_us;
        s.R = (pR - last_pR) * estimated_increment * 1000.0f / dt_us;
    }

    last_us = now_us;
    last_pL = pL;
    last_pR = pR;
    return s;
}



/* Cinematica inversa: velocidades do corpo (v [cm/s], w [rad/s]) -> velocidades
 * lineares das rodas [cm/s]. d = WHEEL_HALF_TRACK_CM (meia-bitola).
 *   v_L = v + w*d ; v_R = v - w*d
 * (w positivo => roda esquerda mais rapida => giro a direita) */
cinematic_t inverse_cinematic_converter(float _target_lin_speed, float _target_ang_speed){
    cinematic_t Cinematic;
    Cinematic.l_wheel = _target_lin_speed + (_target_ang_speed * WHEEL_HALF_TRACK_CM);
    Cinematic.r_wheel = _target_lin_speed - (_target_ang_speed * WHEEL_HALF_TRACK_CM);
    return Cinematic;
}

/* Cinematica direta: velocidades lineares das rodas [cm/s] -> velocidades do corpo.
 * E exatamente a inversa de inverse_cinematic_converter():
 *   v = (v_L + v_R) / 2          [cm/s]
 *   w = (v_L - v_R) / (2*d)      [rad/s]   (rad = cm/cm, adimensional) */
body_speed_t forward_cinematic_converter(float l_wheel, float r_wheel){
    body_speed_t body;
    body.linear  = (l_wheel + r_wheel) / 2.0f;
    body.angular = (l_wheel - r_wheel) / (2.0f * WHEEL_HALF_TRACK_CM);
    return body;
}

/* Converte os 5 bits dos sensores de linha num unico estado exclusivo.
 * Reconhece as 5 posicoes simples e as 4 intermediarias (dois sensores
 * adjacentes). Qualquer outro padrao (0, nao adjacentes, 3+ sensores) -> LINE_LOST. */
line_state_t read_line_state(uint8_t sens){
    uint8_t s = sens & 0x1F;          // apenas os 5 sensores de linha (BIT0..BIT4)

    if (s == 0) return LINE_LOST;     // so "perdido" quando NENHUM sensor ve a linha

    /* Centroide dos sensores ativos: posicao media em [0..4] (0=esq, 4=dir).
     * Robusto a 3+ sensores (linha grossa) e a padroes fora dos 9 casos exatos. */
    int sum = 0, count = 0;
    for (int i = 0; i < 5; i++){
        if (s & (1 << i)){ sum += i; count++; }
    }

    /* idx em [0..8] (passos de 0.5 do centroide) -> estados LINE_L_DISTANT..LINE_R_DISTANT */
    int idx = (int)(((float)sum / count) * 2.0f + 0.5f);   // arredonda
    if (idx > 8) idx = 8;
    return (line_state_t)(LINE_L_DISTANT + idx);
}


portTASK_FUNCTION(speed_ctrl, args)
{
    //wcet_init(46, 47);
    esp_task_wdt_add(NULL);
    /* Unico PID de velocidade linear do corpo. Saida em cm/s (max ~16 cm/s).
     * kd pequeno: a velocidade medida e quantizada/ruidosa, derivada alta amplifica ruido. */
    pid_ctrl_config_t pid_linear_speed_config = {
        .init_param = {
        .kp = 0.8,
        .ki = 0.04,
        .kd = 0.01,
        .max_output = 16,
        .min_output = 0,
        .max_integral = 5,
        .min_integral = -5,
        .cal_type = PID_CAL_TYPE_INCREMENTAL
        }
    };

    /* Nivel alto: velocidade angular do corpo. Saida em rad/s.
     * Com v_roda max ~16 cm/s e d=10 cm: w_max ~ 32/(2*10) ~ 1.6 rad/s. */
    pid_ctrl_config_t pid_angular_speed_config = {
        .init_param = {
        .kp = 0.4,
        .ki = 0.05,
        .kd = 0.0,
        .max_output = 1.7,
        .min_output = -1.7,
        .max_integral = 0.4,
        .min_integral = -0.4,
        .cal_type = PID_CAL_TYPE_INCREMENTAL
        }
    };

    handlers_t *ctx = (handlers_t *)args;
    EventGroupHandle_t evt          = ctx->events;
    TaskHandle_t       wheel_handle = ctx->wheel;
    speed_t speed;


    int eventBits;
    volatile float target_lin_speed, target_ang_speed = 0;

    /* Unico nivel: PID de velocidade linear/angular do corpo */
    pid_ctrl_block_handle_t linear_pid_block;
    pid_ctrl_block_handle_t angular_pid_block;

    ESP_ERROR_CHECK(pid_new_control_block(&pid_linear_speed_config,  &linear_pid_block));
    ESP_ERROR_CHECK(pid_new_control_block(&pid_angular_speed_config, &angular_pid_block));

    /* Periodo fixo (RMS): vTaskDelayUntil mantem T constante, sem drift */
    TickType_t last_wake = xTaskGetTickCount();

    while(1){
      //wcet_begin(46, 47);

      eventBits = xEventGroupGetBits(evt);
      speed = speed_estimator();

      /* Inverte so os 5 sensores de linha (ativos em baixo => bit=1 = linha detectada).
       * O BIT5 (flag de parada do sonar) e ativo em alto, entao e mantido sem inversao. */
      uint8_t sens = (~eventBits & 0x1F) | (eventBits & BIT5);

      /* Estado da linha: exclusivo, inclui posicoes intermediarias (2 sensores) */
      line_state_t line = read_line_state(sens);

      bool sonar_stop = (sens & BIT5) != 0;

        /* === Setpoint do corpo a partir do estado (exclusivo) da linha ===
         * Linha a esquerda => virar a esquerda (w < 0, pois w > 0 gira a direita).
         * Quanto mais longe do centro, maior |w| e menor a velocidade linear. */
        const float V_MAX  = 12.0f;   // velocidade linear maxima (cm/s)
        const float W_STEP = 0.4f;    // passo de velocidade angular por nivel (rad/s)

        /* Setpoint a partir do estado da linha (LINE_LOST cai no default => pos=0 => reto).
         * Calculado sempre, mesmo parado, para os graficos ficarem coerentes. */
        int pos;
        switch (line){
            case LINE_L_DISTANT:     pos = -5; break;
            case LINE_L_DISTANT_MID: pos = -3; break;
            case LINE_L_MID:         pos = -2; break;
            case LINE_L_MID_CENTER:  pos = -1; break;
            case LINE_CENTER:        pos =  0; break;
            case LINE_CENTER_R_MID:  pos =  1; break;
            case LINE_R_MID:         pos =  2; break;
            case LINE_R_MID_DISTANT: pos =  3; break;
            case LINE_R_DISTANT:     pos =  5; break;
            default:                 pos =  0; break;
        }
        int dist = (pos < 0) ? -pos : pos;
        target_ang_speed = pos * W_STEP;                  // gira na direcao da linha
        target_lin_speed = V_MAX * (1.0f - 0.15f * dist); // mais devagar nas curvas

        /* === Nivel alto: PID de velocidade linear/angular do corpo ===
         * Se o sonar levanta a flag de parada, CONGELAMOS o PID: nao chamamos
         * pid_compute (preserva integral, erros e saida anteriores) e mandamos 0
         * aos motores. Assim, ao retomar (ex.: no meio de uma curva fechada) o
         * controle continua de onde parou, sem zerar o comando angular. */
        float lin_cmd = 0, ang_cmd = 0;
        cinematic_t wheel_target = {0.0f, 0.0f};
        int L_val = 0, R_val = 0;

        if (!sonar_stop){
            body_speed_t measured = forward_cinematic_converter(speed.L, speed.R);
            pid_compute(linear_pid_block,  (target_lin_speed - measured.linear),  &lin_cmd);
            pid_compute(angular_pid_block, (target_ang_speed - measured.angular), &ang_cmd);

            /* Cinematica inversa: comando (lin, ang) -> velocidades-alvo das rodas [cm/s] */
            wheel_target = inverse_cinematic_converter(lin_cmd, ang_cmd);

            /* cm/s -> ticks de PWM: multiplica em float antes de truncar (duty fino);
             * o clamp em +-400 fica no wheel_task. */
            const int common_speed_mult = 25;   // +-16 cm/s -> +-400
            L_val = (int)(common_speed_mult * wheel_target.l_wheel);
            R_val = (int)(common_speed_mult * wheel_target.r_wheel);
        }
        /* sonar_stop: L_val=R_val=0 e PID congelado (pid_compute nao foi chamado) */

        ///Preparando sinal para enviar no wheel_task.c
        uint16_t L_pkt = (uint16_t)(L_val + 1024);   // + 1024: transmite o sinal como unsigned
        uint16_t R_pkt = (uint16_t)(R_val + 1024);
        xTaskNotifyIndexed(wheel_handle, 0, (uint32_t)L_pkt | ((uint32_t)R_pkt << 16), eSetValueWithOverwrite);

        /* Telemetria para a GUI Python via UART do console (115200 baud).
         * Formato: >DATA:<sens>,<alvo_roda_L>,<alvo_roda_R>,<lin_cmd>,<ang_cmd>,<speed.L>,<speed.R>\n
         *   sens       - 6 bits dos sensores (bit=1 => linha detectada), bit5 = parada (sonar).
         *   alvo_roda  - velocidade-alvo de cada roda (cm/s), saida da cinematica inversa.
         *   lin/ang_cmd  - comando do PID do corpo (v em cm/s, w em rad/s).
         *   speed        - velocidade estimada das rodas (cm/s).
         *   tgt_lin/ang  - setpoint do corpo (v em cm/s, w em rad/s). */
        printf(">DATA:%u,%d,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
               (unsigned)sens, (int)wheel_target.l_wheel, (int)wheel_target.r_wheel,
               lin_cmd, ang_cmd, speed.L, speed.R, target_lin_speed, target_ang_speed);

        //wcet_end(47);
        //esp_task_wdt_reset(); 
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(30));
        }
}
