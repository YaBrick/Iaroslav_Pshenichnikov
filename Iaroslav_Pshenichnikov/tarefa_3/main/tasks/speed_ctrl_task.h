#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

/* Meia-bitola: metade da distancia entre as rodas, em cm.
 * Usada na cinematica diferencial (v_roda = v +- w*d). CALIBRAR conforme o robo. */
#define WHEEL_HALF_TRACK_CM 10.0f

typedef struct {
    EventGroupHandle_t events;
    TaskHandle_t       wheel;
} handlers_t;


typedef struct {
    float L;
    float R;
} speed_t;
// cm/s

typedef struct {
    float l_wheel;
    float r_wheel;
} cinematic_t; /// Velocidades lineares das rodas, em cm/s

typedef struct {
    float linear;   // velocidade linear do corpo (cm/s)
    float angular;  // velocidade angular do corpo (rad/s)
} body_speed_t;

/* Estado da linha: exclusivo (apenas um ativo por vez), da esquerda para a direita.
 * Os estados "_*" entre posicoes sao os intermediarios (dois sensores adjacentes
 * veem a linha ao mesmo tempo). LINE_LOST = nenhuma linha / padrao ambiguo. */
typedef enum {
    LINE_LOST = 0,
    LINE_L_DISTANT,        // so extremo esquerdo
    LINE_L_DISTANT_MID,    // extremo esq + esq      (intermediario)
    LINE_L_MID,            // so esquerdo
    LINE_L_MID_CENTER,     // esq + centro           (intermediario)
    LINE_CENTER,           // so centro
    LINE_CENTER_R_MID,     // centro + dir           (intermediario)
    LINE_R_MID,            // so direito
    LINE_R_MID_DISTANT,    // dir + extremo dir      (intermediario)
    LINE_R_DISTANT,        // so extremo direito
} line_state_t;

/* Estima a velocidade de cada roda (cm/s) a partir dos pulsos do encoder. */
speed_t speed_estimator(void);

/* Cinematica inversa: (v [cm/s], w [rad/s]) do corpo -> velocidades das rodas [cm/s]. */
cinematic_t inverse_cinematic_converter(float _target_lin_speed, float _target_ang_speed);

/* Cinematica direta: velocidades das rodas [cm/s] -> (v [cm/s], w [rad/s]) do corpo. */
body_speed_t forward_cinematic_converter(float l_wheel, float r_wheel);

/* Converte os bits dos sensores num unico estado exclusivo da linha. */
line_state_t read_line_state(uint8_t sens);

portTASK_FUNCTION_PROTO(speed_ctrl, args);