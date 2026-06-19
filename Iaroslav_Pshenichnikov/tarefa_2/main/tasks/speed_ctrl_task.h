#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

/* Meia-bitola: metade da distância entre as rodas, em cm.
 * Usada na cinemática diferencial (v_roda = v ± w*d). CALIBRAR conforme o robô. */
#define WHEEL_HALF_TRACK_CM 10.0f

typedef struct {
    EventGroupHandle_t events;
    TaskHandle_t       wheel;
} handlers_t;


typedef struct {
    int L;
    int R;
} speed_t;
// cm/s

typedef struct {
    int l_wheel;
    int r_wheel;
} cinematic_t; /// Velocidades lineares das rodas, em cm/s

typedef struct {
    float linear;   // velocidade linear do corpo (cm/s)
    float angular;  // velocidade angular do corpo (rad/s)
} body_speed_t;

/* Estado da linha: exclusivo (apenas um ativo por vez), da esquerda para a direita.
 * Os estados "_*" entre posições são os intermediários (dois sensores adjacentes
 * veem a linha ao mesmo tempo). LINE_LOST = nenhuma linha / padrão ambíguo. */
typedef enum {
    LINE_LOST = 0,
    LINE_L_DISTANT,        // só extremo esquerdo
    LINE_L_DISTANT_MID,    // extremo esq + esq      (intermediário)
    LINE_L_MID,            // só esquerdo
    LINE_L_MID_CENTER,     // esq + centro           (intermediário)
    LINE_CENTER,           // só centro
    LINE_CENTER_R_MID,     // centro + dir           (intermediário)
    LINE_R_MID,            // só direito
    LINE_R_MID_DISTANT,    // dir + extremo dir      (intermediário)
    LINE_R_DISTANT,        // só extremo direito
} line_state_t;

/* Estima a velocidade de cada roda (cm/s) a partir dos pulsos do encoder. */
speed_t speed_estimator(void);

/* Cinemática inversa: (v [cm/s], w [rad/s]) do corpo → velocidades das rodas [cm/s]. */
cinematic_t inverse_cinematic_converter(float _target_lin_speed, float _target_ang_speed);

/* Cinemática direta: velocidades das rodas [cm/s] → (v [cm/s], w [rad/s]) do corpo. */
body_speed_t forward_cinematic_converter(float l_wheel, float r_wheel);

/* Converte os bits dos sensores num único estado exclusivo da linha. */
line_state_t read_line_state(uint8_t sens);

portTASK_FUNCTION(speed_ctrl, args);