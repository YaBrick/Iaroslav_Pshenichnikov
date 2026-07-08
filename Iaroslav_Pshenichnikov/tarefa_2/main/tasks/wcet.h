#pragma once
#include "driver/gpio.h"

/* === Pinos para medicao de WCET (um par por task) ===
 * START sobe no inicio de cada ciclo do task; END sobe no fim (antes do delay).
 * No osciloscopio / analisador logico: o tempo entre a borda de subida de START
 * e a de END e o tempo de execucao daquele ciclo. Ambos descem no inicio do
 * ciclo seguinte (gap entre medicoes).
 *
 * Use -1 para desabilitar. DEFINIR os GPIOs conforme a placa. */
#define WCET_SPEED_START_GPIO     -1
#define WCET_SPEED_END_GPIO       -1
#define WCET_WHEEL_START_GPIO     -1
#define WCET_WHEEL_END_GPIO       -1
#define WCET_IRLINE_START_GPIO    -1
#define WCET_IRLINE_END_GPIO      -1
#define WCET_TREEEYES_START_GPIO  -1
#define WCET_TREEEYES_END_GPIO    -1
#define WCET_IMU_START_GPIO       -1
#define WCET_IMU_END_GPIO         -1

/* Configura os dois pinos como saida (chamar uma vez, antes do while). */
static inline void wcet_init(int start_gpio, int end_gpio){
    if (start_gpio >= 0){ gpio_set_direction(start_gpio, GPIO_MODE_OUTPUT); gpio_set_level(start_gpio, 0); }
    if (end_gpio   >= 0){ gpio_set_direction(end_gpio,   GPIO_MODE_OUTPUT); gpio_set_level(end_gpio,   0); }
}

/* Define o nivel de um pino (ignora se desabilitado, gpio < 0). */
static inline void wcet_set(int gpio, int level){
    if (gpio >= 0) gpio_set_level(gpio, level);
}

/* Reseta o par (descer ambos) e marca o inicio (subir START). No topo do ciclo. */
static inline void wcet_begin(int start_gpio, int end_gpio){
    wcet_set(start_gpio, 0);
    wcet_set(end_gpio, 0);
    wcet_set(start_gpio, 1);
}

/* Marca o fim (subir END). No fim do ciclo, antes do delay. */
static inline void wcet_end(int end_gpio){
    wcet_set(end_gpio, 1);
}
