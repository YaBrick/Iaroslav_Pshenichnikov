#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "ir_line_task.h"
#include "wheel.h"

static const char *TAG = "IR_Line";

#define INFRA_RED_VERY_LEFT_GPIO    GPIO_NUM_15
#define INFRA_RED_LEFT_GPIO         GPIO_NUM_16
#define INFRA_RED_MIDDLE_GPIO       GPIO_NUM_17
#define INFRA_RED_RIGHT_GPIO        18
#define INFRA_RED_VERY_RIGHT_GPIO   8

#define INFRA_RED_OUT_GPIO_MASK ((uint64_t)( \
    (1ULL << INFRA_RED_VERY_LEFT_GPIO)  | \
    (1ULL << INFRA_RED_LEFT_GPIO)       | \
    (1ULL << INFRA_RED_MIDDLE_GPIO)     | \
    (1ULL << INFRA_RED_RIGHT_GPIO)      | \
    (1ULL << INFRA_RED_VERY_RIGHT_GPIO)))

#define COMMON_SPEED   27

portTASK_FUNCTION(ir_line_ctrl, args)
{
    gpio_config_t ir_line_config = {
        .pin_bit_mask = INFRA_RED_OUT_GPIO_MASK,
        .mode         = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&ir_line_config);

    wheel_Init();
    wheel_GoForward();

    while (1) {
        
        /* Read sensors — sensor outputs 0 when line is detected */
        uint8_t raw = (uint8_t)(
            (uint8_t)gpio_get_level(INFRA_RED_VERY_LEFT_GPIO)       |
            (uint8_t)gpio_get_level(INFRA_RED_LEFT_GPIO)        << 1 |
            (uint8_t)gpio_get_level(INFRA_RED_MIDDLE_GPIO)      << 2 |
            (uint8_t)gpio_get_level(INFRA_RED_RIGHT_GPIO)       << 3 |
            (uint8_t)gpio_get_level(INFRA_RED_VERY_RIGHT_GPIO)  << 4
        );

        wheel_GoForward();

        /* Invert: bit = 1 means line detected under that sensor */
        uint8_t sens = ~raw & 0x1F;

        bool L_big   = (sens & BIT0) != 0;   /* very-left  */
        bool L_med   = (sens & BIT1) != 0;   /* left       */
        bool middle  = (sens & BIT2) != 0;   /* middle     */
        bool R_med   = (sens & BIT3) != 0;   /* right      */
        bool R_big   = (sens & BIT4) != 0;   /* very-right */

        bool error_state = (sens == 0x00);

        /* Speed multipliers (0-11) — mirrors original wheel_task formula */
        uint8_t L_mult = 5 *  L_big
                       +  8 * (L_med  & !L_big)
                       +  7 * (middle & !L_big  & !L_med)
                       +  6 * (!L_big & !L_med  & !middle & R_med)
                       +  7 *  error_state;

        uint8_t R_mult = 5 *  R_big
                       +  8 * (R_med  & !R_big)
                       +  7 * (middle & !R_big  & !R_med)
                       +  6 * (!R_big & !R_med  & !middle & L_med)
                       +  7 *  error_state;
        
        if(L_mult == 0){
            left_wheel_GoBackward();
            L_mult = 12;
        }
        if(R_mult == 0){
            right_wheel_GoBackward();
            R_mult = 12;
        }
        wheel_SetVel(L_mult * COMMON_SPEED, R_mult * COMMON_SPEED);

        ESP_LOGI(TAG, "sens=0x%02X  L=%d  R=%d", sens, L_mult, R_mult);

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}
