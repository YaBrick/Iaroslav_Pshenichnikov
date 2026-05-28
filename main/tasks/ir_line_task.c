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

    bool most_left, left, middle, right, most_right, all_sensors;
    int8_t L_mult, R_mult;

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

        /* Invert: bit = 1 means line detected under that sensor */
        uint8_t sens = ~raw & 0x1F;

        most_left =   (sens & BIT0) != 0;  
        left =        (sens & BIT1) != 0;  
        middle =      (sens & BIT2) != 0;
        right =       (sens & BIT3) != 0;  
        most_right =  (sens & BIT4) != 0;  

        all_sensors = (sens == 0x00);

        /* Speed multipliers

        * - line not detected
        # - line detected
        ? - state doesn't matter
        
        */

        L_mult =          -12 *  most_right // ????#
                       +  1 * (right  & !most_right) // ???#*
                       +  14 * (middle & !most_right  & !right) // ??#**
                       +  9 * (!most_right & !right  & !middle & left) // ?#***
                       +  6 * (!most_right & !right  & !middle & !left & most_left) // #****
                       +  9 *  all_sensors; // *****

        R_mult =          -12 *  most_left // #????
                       +  1 * (left  & !most_left) // *#???
                       +  14 * (middle & !most_left  & !left) // **#??
                       +  9 * (!most_left & !left  & !middle & right) // ***#?
                       +  6 * (!most_left & !left  & !middle & !right & most_right) // ****#
                       +  9 *  all_sensors; // *****
        
        ESP_LOGI(TAG, "sens=0x%02X  L=%d  R=%d", sens, L_mult, R_mult);

        if(L_mult < 0){
            L_mult = -L_mult;
            left_wheel_GoBackward();
        } else {
            left_wheel_GoForward();
        }
        if(R_mult < 0){
            R_mult = -R_mult;
            right_wheel_GoBackward();
        } else {
            right_wheel_GoForward();
        }
        wheel_SetVel(L_mult * COMMON_SPEED, R_mult * COMMON_SPEED);


        vTaskDelay(pdMS_TO_TICKS(30));
    }
}
