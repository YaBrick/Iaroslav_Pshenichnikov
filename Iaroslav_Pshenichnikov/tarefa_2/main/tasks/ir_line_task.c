#include "ir_line_task.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "wcet.h"

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

portTASK_FUNCTION(ir_line_ctrl, args)
{
    //wcet_init(46, 47);
    esp_task_wdt_add(NULL);
    gpio_config_t ir_line_config = {
        .pin_bit_mask = INFRA_RED_OUT_GPIO_MASK,
        .mode         = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&ir_line_config);
    EventGroupHandle_t evt = (EventGroupHandle_t)args;

    uint32_t gpioValue = 0;

    /* Período fixo (RMS): vTaskDelayUntil mantém T constante, sem drift */
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        //wcet_begin(46, 47);
        /* Lê os sensores — o sensor dá 0 quando a linha é detectada */
        gpioValue = (uint8_t)(
            (uint8_t)gpio_get_level(INFRA_RED_VERY_LEFT_GPIO)       |
            (uint8_t)gpio_get_level(INFRA_RED_LEFT_GPIO)        << 1 |
            (uint8_t)gpio_get_level(INFRA_RED_MIDDLE_GPIO)      << 2 |
            (uint8_t)gpio_get_level(INFRA_RED_RIGHT_GPIO)       << 3 |
            (uint8_t)gpio_get_level(INFRA_RED_VERY_RIGHT_GPIO)  << 4
        );

        xEventGroupClearBits(evt, 0x1F);
		xEventGroupSetBits(evt, gpioValue);
		//ESP_LOGI(TAG, "%lu", xEventGroupGetBits(evt));
        //wcet_end(47);
        esp_task_wdt_reset(); 
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
    }
}
