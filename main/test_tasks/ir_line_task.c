#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
//#include "led_strip.h"
#include "sdkconfig.h"
#include "soc/gpio_reg.h"
#include "ir_line_task.h"
#include "mission_control.h"
#include "freertos/event_groups.h"

static const char *TAG = "IR_Line";

#define INFRA_RED_VERY_LEFT_GPIO  GPIO_NUM_15
#define INFRA_RED_LEFT_GPIO  GPIO_NUM_16
#define INFRA_RED_MIDDLE_GPIO  GPIO_NUM_17
#define INFRA_RED_RIGHT_GPIO  18
#define INFRA_RED_VERY_RIGHT_GPIO 8

#define IR_LINE_VERY_VERY_LEFT (1ULL << INFRA_RED_VERY_LEFT_GPIO)
#define IR_LINE_VERY_LEFT ((1ULL << INFRA_RED_VERY_LEFT_GPIO) | (1ULL << INFRA_RED_LEFT_GPIO))
#define IR_LINE_LEFT (1ULL << INFRA_RED_LEFT_GPIO)
#define IR_LINE_LEFT_MIDDLE ((1ULL << INFRA_RED_LEFT_GPIO) | (1ULL << INFRA_RED_MIDDLE_GPIO))
#define IR_LINE_MIDDLE (1ULL << INFRA_RED_MIDDLE_GPIO)
#define IR_LINE_RIGHT_MIDDLE ((1ULL << INFRA_RED_MIDDLE_GPIO) | (1ULL << INFRA_RED_RIGHT_GPIO))
#define IR_LINE_RIGHT (1ULL << INFRA_RED_RIGHT_GPIO)
#define IR_LINE_VERY_RIGHT ((1ULL << INFRA_RED_RIGHT_GPIO) | (1ULL << INFRA_RED_VERY_RIGHT_GPIO))
#define IR_LINE_VERY_VERY_RIGHT (1ULL << INFRA_RED_VERY_RIGHT_GPIO)

#define INFRA_RED_OUT_GPIO_MASK ((uint64_t)((1ULL << INFRA_RED_VERY_LEFT_GPIO) | (1ULL << INFRA_RED_LEFT_GPIO) | (1ULL << INFRA_RED_MIDDLE_GPIO) | (1ULL << INFRA_RED_RIGHT_GPIO) | (1ULL << INFRA_RED_VERY_RIGHT_GPIO)))


portTASK_FUNCTION(ir_line_ctrl, args)
{
	gpio_config_t ir_line_config = {
			.pin_bit_mask = INFRA_RED_OUT_GPIO_MASK,
			.mode = GPIO_MODE_INPUT,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
			.pull_up_en = GPIO_PULLUP_DISABLE,
			.intr_type = GPIO_INTR_DISABLE
	};

	gpio_config( &ir_line_config );

	uint64_t gpioValue;
	while (1) {
		gpioValue = (uint64_t)gpio_get_level(INFRA_RED_VERY_LEFT_GPIO) |
				(uint64_t)gpio_get_level(INFRA_RED_LEFT_GPIO) << 1 |
				(uint64_t)gpio_get_level(INFRA_RED_MIDDLE_GPIO) << 2 |
				(uint64_t)gpio_get_level(INFRA_RED_RIGHT_GPIO) << 3 |
				(uint64_t)gpio_get_level(INFRA_RED_VERY_RIGHT_GPIO) << 4;

		//gpioValue &= INFRA_RED_OUT_GPIO_MASK;

        //ESP_LOGI(TAG, "gpio value: %llu", gpioValue);
		//ESP_LOGI(TAG, "gpio_value: %x", gpioValue);
		//gpioValue = (REG_READ(GPIO_IN_REG) & INFRA_RED_OUT_GPIO_MASK);
		xEventGroupClearBits(xEvents, ((1ULL << 24) - 1));
		xEventGroupSetBits(xEvents, gpioValue);
		ESP_LOGI(TAG, "%lu", xEventGroupGetBits(xEvents)); //clearBits !!!!
		/*
		switch(gpioValue)
		{
		case IR_LINE_VERY_VERY_LEFT:
			ESP_LOGI(TAG, "GPIO VERY VERY LEFT!");
			xEventGroupSetBits(xEvents, IR_VERY_VERY_LEFT_FLAG);
		case IR_LINE_LEFT:
			ESP_LOGI(TAG, "GPIO LEFT!");
			xEventGroupSetBits(xEvents, IR_LEFT_FLAG);
		case IR_LINE_MIDDLE:
			ESP_LOGI(TAG, "GPIO MIDDLE!");
			xEventGroupSetBits(xEvents, IR_MIDDLE_FLAG);
		case IR_LINE_RIGHT:
			ESP_LOGI(TAG, "GPIO RIGHT!");
			xEventGroupSetBits(xEvents, IR_RIGHT_FLAG);
		case IR_LINE_VERY_VERY_RIGHT:
			ESP_LOGI(TAG, "GPIO VERY VERY RIGHT!");
			xEventGroupSetBits(xEvents, IR_VERY_VERY_RIGHT_FLAG);
		default:
			ESP_LOGI(TAG, "NOTHING!");
			break;
		}
			*/
         vTaskDelay(pdMS_TO_TICKS(50));
    }
}
