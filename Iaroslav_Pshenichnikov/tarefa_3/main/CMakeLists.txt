idf_component_register(SRCS "wheel.c" "main.c" "treeeyes.c" "tasks/treeeyes_task.c" "tasks/wheel_task.c" "tasks/ir_line_task.c" "tasks/speed_ctrl_task.c" "tasks/wdt_handler_task.c"
                    INCLUDE_DIRS "." "./tasks"
                    REQUIRES bdc_motor driver esp_adc esp_timer)
