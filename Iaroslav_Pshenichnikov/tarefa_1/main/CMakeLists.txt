idf_component_register(SRCS "wheel.c" "main.c" "mission_control.c" "mpu6050.c" "treeeyes.c" "tasks/treeeyes_task.c" "tasks/imu_task.c" "tasks/wheel_task.c" "tasks/ir_line_task.c"
                    INCLUDE_DIRS "." "./tasks"
                    REQUIRES bdc_motor driver esp_adc esp_timer)
