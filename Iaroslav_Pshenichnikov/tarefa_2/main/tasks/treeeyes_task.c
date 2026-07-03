#include "treeeyes_task.h"
#include "wheel.h"
#include "wcet.h"
#include "esp_task_wdt.h"

const static char *TAG = "main_app";

portTASK_FUNCTION(Treeeyes, args)
{
    //wcet_init(46, 47);
    const float threshold = 13.0f; //< aumentei um pouco pra ter margem de seguranca
    EventGroupHandle_t evt = (EventGroupHandle_t)args;
    TreeEyes_Init();
	//TreeEyes_DisableLeft();
    //TreeEyes_DisableRight();
    ultrasonic_value_t sensor[3];
    char *near_sensor_name;
    char *sensor_name[] = {"left", "middle", "right"};

    /* Inscreve este task no TWDT (uma vez, fora do loop) */
    esp_task_wdt_add(NULL);

    /* Período fixo (RMS): vTaskDelayUntil mantém T constante, sem drift */
    TickType_t last_wake = xTaskGetTickCount();

	while(1)
	{
        //wcet_begin(46, 47);
        TreeEyes_TrigAndWait(portMAX_DELAY);
        TreeEyes_Read(&sensor[0], &sensor[1], &sensor[2]);
        
        uint32_t min_ticks = 0xFFFFFFFF; 
        near_sensor_name = "none";

        for ( int i = 0; i < 3; i++ )
        {
            if (sensor[i].isUpdated == pdTRUE && sensor[i].tof_ticks < min_ticks) 
            {
                min_ticks = sensor[i].tof_ticks;
                near_sensor_name = sensor_name[i];
            }
        }

        float distance = (min_ticks * (1000000.0 / esp_clk_apb_freq())) / 58.0;

        /* Flag de parada por sonar (BIT5 do event group): levanta quando o objeto
         * mais próximo está abaixo do limiar; o speed_ctrl zera as velocidades.
         * Para evitar liberação por falso positivo (uma leitura ruidosa acima do
         * limiar), o flag só é derrubado após CLEAR_STREAK leituras consecutivas
         * acima do limiar. Qualquer leitura abaixo reinicia a contagem. */
        const int CLEAR_STREAK = 6;
        static int clear_count = 0;

        if (distance < threshold){ 
            xEventGroupSetBits(evt, BIT5);
            clear_count = 0;
        }
        else if (clear_count < CLEAR_STREAK){
            clear_count++;
            if (clear_count >= CLEAR_STREAK){
                xEventGroupClearBits(evt, BIT5);
            }
        }

        ESP_LOGI(TAG, "The sensor with the nearest detected object was: %s (Distance: %.2f cm)", near_sensor_name, distance);
        //printf("The sensor with the nearest detected object was: %s (Distance: %"PRIu32" ticks)\n", near_sensor_name, min_ticks);
    
        //wcet_end(47);
        esp_task_wdt_reset(); 
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(60));
    }
	
}