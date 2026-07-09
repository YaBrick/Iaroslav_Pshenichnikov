/* O handler do TWDT (esp_task_wdt_isr_user_handler) foi movido para
 * tasks/wdt_handler_task.c: a ISR agora apenas notifica um task dedicado
 * (wdt_handler), que suspende speed_ctrl/wheel e coloca o robo em panico. */
