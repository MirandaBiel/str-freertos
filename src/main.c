#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

// Definições dos pinos
#define RED_PIN     13
#define GREEN_PIN   11
#define BLUE_PIN    12
#define BUZZER_PIN  21
#define BUTTON_A    5   // Controla LED
#define BUTTON_B    6   // Controla Buzzer

// Handles das tarefas
TaskHandle_t xLedTaskHandle = NULL;
TaskHandle_t xBuzzerTaskHandle = NULL;

// Tarefa LED RGB: alterna cores a cada 500ms
void vLedTask(void *params) {
    while (true) {
        gpio_put(RED_PIN, 1); gpio_put(GREEN_PIN, 0); gpio_put(BLUE_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_put(RED_PIN, 0); gpio_put(GREEN_PIN, 1); gpio_put(BLUE_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_put(RED_PIN, 0); gpio_put(GREEN_PIN, 0); gpio_put(BLUE_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Tarefa do buzzer: bip a cada 1s
void vBuzzerTask(void *params) {
    while (true) {
        gpio_put(BUZZER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_put(BUZZER_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(900));
    }
}

// Tarefa para leitura dos botões
void vButtonTask(void *params) {
    bool led_suspended = false;
    bool buzzer_suspended = false;

    while (true) {
        if (!gpio_get(BUTTON_A)) {
            if (!led_suspended) {
                vTaskSuspend(xLedTaskHandle);
                led_suspended = true;
            } else {
                vTaskResume(xLedTaskHandle);
                led_suspended = false;
            }
            vTaskDelay(pdMS_TO_TICKS(300)); // debounce
        }

        if (!gpio_get(BUTTON_B)) {
            if (!buzzer_suspended) {
                vTaskSuspend(xBuzzerTaskHandle);
                buzzer_suspended = true;
            } else {
                vTaskResume(xBuzzerTaskHandle);
                buzzer_suspended = false;
            }
            vTaskDelay(pdMS_TO_TICKS(300)); // debounce
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // polling
    }
}

int main() {
    stdio_init_all();

    // Configuração dos GPIOs
    gpio_init(RED_PIN); gpio_set_dir(RED_PIN, GPIO_OUT);
    gpio_init(GREEN_PIN); gpio_set_dir(GREEN_PIN, GPIO_OUT);
    gpio_init(BLUE_PIN); gpio_set_dir(BLUE_PIN, GPIO_OUT);
    gpio_init(BUZZER_PIN); gpio_set_dir(BUZZER_PIN, GPIO_OUT);

    gpio_init(BUTTON_A); gpio_set_dir(BUTTON_A, GPIO_IN); gpio_pull_up(BUTTON_A);
    gpio_init(BUTTON_B); gpio_set_dir(BUTTON_B, GPIO_IN); gpio_pull_up(BUTTON_B);

    // Criação das tarefas
    xTaskCreate(vLedTask, "LED Task", 256, NULL, 1, &xLedTaskHandle);
    xTaskCreate(vBuzzerTask, "Buzzer Task", 256, NULL, 1, &xBuzzerTaskHandle);
    xTaskCreate(vButtonTask, "Button Task", 256, NULL, 2, NULL);

    // Inicia o agendador
    vTaskStartScheduler();

    while (true); // Nunca deve chegar aqui
}
