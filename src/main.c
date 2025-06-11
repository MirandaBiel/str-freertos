#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Display driver include
#include "ssd1306.h"
#include "ssd1306_i2c.h" 

// PROTÓTIPOS DAS FUNÇÕES DO neopixel.c
void npInit(uint pin, uint amount);
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b);
void npClear();
void npWrite();

// Definições dos pinos
#define RED_PIN     13
#define GREEN_PIN   11
#define BLUE_PIN    12
#define BUZZER_PIN  21
#define BUTTON_A    5
#define BUTTON_B    6
#define LED_PIN     7   
#define LED_COUNT   25  

// Definições do Display OLED
#define I2C_SDA     14
#define I2C_SCL     15
#define I2C_PORT    i2c1
#define OLED_ADDR   0x3C

// Globais do Display
uint8_t g_display_buffer[ssd1306_buffer_length];
struct render_area g_frame_area;

// Handles e Fila
TaskHandle_t xLedTaskHandle = NULL;
TaskHandle_t xBuzzerTaskHandle = NULL;
QueueHandle_t xDisplayQueue = NULL;

typedef struct {
    char line1[17];
    char line2[17];
} DisplayMessage;


// Protótipos
void init_peripherals();
void init_display();
int getIndex(int x, int y);


// Função de mapeamento da matriz
int getIndex(int x, int y) {
    if (y % 2 == 0) { return 24 - (y * 5 + x); } 
    else { return 24 - (y * 5 + (4 - x)); }
}

// Tarefa LED RGB E MATRIZ
void vLedTask(void *params) {
    const uint8_t BRIGHTNESS = 40;
    while (true) {
        gpio_put(RED_PIN, 1); gpio_put(GREEN_PIN, 0); gpio_put(BLUE_PIN, 0);
        for(int i = 0; i < LED_COUNT; i++) { npSetLED(i, BRIGHTNESS, 0, 0); }
        npWrite();
        vTaskDelay(pdMS_TO_TICKS(1000));

        gpio_put(RED_PIN, 0); gpio_put(GREEN_PIN, 1); gpio_put(BLUE_PIN, 0);
        for(int i = 0; i < LED_COUNT; i++) { npSetLED(i, 0, BRIGHTNESS, 0); }
        npWrite();
        vTaskDelay(pdMS_TO_TICKS(1000));

        gpio_put(RED_PIN, 0); gpio_put(GREEN_PIN, 0); gpio_put(BLUE_PIN, 1);
        for(int i = 0; i < LED_COUNT; i++) { npSetLED(i, 0, 0, BRIGHTNESS); }
        npWrite();
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        gpio_put(RED_PIN, 0); gpio_put(GREEN_PIN, 0); gpio_put(BLUE_PIN, 0);
        npClear();
        npWrite();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Tarefa do buzzer
void vBuzzerTask(void *params) {
    while (true) {
        gpio_put(BUZZER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_put(BUZZER_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(900));
    }
}

// Tarefa para gerenciar o display OLED
void vDisplayTask(void *params) {
    DisplayMessage msg;
    strcpy(msg.line1, "STR-FreeRTOS");
    strcpy(msg.line2, "System Ready");
    while (true) {
        memset(g_display_buffer, 0, sizeof(g_display_buffer));
        ssd1306_draw_string(g_display_buffer, 5, 16, msg.line1);
        ssd1306_draw_string(g_display_buffer, 5, 32, msg.line2);
        render_on_display(g_display_buffer, &g_frame_area);
        xQueueReceive(xDisplayQueue, &msg, portMAX_DELAY);
    }
}

// Tarefa para leitura dos botões
void vButtonTask(void *params) {
    bool led_suspended = false;
    bool buzzer_suspended = false;
    DisplayMessage msg;
    while (true) {
        if (!gpio_get(BUTTON_A)) {
            // --- CÓDIGO NOVO ---
            printf("Botao A Pressionado! Status LED: %s\n", led_suspended ? "RUNNING" : "PAUSED");

            led_suspended = !led_suspended;
            if (led_suspended) {
                vTaskSuspend(xLedTaskHandle);
                strcpy(msg.line1, "LED Task"); strcpy(msg.line2, "PAUSED");
            } else {
                vTaskResume(xLedTaskHandle);
                strcpy(msg.line1, "LED Task"); strcpy(msg.line2, "RUNNING");
            }
            xQueueSend(xDisplayQueue, &msg, 0);
            vTaskDelay(pdMS_TO_TICKS(300));
        }

        if (!gpio_get(BUTTON_B)) {
            // --- CÓDIGO NOVO ---
            printf("Botao B Pressionado! Status Buzzer: %s\n", buzzer_suspended ? "RUNNING" : "PAUSED");

            buzzer_suspended = !buzzer_suspended;
            if (buzzer_suspended) {
                vTaskSuspend(xBuzzerTaskHandle);
                strcpy(msg.line1, "Buzzer Task"); strcpy(msg.line2, "PAUSED");
            } else {
                vTaskResume(xBuzzerTaskHandle);
                strcpy(msg.line1, "Buzzer Task"); strcpy(msg.line2, "RUNNING");
            }
            xQueueSend(xDisplayQueue, &msg, 0);
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


int main() {
    stdio_init_all();
    // Adiciona um pequeno delay para dar tempo ao terminal de se conectar
    sleep_ms(2000); 
    printf("\n--== Projeto STR-FreeRTOS Iniciado ==--\n");

    init_peripherals();
    init_display();
    npInit(LED_PIN, LED_COUNT);

    xDisplayQueue = xQueueCreate(5, sizeof(DisplayMessage));

    xTaskCreate(vLedTask, "LED Task", 512, NULL, 1, &xLedTaskHandle);
    xTaskCreate(vBuzzerTask, "Buzzer Task", 256, NULL, 1, &xBuzzerTaskHandle);
    xTaskCreate(vButtonTask, "Button Task", 256, NULL, 2, NULL);
    xTaskCreate(vDisplayTask, "Display Task", 384, NULL, 3, NULL);

    vTaskStartScheduler();

    while (true);
}

void init_peripherals() {
    gpio_init(RED_PIN); gpio_set_dir(RED_PIN, GPIO_OUT);
    gpio_init(GREEN_PIN); gpio_set_dir(GREEN_PIN, GPIO_OUT);
    gpio_init(BLUE_PIN); gpio_set_dir(BLUE_PIN, GPIO_OUT);
    gpio_init(BUZZER_PIN); gpio_set_dir(BUZZER_PIN, GPIO_OUT);

    gpio_init(BUTTON_A); gpio_set_dir(BUTTON_A, GPIO_IN); gpio_pull_up(BUTTON_A);
    gpio_init(BUTTON_B); gpio_set_dir(BUTTON_B, GPIO_IN); gpio_pull_up(BUTTON_B);

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

void init_display() {
    ssd1306_init(I2C_PORT, OLED_ADDR, 128, 64);
    g_frame_area = (struct render_area){
        .start_column = 0, .end_column = ssd1306_width - 1,
        .start_page = 0, .end_page = ssd1306_n_pages - 1
    };
    calculate_render_area_buffer_length(&g_frame_area);
}
