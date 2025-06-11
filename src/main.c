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
#include "ssd1306_i2c.h" // Adicionado para garantir a visibilidade das funções de renderização

// Definições dos pinos
#define RED_PIN     13
#define GREEN_PIN   11
#define BLUE_PIN    12
#define BUZZER_PIN  21
#define BUTTON_A    5   // Controla LED
#define BUTTON_B    6   // Controla Buzzer

// Definições do Display OLED
#define I2C_SDA     14
#define I2C_SCL     15
#define I2C_PORT    i2c1
#define OLED_ADDR   0x3C

// --- Buffer global e área de renderização para o display (necessário para o seu driver) ---
uint8_t g_display_buffer[ssd1306_buffer_length];
struct render_area g_frame_area;

// Handles das tarefas e da fila
TaskHandle_t xLedTaskHandle = NULL;
TaskHandle_t xBuzzerTaskHandle = NULL;
QueueHandle_t xDisplayQueue = NULL;

// Estrutura da mensagem para a fila do display
typedef struct {
    char line1[17]; // 16 chars + null terminator
    char line2[17];
} DisplayMessage;


// Protótipos de funções
void init_peripherals();
void init_display();


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

// Tarefa para gerenciar o display OLED
void vDisplayTask(void *params) {
    DisplayMessage msg;
    // Mensagem inicial
    strcpy(msg.line1, "STR-FreeRTOS");
    strcpy(msg.line2, "System Ready");

    while (true) {
        // 1. Limpa o buffer de memória
        memset(g_display_buffer, 0, sizeof(g_display_buffer));

        // 2. Desenha as strings no buffer (agora com os parâmetros corretos)
        ssd1306_draw_string(g_display_buffer, 5, 16, msg.line1);
        ssd1306_draw_string(g_display_buffer, 5, 32, msg.line2);

        // 3. Envia o buffer para o display físico
        render_on_display(g_display_buffer, &g_frame_area);
        
        // Aguarda por uma nova mensagem na fila, sem consumir CPU.
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
            led_suspended = !led_suspended;
            if (led_suspended) {
                vTaskSuspend(xLedTaskHandle);
                strcpy(msg.line1, "LED Task");
                strcpy(msg.line2, "PAUSED");
            } else {
                vTaskResume(xLedTaskHandle);
                strcpy(msg.line1, "LED Task");
                strcpy(msg.line2, "RUNNING");
            }
            xQueueSend(xDisplayQueue, &msg, 0);
            vTaskDelay(pdMS_TO_TICKS(300)); // debounce
        }

        if (!gpio_get(BUTTON_B)) {
            buzzer_suspended = !buzzer_suspended;
            if (buzzer_suspended) {
                vTaskSuspend(xBuzzerTaskHandle);
                strcpy(msg.line1, "Buzzer Task");
                strcpy(msg.line2, "PAUSED");
            } else {
                vTaskResume(xBuzzerTaskHandle);
                strcpy(msg.line1, "Buzzer Task");
                strcpy(msg.line2, "RUNNING");
            }
            xQueueSend(xDisplayQueue, &msg, 0);
            vTaskDelay(pdMS_TO_TICKS(300)); // debounce
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // polling
    }
}


int main() {
    stdio_init_all();
    init_peripherals();
    init_display();

    xDisplayQueue = xQueueCreate(5, sizeof(DisplayMessage));

    xTaskCreate(vLedTask, "LED Task", 256, NULL, 1, &xLedTaskHandle);
    xTaskCreate(vBuzzerTask, "Buzzer Task", 256, NULL, 1, &xBuzzerTaskHandle);
    xTaskCreate(vButtonTask, "Button Task", 256, NULL, 2, NULL);
    xTaskCreate(vDisplayTask, "Display Task", 256, NULL, 3, NULL);

    vTaskStartScheduler();

    while (true);
}

void init_peripherals() {
    // GPIOs de Saída
    gpio_init(RED_PIN); gpio_set_dir(RED_PIN, GPIO_OUT);
    gpio_init(GREEN_PIN); gpio_set_dir(GREEN_PIN, GPIO_OUT);
    gpio_init(BLUE_PIN); gpio_set_dir(BLUE_PIN, GPIO_OUT);
    gpio_init(BUZZER_PIN); gpio_set_dir(BUZZER_PIN, GPIO_OUT);

    // GPIOs de Entrada
    gpio_init(BUTTON_A); gpio_set_dir(BUTTON_A, GPIO_IN); gpio_pull_up(BUTTON_A);
    gpio_init(BUTTON_B); gpio_set_dir(BUTTON_B, GPIO_IN); gpio_pull_up(BUTTON_B);

    // I2C para o Display
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

void init_display() {
    // Inicializa o display e a área de renderização
    ssd1306_init(I2C_PORT, OLED_ADDR, 128, 64);
    
    g_frame_area = (struct render_area){
        .start_column = 0,
        .end_column = ssd1306_width - 1,
        .start_page = 0,
        .end_page = ssd1306_n_pages - 1
    };
    
    // Esta função pode ou não ser necessária dependendo do seu driver,
    // mas estava no seu exemplo 'reading_temp'.
    calculate_render_area_buffer_length(&g_frame_area);
}