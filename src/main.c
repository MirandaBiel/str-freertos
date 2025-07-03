/*
 * PROJETO: Simulador de Sistema de Controle Veicular em Tempo Real
 * DESCRIÇÃO: Implementa 7 tarefas veiculares conforme o planejamento,
 * utilizando FreeRTOS no RP2040 e uma tarefa dedicada para o display OLED.
 * VERSÃO: Ajustada para seguir o planejamento original.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

// --- SDK Pico ---
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

// --- FreeRTOS ---
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

// --- Drivers de Periféricos ---
#include "ssd1306.h"
#include "ssd1306_i2c.h"

// Protótipos das funções do neopixel.c
void npInit(uint pin, uint amount);
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b);
void npClear();
void npWrite();

// =============================================================================
// --- DEFINIÇÕES E MAPEAMENTO DE HARDWARE (SIMULAÇÃO) ---
// =============================================================================

// --- Pinos dos Atuadores Simulados ---
#define PIN_LED_FAROL       11 // LED Verde: Simula faróis ligados/desligados
#define PIN_BUZZER          21 // Buzzer para alertas sonoros
#define PIN_NEOPIXEL        7  // Matriz de LEDs
#define NEOPIXEL_COUNT      25

// --- Pinos dos Sensores/Gatilhos Simulados ---
#define PIN_SENSOR_COLISAO  5  // Botão A: Dispara o airbag
#define PIN_SENSOR_ABS      6  // Botão B: Dispara a lógica do ABS
#define PIN_CMD_PILOTO      22 // Botão do Joystick: Aciona faróis

// Canais ADC para sensores analógicos simulados
#define ADC_RPM             1  // Eixo Y do Joystick (GP27) -> Rotação do motor
#define ADC_VELOCIDADE      0  // Eixo X do Joystick (GP26) -> Velocidade
#define ADC_TEMP            4  // Sensor de temperatura interno do RP2040

// --- Configurações do Display OLED ---
#define I2C_SDA     14
#define I2C_SCL     15
#define I2C_PORT    i2c1
#define OLED_ADDR   0x3C
uint8_t g_display_buffer[ssd1306_buffer_length];
struct render_area g_frame_area;

// =============================================================================
// --- ESTRUTURAS E VARIÁVEIS GLOBAIS DO RTOS ---
// =============================================================================

// --- Semáforos para tarefas esporádicas/aperiódicas ---
SemaphoreHandle_t g_sem_airbag = NULL;
SemaphoreHandle_t g_sem_abs = NULL;
SemaphoreHandle_t g_sem_piloto = NULL;

// --- Fila para comunicação com a tarefa do Display ---
QueueHandle_t g_queue_display = NULL;

// Estrutura da mensagem enviada para o display
typedef enum {
    UPDATE_RPM, // MODIFICADO: Adicionado para o RPM
    UPDATE_VELOCIDADE,
    UPDATE_TEMP
} DisplaySource;

typedef struct {
    DisplaySource source;
    float value;
} DisplayMessage;

// --- Estado global simulado ---
volatile bool g_farol_ligado = false;
// MODIFICADO: Valor de combustível agora é fixo, como no plano.
volatile float g_nivel_combustivel = 80.0f; // Fixo em 80% para teste

// =============================================================================
// --- PROTÓTIPOS DAS TAREFAS E FUNÇÕES ---
// =============================================================================
void init_hardware();
void gpio_callback_isr(uint gpio, uint32_t events);

void task_monitor_rpm(void *params);
void task_monitor_velocidade(void *params);
void task_monitor_temperatura(void *params);
void task_monitor_combustivel(void *params);
void task_logica_airbag(void *params);
void task_logica_abs(void *params);
void task_comando_piloto(void *params);
void task_gerencia_display(void *params);

// =============================================================================
// --- TAREFAS PERIÓDICAS ---
// =============================================================================

/**
 * @brief Tarefa 1: Monitora RPM e envia para o display. HRT.
 * @details Período: 5ms, Prioridade: 10
 */
void task_monitor_rpm(void *params) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5);
    DisplayMessage msg;
    msg.source = UPDATE_RPM;

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        adc_select_input(ADC_RPM);
        uint16_t rpm_raw = adc_read();
        
        // MODIFICADO: Mapeia o valor do ADC (0-4095) para RPM (0-8000) e envia para o display.
        msg.value = (float)rpm_raw * 8000.0f / 4095.0f;
        xQueueSend(g_queue_display, &msg, 0);
    }
}

/**
 * @brief Tarefa 2: Lê a velocidade e envia para o display. HRT.
 * @details Período: 25ms, Prioridade: 8
 */
void task_monitor_velocidade(void *params) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(25);
    DisplayMessage msg;
    msg.source = UPDATE_VELOCIDADE;

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        adc_select_input(ADC_VELOCIDADE);
        uint16_t vel_raw = adc_read();
        msg.value = (float)vel_raw * 250.0f / 4095.0f; // Mapeia para 0-250 km/h

        xQueueSend(g_queue_display, &msg, 0);
    }
}

/**
 * @brief Tarefa 3: Lê a temperatura e envia para o display. SRT.
 * @details Período: 500ms, Prioridade: 5
 */
void task_monitor_temperatura(void *params) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500);
    DisplayMessage msg;
    msg.source = UPDATE_TEMP;
    const float CONVERSION_FACTOR = 3.3f / (1 << 12);

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        adc_select_input(ADC_TEMP);
        uint16_t temp_raw = adc_read();
        float voltage = temp_raw * CONVERSION_FACTOR;
        float temp_c = 27.0f - (voltage - 0.706f) / 0.001721f;

        msg.value = temp_c;
        xQueueSend(g_queue_display, &msg, 0);

        // REMOVIDO: A lógica de alerta do LED azul foi removida para seguir o plano.
    }
}

/**
 * @brief Tarefa 4: Exibe o nível de combustível fixo na matriz. SRT.
 * @details Período: 1000ms, Prioridade: 2
 */
void task_monitor_combustivel(void *params) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);

    // Primeira atualização ao iniciar
    npClear();
    int leds_acesos = (int)(g_nivel_combustivel / 4.0f); // 100% / 25 LEDs = 4% por LED
    for (int i = 0; i < leds_acesos; i++) {
        if (i < 5) npSetLED(i, 0, 0, 50);      // Azul
        else if (i < 15) npSetLED(i, 0, 50, 0); // Verde
        else npSetLED(i, 50, 0, 0);             // Vermelho se baixo
    }
    npWrite();

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        // REMOVIDO: Simulação de consumo de combustível removida.
        // A tarefa agora apenas garante que o display de combustível permaneça
        // aceso, caso outra tarefa (ABS/Airbag) o apague.
        // (Re-desenha a cada 1s)
        npClear();
        leds_acesos = (int)(g_nivel_combustivel / 4.0f);
        for (int i = 0; i < leds_acesos; i++) {
             if (i < 5) npSetLED(i, 0, 0, 50);
             else if (i < 15) npSetLED(i, 0, 50, 0);
             else npSetLED(i, 50, 0, 0);
        }
        npWrite();
    }
}

// =============================================================================
// --- TAREFAS ESPORÁDICAS E APERIÓDICAS ---
// =============================================================================

/**
 * @brief Tarefa 5: Lógica de acionamento do Airbag. HRT.
 * @details Ativada por semáforo, Prioridade: 12 (Crítica)
 */
void task_logica_airbag(void *params) {
    while (true) {
        xSemaphoreTake(g_sem_airbag, portMAX_DELAY);
        
        printf("EVENTO CRITICO: COLISAO DETECTADA! ACIONANDO AIRBAG...\n");
        // REMOVIDO: Envio de mensagem para o display.

        // Ação crítica de acordo com o plano
        portDISABLE_INTERRUPTS();
        gpio_put(PIN_BUZZER, 1);
        for (int i = 0; i < NEOPIXEL_COUNT; i++) npSetLED(i, 255, 255, 255);
        npWrite();
        portENABLE_INTERRUPTS();
        
        // Bloqueio do sistema em loop infinito, como planejado.
        while(true);
    }
}

/**
 * @brief Tarefa 6: Lógica de controle do ABS. HRT.
 * @details Ativada por semáforo, Prioridade: 11
 */
void task_logica_abs(void *params) {
    while (true) {
        xSemaphoreTake(g_sem_abs, portMAX_DELAY);
        
        printf("EVENTO: RODA TRAVANDO! ABS ATIVADO...\n");
        // REMOVIDO: Envio de mensagem para o display.

        // Simula modulação do freio por 2 segundos
        for(int i = 0; i < 10; i++) {
            gpio_put(PIN_BUZZER, 1);
            for(int j=0; j<NEOPIXEL_COUNT; j++) npSetLED(j, 200, 100, 0); // Amarelo
            npWrite();
            vTaskDelay(pdMS_TO_TICKS(100));

            gpio_put(PIN_BUZZER, 0);
            npClear();
            npWrite();
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/**
 * @brief Tarefa 7: Processa comandos do piloto (Farol). SRT.
 * @details Ativada por semáforo, Prioridade: 4
 */
void task_comando_piloto(void *params) {
    while (true) {
        xSemaphoreTake(g_sem_piloto, portMAX_DELAY);
        
        g_farol_ligado = !g_farol_ligado;
        gpio_put(PIN_LED_FAROL, g_farol_ligado);
        
        printf("COMANDO: Farois %s\n", g_farol_ligado ? "LIGADOS" : "DESLIGADOS");
        // REMOVIDO: Envio de mensagem para o display.
    }
}

// =============================================================================
// --- TAREFA DE GERENCIAMENTO DE UI ---
// =============================================================================

/**
 * @brief Gerencia todas as atualizações do display OLED.
 * @details Recebe mensagens de outras tarefas via fila. Prioridade: 9
 */
void task_gerencia_display(void *params) {
    DisplayMessage msg;
    // MODIFICADO: Ajuste das strings para o novo layout do display
    char str_rpm[17] = "RPM: ----";
    char str_vel[17] = "Vel: --- km/h";
    char str_temp[17] = "Temp: --.- C";

    const TickType_t xFrequency = pdMS_TO_TICKS(100); // Taxa de atualização da tela
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Esvazia a fila de todas as mensagens pendentes antes de redesenhar
        while (xQueueReceive(g_queue_display, &msg, 0) == pdPASS) {
            switch(msg.source) {
                case UPDATE_RPM: // MODIFICADO: Adicionado case para RPM
                    sprintf(str_rpm, "RPM: %.0f", msg.value);
                    break;
                case UPDATE_VELOCIDADE:
                    sprintf(str_vel, "Vel: %.0f km/h", msg.value);
                    break;
                case UPDATE_TEMP:
                    sprintf(str_temp, "Temp: %.1f C", msg.value);
                    break;
            }
        }

        // Desenha as informações atuais no buffer
        memset(g_display_buffer, 0, ssd1306_buffer_length);
        ssd1306_draw_string(g_display_buffer, 0, 0, "STR Veicular");
        ssd1306_draw_string(g_display_buffer, 0, 16, str_rpm);
        ssd1306_draw_string(g_display_buffer, 0, 32, str_vel);
        ssd1306_draw_string(g_display_buffer, 0, 48, str_temp);
        
        // Envia o buffer para o display físico
        render_on_display(g_display_buffer, &g_frame_area);
    }
}

// =============================================================================
// --- ROTINA DE INTERRUPÇÃO E MAIN ---
// =============================================================================

void gpio_callback_isr(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (gpio == PIN_SENSOR_COLISAO) {
        xSemaphoreGiveFromISR(g_sem_airbag, &xHigherPriorityTaskWoken);
    } else if (gpio == PIN_SENSOR_ABS) {
        xSemaphoreGiveFromISR(g_sem_abs, &xHigherPriorityTaskWoken);
    } else if (gpio == PIN_CMD_PILOTO) {
        xSemaphoreGiveFromISR(g_sem_piloto, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

int main() {
    init_hardware();
    
    printf("\n--== STR Veicular com FreeRTOS (Versao Planejamento) ==--\n");
    printf("Configurando tarefas e semaforos...\n");

    g_sem_airbag = xSemaphoreCreateBinary();
    g_sem_abs = xSemaphoreCreateBinary();
    g_sem_piloto = xSemaphoreCreateBinary();
    g_queue_display = xQueueCreate(10, sizeof(DisplayMessage));

    // As prioridades e stacks são mantidos do código anterior
    xTaskCreate(task_logica_airbag, "AirbagTask", 256, NULL, 12, NULL);
    xTaskCreate(task_logica_abs, "ABSTask", 256, NULL, 11, NULL);
    xTaskCreate(task_monitor_rpm, "RPMTask", 256, NULL, 10, NULL);
    xTaskCreate(task_gerencia_display, "DisplayTask", 512, NULL, 9, NULL);
    xTaskCreate(task_monitor_velocidade, "VelTask", 256, NULL, 8, NULL);
    xTaskCreate(task_monitor_temperatura, "TempTask", 256, NULL, 5, NULL);
    xTaskCreate(task_comando_piloto, "PilotoTask", 256, NULL, 4, NULL);
    xTaskCreate(task_monitor_combustivel, "FuelTask", 512, NULL, 2, NULL);

    gpio_set_irq_enabled_with_callback(PIN_SENSOR_COLISAO, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_isr);
    gpio_set_irq_enabled_with_callback(PIN_SENSOR_ABS, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_isr);
    gpio_set_irq_enabled_with_callback(PIN_CMD_PILOTO, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_isr);

    printf("Sistema configurado. Iniciando o scheduler...\n");
    vTaskStartScheduler();

    while(true);
}


void init_hardware() {
    stdio_init_all();
    sleep_ms(2000);

    adc_init();
    adc_gpio_init(26 + ADC_VELOCIDADE); // Joystick X
    adc_gpio_init(26 + ADC_RPM);       // Joystick Y
    adc_set_temp_sensor_enabled(true);

    // MODIFICADO: Removidos os LEDs de ignição e alerta de temp
    gpio_init(PIN_LED_FAROL); gpio_set_dir(PIN_LED_FAROL, GPIO_OUT);
    gpio_init(PIN_BUZZER); gpio_set_dir(PIN_BUZZER, GPIO_OUT);

    gpio_init(PIN_SENSOR_COLISAO); gpio_set_dir(PIN_SENSOR_COLISAO, GPIO_IN); gpio_pull_up(PIN_SENSOR_COLISAO);
    gpio_init(PIN_SENSOR_ABS); gpio_set_dir(PIN_SENSOR_ABS, GPIO_IN); gpio_pull_up(PIN_SENSOR_ABS);
    gpio_init(PIN_CMD_PILOTO); gpio_set_dir(PIN_CMD_PILOTO, GPIO_IN); gpio_pull_up(PIN_CMD_PILOTO);

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    ssd1306_init(I2C_PORT, OLED_ADDR, 128, 64);
    g_frame_area = (struct render_area){
        .start_column = 0, .end_column = ssd1306_width - 1,
        .start_page = 0, .end_page = ssd1306_n_pages - 1
    };
    calculate_render_area_buffer_length(&g_frame_area);

    npInit(PIN_NEOPIXEL, NEOPIXEL_COUNT);
    npClear();
    npWrite();
}