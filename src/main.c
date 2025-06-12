/*
 * PROJETO: Simulador de Sistema de Controle Veicular em Tempo Real
 * DESCRIÇÃO: Implementa 7 tarefas veiculares (4 periódicas, 3 esporádicas)
 * utilizando FreeRTOS no RP2040, com simulação de sensores e
 * atuadores via periféricos da placa BitDogLab.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

// --- SDK Pico ---
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
//#include "hardware/spi.h"
#include "pico/binary_info.h"


// --- FreeRTOS ---
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h" // Para Semáforos
#include "queue.h"    // Para Filas

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
#define PIN_LED_IGNICAO     13 // LED Vermelho: Simula pulso de ignição
#define PIN_LED_FAROL       11 // LED Verde: Simula faróis ligados/desligados
#define PIN_LED_TEMP_ALERTA 12 // LED Azul: Simula alerta de superaquecimento
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
// Usados para sinalizar da ISR para a tarefa correspondente.
SemaphoreHandle_t g_sem_airbag = NULL;
SemaphoreHandle_t g_sem_abs = NULL;
SemaphoreHandle_t g_sem_piloto = NULL;

// --- Fila para comunicação com a tarefa do Display ---
QueueHandle_t g_queue_display = NULL;

// Estrutura da mensagem enviada para o display
typedef enum {
    UPDATE_VELOCIDADE,
    UPDATE_TEMP,
    UPDATE_COMANDO
} DisplaySource;

typedef struct {
    DisplaySource source;
    float value;
    char text[17];
} DisplayMessage;

// --- Estado global simulado ---
volatile bool g_farol_ligado = false;
volatile float g_nivel_combustivel = 100.0f; // Começa com 100%

// =============================================================================
// --- PROTÓTIPOS DAS TAREFAS E FUNÇÕES ---
// =============================================================================
void init_hardware();
void gpio_callback_isr(uint gpio, uint32_t events);

// --- Tarefas Periódicas ---
void task_monitor_rpm(void *params);
void task_monitor_velocidade(void *params);
void task_monitor_temperatura(void *params);
void task_monitor_combustivel(void *params);

// --- Tarefas Esporádicas / Aperiódicas ---
void task_logica_airbag(void *params);
void task_logica_abs(void *params);
void task_comando_piloto(void *params);

// --- Tarefa de Gerenciamento de UI ---
void task_gerencia_display(void *params);

// =============================================================================
// --- TAREFAS PERIÓDICAS ---
// =============================================================================

/**
 * @brief Tarefa 1: Monitora RPM e simula ignição. HRT.
 * @details Período: 5ms, Prioridade: 10
 */
void task_monitor_rpm(void *params) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5);

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Garante período exato
        
        adc_select_input(ADC_RPM);
        uint16_t rpm_raw = adc_read();
        
        // Simula pulso de ignição: pisca o LED se RPM > 0
        if (rpm_raw > 50) {
            gpio_put(PIN_LED_IGNICAO, 1);
            vTaskDelay(pdMS_TO_TICKS(1)); // Pulso curto
            gpio_put(PIN_LED_IGNICAO, 0);
        }
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
 * @brief Tarefa 3: Lê a temperatura e verifica alertas. SRT.
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

        // Lógica de alerta
        if (temp_c > 40.0f) { // Limite de alerta baixo para teste
            gpio_put(PIN_LED_TEMP_ALERTA, 1);
        } else {
            gpio_put(PIN_LED_TEMP_ALERTA, 0);
        }
    }
}

/**
 * @brief Tarefa 4: Monitora combustível e atualiza painel (matriz). SRT.
 * @details Período: 1000ms, Prioridade: 2
 */
void task_monitor_combustivel(void *params) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // Simula consumo de combustível
        g_nivel_combustivel -= 0.1f;
        if (g_nivel_combustivel < 0) g_nivel_combustivel = 100.0f; // Reabastece

        npClear();
        int leds_acesos = (int)(g_nivel_combustivel / 4.0f); // 100% / 25 LEDs = 4% por LED
        for (int i = 0; i < leds_acesos; i++) {
            if (i < 5) npSetLED(i, 0, 0, 50);     // Azul
            else if (i < 15) npSetLED(i, 0, 50, 0); // Verde
            else npSetLED(i, 50, 0, 0);             // Vermelho se baixo
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
    DisplayMessage msg;
    msg.source = UPDATE_COMANDO;
    strcpy(msg.text, "AIRBAG ATIVADO!");

    while (true) {
        // Fica bloqueada aqui indefinidamente até o semáforo ser liberado pela ISR
        xSemaphoreTake(g_sem_airbag, portMAX_DELAY);
        
        // --- AÇÃO CRÍTICA ---
        printf("EVENTO CRITICO: COLISAO DETECTADA! ACIONANDO AIRBAG...\n");
        xQueueSend(g_queue_display, &msg, 0);
        
        // Desativa outras tarefas para focar no evento crítico
        vTaskSuspendAll();

        // Sinalização de emergência (luz e som) e bloqueio do sistema
        gpio_put(PIN_BUZZER, 1);
        for (int i = 0; i < NEOPIXEL_COUNT; i++) npSetLED(i, 255, 255, 255);
        npWrite();
        
        // Sistema entra em estado seguro e não sai mais
        while(true);
    }
}

/**
 * @brief Tarefa 6: Lógica de controle do ABS. HRT.
 * @details Ativada por semáforo, Prioridade: 11
 */
void task_logica_abs(void *params) {
    DisplayMessage msg;
    msg.source = UPDATE_COMANDO;
    
    while (true) {
        xSemaphoreTake(g_sem_abs, portMAX_DELAY);
        
        // --- AÇÃO ---
        printf("EVENTO: RODA TRAVANDO! ABS ATIVADO...\n");
        strcpy(msg.text, "ABS ATIVO!");
        xQueueSend(g_queue_display, &msg, 0);

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

        strcpy(msg.text, "ABS DESATIVADO");
        xQueueSend(g_queue_display, &msg, 0);
    }
}

/**
 * @brief Tarefa 7: Processa comandos do piloto (Farol). SRT.
 * @details Ativada por semáforo, Prioridade: 4
 */
void task_comando_piloto(void *params) {
    DisplayMessage msg;
    msg.source = UPDATE_COMANDO;

    while (true) {
        xSemaphoreTake(g_sem_piloto, portMAX_DELAY);
        
        g_farol_ligado = !g_farol_ligado;
        gpio_put(PIN_LED_FAROL, g_farol_ligado);
        
        printf("COMANDO: Farois %s\n", g_farol_ligado ? "LIGADOS" : "DESLIGADOS");
        sprintf(msg.text, "Farois: %s", g_farol_ligado ? "ON" : "OFF");
        xQueueSend(g_queue_display, &msg, 0);
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
    char str_vel[17] = "Vel: --- km/h";
    char str_temp[17] = "Temp: --.- C";
    char str_cmd[17] = "Sistema OK";

    const TickType_t xFrequency = pdMS_TO_TICKS(100);//adicao da laura
    TickType_t xLastWakeTime = xTaskGetTickCount(); //adicao da laura
    while (true) {
        while (true) {
        // Tenta receber mensagens da fila, mas não bloqueia indefinidamente.
        // Se houver uma mensagem, ela é processada. Se não houver, a função retorna pdFAIL imediatamente.
        while (xQueueReceive(g_queue_display, &msg, 0) == pdPASS) { // Use 0 para não bloquear
            switch(msg.source) {
                case UPDATE_VELOCIDADE:
                    sprintf(str_vel, "Vel: %.0f km/h", msg.value);
                    break;
                case UPDATE_TEMP:
                    sprintf(str_temp, "Temp: %.1f C", msg.value);
                    break;
                case UPDATE_COMANDO:
                    strcpy(str_cmd, msg.text);
                    break;
            }
        }

        // Limpa o display
        memset(g_display_buffer, 0, ssd1306_buffer_length);

        // Desenha as informações atuais (que foram atualizadas pelas mensagens da fila)
        ssd1306_draw_string(g_display_buffer, 0, 0, "STR Veicular");
        ssd1306_draw_string(g_display_buffer, 0, 16, str_vel);
        ssd1306_draw_string(g_display_buffer, 0, 32, str_temp);
        ssd1306_draw_string(g_display_buffer, 0, 48, str_cmd);
        
        render_on_display(g_display_buffer, &g_frame_area);

        // Garante que a tarefa de display seja periódica, atualizando a cada 'xFrequency'
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    }
}

// =============================================================================
// --- ROTINA DE INTERRUPÇÃO E MAIN ---
// =============================================================================

/**
 * @brief Callback para TODAS as interrupções de GPIO.
 * @details Deve ser extremamente rápido. Apenas sinaliza o semáforo correto.
 */
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
    
    printf("\n--== STR Veicular com FreeRTOS Iniciado ==--\n");
    printf("Configurando tarefas e semaforos...\n");

    // --- Criação de Semáforos e Filas ---
    g_sem_airbag = xSemaphoreCreateBinary();
    g_sem_abs = xSemaphoreCreateBinary();
    g_sem_piloto = xSemaphoreCreateBinary();
    g_queue_display = xQueueCreate(10, sizeof(DisplayMessage));

    // --- Criação das Tarefas com Prioridades Definidas ---
    // Esporádicas (maiores prioridades)
    xTaskCreate(task_logica_airbag, "AirbagTask", 256, NULL, 12, NULL);
    xTaskCreate(task_logica_abs, "ABSTask", 256, NULL, 11, NULL);
    // Periódicas Críticas
    xTaskCreate(task_monitor_rpm, "RPMTask", 256, NULL, 10, NULL);
    xTaskCreate(task_gerencia_display, "DisplayTask", 512, NULL, 9, NULL);
    xTaskCreate(task_monitor_velocidade, "VelTask", 256, NULL, 8, NULL);
    // Periódicas e Aperiódicas menos críticas
    xTaskCreate(task_monitor_temperatura, "TempTask", 256, NULL, 5, NULL);
    xTaskCreate(task_comando_piloto, "PilotoTask", 256, NULL, 4, NULL);
    xTaskCreate(task_monitor_combustivel, "FuelTask", 512, NULL, 2, NULL);

    // --- Configuração das Interrupções de GPIO ---
    gpio_set_irq_enabled_with_callback(PIN_SENSOR_COLISAO, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_isr);
    gpio_set_irq_enabled_with_callback(PIN_SENSOR_ABS, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_isr);
    gpio_set_irq_enabled_with_callback(PIN_CMD_PILOTO, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_isr);

    printf("Sistema configurado. Iniciando o scheduler...\n");
    vTaskStartScheduler();

    // O código nunca deve chegar aqui
    while(true);
}

/**
 * @brief Inicializa todo o hardware necessário.
 */
void init_hardware() {
    stdio_init_all();
    sleep_ms(2000); // Pausa para permitir a conexão do terminal serial

    // --- Inicialização do ADC ---
    adc_init();
    adc_gpio_init(26 + ADC_VELOCIDADE); // Joystick X
    adc_gpio_init(26 + ADC_RPM);      // Joystick Y
    adc_set_temp_sensor_enabled(true);

    // --- Inicialização dos Pinos de Saída ---
    gpio_init(PIN_LED_IGNICAO); gpio_set_dir(PIN_LED_IGNICAO, GPIO_OUT);
    gpio_init(PIN_LED_FAROL); gpio_set_dir(PIN_LED_FAROL, GPIO_OUT);
    gpio_init(PIN_LED_TEMP_ALERTA); gpio_set_dir(PIN_LED_TEMP_ALERTA, GPIO_OUT);
    gpio_init(PIN_BUZZER); gpio_set_dir(PIN_BUZZER, GPIO_OUT);

    // --- Inicialização dos Pinos de Entrada (Interrupção) ---
    gpio_init(PIN_SENSOR_COLISAO); gpio_set_dir(PIN_SENSOR_COLISAO, GPIO_IN); gpio_pull_up(PIN_SENSOR_COLISAO);
    gpio_init(PIN_SENSOR_ABS); gpio_set_dir(PIN_SENSOR_ABS, GPIO_IN); gpio_pull_up(PIN_SENSOR_ABS);
    gpio_init(PIN_CMD_PILOTO); gpio_set_dir(PIN_CMD_PILOTO, GPIO_IN); gpio_pull_up(PIN_CMD_PILOTO);

    // --- Inicialização do Display OLED ---
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

    // --- Inicialização da Matriz NeoPixel ---
    npInit(PIN_NEOPIXEL, NEOPIXEL_COUNT);
    npClear();
    npWrite();
}
