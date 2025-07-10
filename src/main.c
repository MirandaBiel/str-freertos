#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/timer.h" 
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "pico/binary_info.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "ssd1306.h"
#include "ssd1306_i2c.h"

void npInit(uint pin, uint amount);
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b);
void npClear();
void npWrite();

#define PIN_LED_FAROL       11
#define PIN_BUZZER          21
#define PIN_NEOPIXEL        7
#define NEOPIXEL_COUNT      25
#define PIN_SENSOR_COLISAO  22
#define PIN_SENSOR_ABS      6
#define PIN_CMD_PILOTO      5
#define ADC_COMBUSTIVEL     1
#define ADC_VELOCIDADE      0
#define ADC_TEMP            4
#define MIC_CHANNEL         2
#define MIC_PIN             (26 + MIC_CHANNEL)
#define ADC_CLOCK_DIV       96.f
#define MIC_SAMPLES         200
#define I2C_SDA             14
#define I2C_SCL             15
#define I2C_PORT            i2c1
#define OLED_ADDR           0x3C
#define ANALYSIS_RUNTIME_S  60*5

uint8_t g_display_buffer[ssd1306_buffer_length];
struct render_area g_frame_area;
uint g_dma_channel;
dma_channel_config g_dma_cfg;
uint16_t g_mic_buffer[MIC_SAMPLES];

typedef enum { STATE_NORMAL, STATE_ABS_ACTIVE, STATE_AIRBAG_ACTIVE } SystemState;
volatile SystemState g_system_state = STATE_NORMAL;

SemaphoreHandle_t g_sem_airbag = NULL, g_sem_abs = NULL, g_sem_piloto = NULL;
QueueHandle_t g_queue_display = NULL;
SemaphoreHandle_t g_mutex_system_state = NULL, g_sem_report_trigger = NULL;
typedef enum { UPDATE_RPM, UPDATE_VELOCIDADE, UPDATE_TEMP } DisplaySource;
typedef struct { DisplaySource source; float value; } DisplayMessage;
volatile bool g_farol_ligado = false;

volatile uint32_t g_max_ct_airbag_us=0, g_max_ct_abs_us=0, g_max_ct_rpm_us=0, g_max_ct_display_us=0, g_max_ct_vel_us=0, g_max_ct_debounce_us=0, g_max_ct_temp_us=0, g_max_ct_piloto_us=0, g_max_ct_fuel_us=0;
volatile uint32_t g_max_resp_time_airbag_us=0, g_max_resp_time_abs_us=0, g_max_resp_time_rpm_us=0, g_max_resp_time_display_us=0, g_max_resp_time_vel_us=0, g_max_resp_time_debounce_us=0, g_max_resp_time_temp_us=0, g_max_resp_time_piloto_us=0, g_max_resp_time_fuel_us=0;
volatile uint64_t g_trigger_time_airbag_us=0, g_trigger_time_abs_us=0, g_trigger_time_piloto_us=0;

void init_hardware();
void gpio_callback_isr(uint gpio, uint32_t events);
void task_logica_airbag(void *params);
void task_logica_abs(void *params);
void task_monitor_rpm(void *params);
void task_debounce_buttons(void *params);
void task_comando_piloto(void *params);
void task_monitor_velocidade(void *params);
void task_monitor_temperatura(void *params);
void task_gerencia_display(void *params);
void task_monitor_combustivel(void *params);
void task_analysis_report(void *params);
int64_t report_timer_callback(alarm_id_t id, void *user_data);
void sample_mic() { adc_fifo_drain(); adc_run(false); dma_channel_configure(g_dma_channel, &g_dma_cfg, g_mic_buffer, &adc_hw->fifo, MIC_SAMPLES, true); adc_run(true); dma_channel_wait_for_finish_blocking(g_dma_channel); adc_run(false); }
float mic_power() { float avg = 0.f; for (uint i = 0; i < MIC_SAMPLES; ++i) { avg += g_mic_buffer[i] * g_mic_buffer[i]; } avg /= MIC_SAMPLES; return sqrt(avg); }
void buzzer_set_siren_tone(uint freq) { uint slice_num = pwm_gpio_to_slice_num(PIN_BUZZER); if (freq == 0) { pwm_set_enabled(slice_num, false); } else { uint32_t wrap = 500000 / freq - 1; pwm_set_wrap(slice_num, wrap); pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PIN_BUZZER), wrap / 2); pwm_set_enabled(slice_num, true); } }

void task_analysis_report(void *params) {
    xSemaphoreTake(g_sem_report_trigger, portMAX_DELAY);
    printf("\n\n---====== RELATORIO DE PERFORMANCE (CT & WCRT) APOS %d S ======---\n", ANALYSIS_RUNTIME_S);
    printf("--- (Valores em microssegundos - us) ---\n");
    printf("----------------------------------------------------------------------\n");
    printf("Tarefa\t\t\t| CT (Execucao)\t\t| WCRT (Resposta)\n");
    printf("------------------------|-----------------------|---------------------\n");
    printf("AirbagTask\t\t| %-21lu | %lu\n", g_max_ct_airbag_us, g_max_resp_time_airbag_us);
    printf("ABSTask\t\t\t| %-21lu | %lu\n", g_max_ct_abs_us, g_max_resp_time_abs_us);
    printf("PilotoTask\t\t| %-21lu | %lu\n", g_max_ct_piloto_us, g_max_resp_time_piloto_us);
    printf("DebounceTask\t\t| %-21lu | %lu\n", g_max_ct_debounce_us, g_max_resp_time_debounce_us);
    printf("DisplayTask\t\t| %-21lu | %lu\n", g_max_ct_display_us, g_max_resp_time_display_us);
    printf("RPMTask\t\t\t| %-21lu | %lu\n", g_max_ct_rpm_us, g_max_resp_time_rpm_us);
    printf("VelTask\t\t\t| %-21lu | %lu\n", g_max_ct_vel_us, g_max_resp_time_vel_us);
    printf("TempTask\t\t| %-21lu | %lu\n", g_max_ct_temp_us, g_max_resp_time_temp_us);
    printf("FuelTask\t\t| %-21lu | %lu\n", g_max_ct_fuel_us, g_max_resp_time_fuel_us);
    printf("----------------------------------------------------------------------\n");
    printf("--- Analise concluida. O sistema sera travado. ---\n");
    taskDISABLE_INTERRUPTS();
    while(1);
}

void task_logica_airbag(void *params) {
    while (true) {
        xSemaphoreTake(g_sem_airbag, portMAX_DELAY);
        uint64_t start_resp_time = g_trigger_time_airbag_us;
        uint64_t start_ct = time_us_64();
        
        if (xSemaphoreTake(g_mutex_system_state, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_system_state = STATE_AIRBAG_ACTIVE;
            xSemaphoreGive(g_mutex_system_state);
        }
        printf("EVENTO CRITICO: COLISAO DETECTADA! ACIONANDO PROTOCOLO DE SEGURANCA...\n");
        DisplayMessage msg_rpm = {.source = UPDATE_RPM, .value = 0.0f};
        DisplayMessage msg_vel = {.source = UPDATE_VELOCIDADE, .value = 0.0f};
        xQueueSend(g_queue_display, &msg_rpm, 0);
        xQueueSend(g_queue_display, &msg_vel, 0);
        taskENTER_CRITICAL();
        for (int i = 0; i < NEOPIXEL_COUNT; i++) npSetLED(i, 255, 255, 255);
        npWrite();
        g_farol_ligado = true;
        gpio_put(PIN_LED_FAROL, g_farol_ligado);
        taskEXIT_CRITICAL();
        for(int i = 0; i < 10; i++) {
            buzzer_set_siren_tone(900); vTaskDelay(pdMS_TO_TICKS(250));
            buzzer_set_siren_tone(600); vTaskDelay(pdMS_TO_TICKS(250));
        }
        buzzer_set_siren_tone(0);
        npClear();
        npWrite();
        if (xSemaphoreTake(g_mutex_system_state, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_system_state = STATE_NORMAL;
            xSemaphoreGive(g_mutex_system_state);
        }

        uint64_t end_time = time_us_64();
        uint32_t ct = end_time - start_ct;
        uint32_t resp_time = end_time - start_resp_time;
        if (ct > g_max_ct_airbag_us) g_max_ct_airbag_us = ct;
        if (resp_time > g_max_resp_time_airbag_us) g_max_resp_time_airbag_us = resp_time;
    }
}

void task_logica_abs(void *params) {
    while (true) {
        xSemaphoreTake(g_sem_abs, portMAX_DELAY);
        uint64_t start_resp_time = g_trigger_time_abs_us;
        uint64_t start_ct = time_us_64();
        
        bool should_run = false;
        if (xSemaphoreTake(g_mutex_system_state, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (g_system_state == STATE_NORMAL) {
                g_system_state = STATE_ABS_ACTIVE;
                should_run = true;
            }
            xSemaphoreGive(g_mutex_system_state);
        }
        if (should_run) {
            for(int i = 0; i < 10; i++) {
                buzzer_set_siren_tone(1500);
                for(int j=0; j<NEOPIXEL_COUNT; j++) npSetLED(j, 200, 100, 0);
                npWrite();
                vTaskDelay(pdMS_TO_TICKS(100));
                SystemState current_state;
                if (xSemaphoreTake(g_mutex_system_state, pdMS_TO_TICKS(10)) == pdTRUE) {
                    current_state = g_system_state;
                    xSemaphoreGive(g_mutex_system_state);
                }
                if (current_state != STATE_ABS_ACTIVE) break;
                buzzer_set_siren_tone(0);
                npClear(); npWrite();
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            if (xSemaphoreTake(g_mutex_system_state, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (g_system_state == STATE_ABS_ACTIVE) {
                    g_system_state = STATE_NORMAL;
                }
                xSemaphoreGive(g_mutex_system_state);
            }
        }
        uint64_t end_time = time_us_64();
        uint32_t ct = end_time - start_ct;
        uint32_t resp_time = end_time - start_resp_time;
        if (ct > g_max_ct_abs_us) g_max_ct_abs_us = ct;
        if (resp_time > g_max_resp_time_abs_us) g_max_resp_time_abs_us = resp_time;
    }
}

void task_comando_piloto(void *params){
    while (true) {
        xSemaphoreTake(g_sem_piloto, portMAX_DELAY);
        uint64_t start_resp_time = g_trigger_time_piloto_us;
        uint64_t start_ct = time_us_64();
        
        g_farol_ligado = !g_farol_ligado;
        gpio_put(PIN_LED_FAROL, g_farol_ligado);
        
        uint64_t end_time = time_us_64();
        uint32_t ct = end_time - start_ct;
        uint32_t resp_time = end_time - start_resp_time;
        if (ct > g_max_ct_piloto_us) g_max_ct_piloto_us = ct;
        if (resp_time > g_max_resp_time_piloto_us) g_max_resp_time_piloto_us = resp_time;
    }
}

void task_debounce_buttons(void *params){
    typedef enum { STATE_RELEASED, STATE_PRESSING, STATE_PRESSED } ButtonState;
    ButtonState state = STATE_RELEASED;
    int press_counter = 0;
    const int DEBOUNCE_CYCLES = 3;
    const TickType_t xFrequency = pdMS_TO_TICKS(20);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(true){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint64_t start_resp_time = time_us_64();
        
        bool is_pressed = (gpio_get(PIN_CMD_PILOTO) == 0);
        switch (state){
            case STATE_RELEASED: if (is_pressed) { state = STATE_PRESSING; press_counter = 1; } break;
            case STATE_PRESSING:
                if (is_pressed) {
                    press_counter++;
                    if (press_counter >= DEBOUNCE_CYCLES) {
                        g_trigger_time_piloto_us = time_us_64();
                        xSemaphoreGive(g_sem_piloto);
                        state = STATE_PRESSED;
                    }
                } else { state = STATE_RELEASED; }
                break;
            case STATE_PRESSED: if (!is_pressed) { state = STATE_RELEASED; } break;
        }

        uint64_t end_time = time_us_64();
        uint32_t ct = end_time - start_resp_time;
        if (ct > g_max_ct_debounce_us) g_max_ct_debounce_us = ct;
        if (ct > g_max_resp_time_debounce_us) g_max_resp_time_debounce_us = ct;
    }
}

void task_monitor_rpm(void *params){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    DisplayMessage msg;
    msg.source = UPDATE_RPM;
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint64_t start_time = time_us_64();

        adc_select_input(MIC_CHANNEL);
        sample_mic();
        float raw_rms = mic_power();
        msg.value = raw_rms;
        xQueueSend(g_queue_display, &msg, 0);

        uint64_t end_time = time_us_64();
        uint32_t ct = end_time - start_time;
        if (ct > g_max_ct_rpm_us) g_max_ct_rpm_us = ct;
        if (ct > g_max_resp_time_rpm_us) g_max_resp_time_rpm_us = ct;
    }
}

void task_monitor_velocidade(void *params){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(25);
    DisplayMessage msg;
    msg.source = UPDATE_VELOCIDADE;
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint64_t start_time = time_us_64();

        adc_select_input(ADC_VELOCIDADE);
        uint16_t vel_raw = adc_read();
        msg.value = (float)vel_raw * 250.0f / 4095.0f;
        xQueueSend(g_queue_display, &msg, 0);
        
        uint64_t end_time = time_us_64();
        uint32_t ct = end_time - start_time;
        if (ct > g_max_ct_vel_us) g_max_ct_vel_us = ct;
        if (ct > g_max_resp_time_vel_us) g_max_resp_time_vel_us = ct;
    }
}

void task_monitor_temperatura(void *params){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500);
    DisplayMessage msg;
    msg.source = UPDATE_TEMP;
    const float CONVERSION_FACTOR = 3.3f / (1 << 12);
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint64_t start_time = time_us_64();

        adc_select_input(ADC_TEMP);
        uint16_t temp_raw = adc_read();
        float voltage = temp_raw * CONVERSION_FACTOR;
        float temp_c = 27.0f - (voltage - 0.706f) / 0.001721f;
        msg.value = temp_c;
        xQueueSend(g_queue_display, &msg, 0);
        
        uint64_t end_time = time_us_64();
        uint32_t ct = end_time - start_time;
        if (ct > g_max_ct_temp_us) g_max_ct_temp_us = ct;
        if (ct > g_max_resp_time_temp_us) g_max_resp_time_temp_us = ct;
    }
}

void task_monitor_combustivel(void *params) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint64_t start_time = time_us_64();
        
        if (xSemaphoreTake(g_mutex_system_state, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (g_system_state == STATE_NORMAL) {
                adc_select_input(ADC_COMBUSTIVEL);
                uint16_t joy_raw = adc_read();
                float fuel_percentage = ((float)joy_raw / 4095.0f) * 100.0f;
                npClear();
                int leds_acesos = (int)(fuel_percentage / 4.0f);
                for (int i = 0; i < leds_acesos; i++) {
                    if (i < 5) npSetLED(i, 0, 0, 50); else if (i < 15) npSetLED(i, 0, 50, 0); else npSetLED(i, 50, 0, 0);
                }
                npWrite();
            }
            xSemaphoreGive(g_mutex_system_state);
        }
        
        uint64_t end_time = time_us_64();
        uint32_t ct = end_time - start_time;
        if (ct > g_max_ct_fuel_us) g_max_ct_fuel_us = ct;
        if (ct > g_max_resp_time_fuel_us) g_max_resp_time_fuel_us = ct;
    }
}

void task_gerencia_display(void *params){
    DisplayMessage msg;
    char str_rpm[17] = "RPM: ----";
    char str_vel[17] = "Vel: --- km/h";
    char str_temp[17] = "Temp: --.- C";
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint64_t start_time = time_us_64();

        while (xQueueReceive(g_queue_display, &msg, 0) == pdPASS) {
            switch(msg.source) {
                case UPDATE_RPM: sprintf(str_rpm, "RPM: %.0f", msg.value); break;
                case UPDATE_VELOCIDADE: sprintf(str_vel, "Vel: %.0f km/h", msg.value); break;
                case UPDATE_TEMP: sprintf(str_temp, "Temp: %.1f C", msg.value); break;
            }
        }
        memset(g_display_buffer, 0, ssd1306_buffer_length);
        ssd1306_draw_string(g_display_buffer, 0, 0, "STR Veicular");
        ssd1306_draw_string(g_display_buffer, 0, 16, str_rpm);
        ssd1306_draw_string(g_display_buffer, 0, 32, str_vel);
        ssd1306_draw_string(g_display_buffer, 0, 48, str_temp);
        render_on_display(g_display_buffer, &g_frame_area);
        
        uint64_t end_time = time_us_64();
        uint32_t ct = end_time - start_time;
        if (ct > g_max_ct_display_us) g_max_ct_display_us = ct;
        if (ct > g_max_resp_time_display_us) g_max_resp_time_display_us = ct;
    }
}

int64_t report_timer_callback(alarm_id_t id, void *user_data) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(g_sem_report_trigger, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return 0;
}

void gpio_callback_isr(uint gpio, uint32_t events){ 
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; 
    if (gpio == PIN_SENSOR_COLISAO) {
        g_trigger_time_airbag_us = time_us_64();
        xSemaphoreGiveFromISR(g_sem_airbag, &xHigherPriorityTaskWoken);
    } else if (gpio == PIN_SENSOR_ABS) {
        g_trigger_time_abs_us = time_us_64();
        xSemaphoreGiveFromISR(g_sem_abs, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

int main() {
    stdio_init_all();
    sleep_ms(2000); 
    init_hardware();
    printf("\n--== STR Veicular com FreeRTOS (v3.1.2-Full - Profiling) ==--\n");
    printf("Coletando dados de performance por %d segundos...\n\n", ANALYSIS_RUNTIME_S);
    g_sem_airbag = xSemaphoreCreateBinary();
    g_sem_abs = xSemaphoreCreateBinary();
    g_sem_piloto = xSemaphoreCreateBinary();
    g_sem_report_trigger = xSemaphoreCreateBinary();
    g_mutex_system_state = xSemaphoreCreateMutex();
    g_queue_display = xQueueCreate(10, sizeof(DisplayMessage));
    xTaskCreate(task_analysis_report, "ReportTask", 1024, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(task_logica_airbag, "AirbagTask", 256, NULL, 12, NULL);
    xTaskCreate(task_logica_abs, "ABSTask", 256, NULL, 11, NULL);
    xTaskCreate(task_monitor_rpm, "RPMTask", 256, NULL, 10, NULL);
    xTaskCreate(task_gerencia_display, "DisplayTask", 512, NULL, 9, NULL);
    xTaskCreate(task_monitor_velocidade, "VelTask", 256, NULL, 8, NULL);
    xTaskCreate(task_debounce_buttons, "DebounceTask", 256, NULL, 6, NULL);
    xTaskCreate(task_monitor_temperatura, "TempTask", 256, NULL, 5, NULL);
    xTaskCreate(task_comando_piloto, "PilotoTask", 256, NULL, 4, NULL);
    xTaskCreate(task_monitor_combustivel, "FuelTask", 512, NULL, 2, NULL);
    gpio_set_irq_enabled_with_callback(PIN_SENSOR_COLISAO, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_isr);
    gpio_set_irq_enabled_with_callback(PIN_SENSOR_ABS, GPIO_IRQ_EDGE_FALL, true, &gpio_callback_isr);
    add_alarm_in_ms(ANALYSIS_RUNTIME_S * 1000, report_timer_callback, NULL, false);
    vTaskStartScheduler();
    while(true);
}

void init_hardware() {
    adc_init();
    adc_gpio_init(26 + ADC_VELOCIDADE);
    adc_gpio_init(26 + ADC_COMBUSTIVEL);
    adc_set_temp_sensor_enabled(true);
    adc_gpio_init(MIC_PIN);
    adc_fifo_setup(true, true, 1, false, false);
    adc_set_clkdiv(ADC_CLOCK_DIV);
    g_dma_channel = dma_claim_unused_channel(true);
    g_dma_cfg = dma_channel_get_default_config(g_dma_channel);
    channel_config_set_transfer_data_size(&g_dma_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&g_dma_cfg, false);
    channel_config_set_write_increment(&g_dma_cfg, true);
    channel_config_set_dreq(&g_dma_cfg, DREQ_ADC);
    gpio_init(PIN_LED_FAROL); gpio_set_dir(PIN_LED_FAROL, GPIO_OUT);
    gpio_set_function(PIN_BUZZER, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PIN_BUZZER);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 250.0f);
    pwm_init(slice_num, &config, false);
    gpio_init(PIN_SENSOR_COLISAO); gpio_set_dir(PIN_SENSOR_COLISAO, GPIO_IN); gpio_pull_up(PIN_SENSOR_COLISAO);
    gpio_init(PIN_SENSOR_ABS); gpio_set_dir(PIN_SENSOR_ABS, GPIO_IN); gpio_pull_up(PIN_SENSOR_ABS);
    gpio_init(PIN_CMD_PILOTO); gpio_set_dir(PIN_CMD_PILOTO, GPIO_IN); gpio_pull_up(PIN_CMD_PILOTO);
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    ssd1306_init(I2C_PORT, OLED_ADDR, 128, 64);
    g_frame_area = (struct render_area){ .start_column = 0, .end_column = ssd1306_width - 1, .start_page = 0, .end_page = ssd1306_n_pages - 1 };
    calculate_render_area_buffer_length(&g_frame_area);
    npInit(PIN_NEOPIXEL, NEOPIXEL_COUNT);
    npClear();
    npWrite();
}