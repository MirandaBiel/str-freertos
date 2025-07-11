/*
 * PROJETO: Simulador de Sistema de Controle Veicular em Tempo Real
 * VERSÃO: 5.1.0 (Relatório de Dados Brutos)
 *
 * DESCRIÇÃO: 
 * Esta versão mantém toda a funcionalidade de análise simultânea de CT e WCRT,
 * mas modifica a tarefa de relatório final para imprimir os dados brutos em
 * duas linhas separadas (uma para CT, uma para WCRT), facilitando a exportação
 * para outras ferramentas de análise.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

// --- SDK Pico ---
#include "pico/stdlib.h"
#include "hardware/timer.h" 
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "pico/binary_info.h"

// --- FreeRTOS ---
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

// --- Drivers de Periféricos ---
#include "ssd1306.h"
#include "ssd1306_i2c.h"

// --- Protótipos de drivers ---
void npInit(uint pin, uint amount);
void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b);
void npClear();
void npWrite();

// =============================================================================
// --- CONFIGURAÇÃO DA ANÁLISE ---
// =============================================================================
typedef enum {
    TASK_ID_RPM, TASK_ID_VELOCIDADE, TASK_ID_TEMPERATURA, TASK_ID_COMBUSTIVEL,
    TASK_ID_AIRBAG, TASK_ID_ABS, TASK_ID_PILOTO, TASK_ID_DEBOUNCE, TASK_ID_DISPLAY
} TaskID;

// ESCOLHA QUAL TAREFA ANALISAR AQUI
#define TASK_TO_ANALYZE TASK_ID_PILOTO
#define MAX_SAMPLES 100
#define ANALYSIS_TRIGGER_PERIOD_MS 5000

// =============================================================================
// --- DEFINIÇÕES E MAPEAMENTO DE HARDWARE ---
// =============================================================================
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

uint8_t g_display_buffer[ssd1306_buffer_length];
struct render_area g_frame_area;
uint g_dma_channel;
dma_channel_config g_dma_cfg;
uint16_t g_mic_buffer[MIC_SAMPLES];

// =============================================================================
// --- ESTRUTURAS E VARIÁVEIS GLOBAIS DO RTOS ---
// =============================================================================
typedef enum { STATE_NORMAL, STATE_ABS_ACTIVE, STATE_AIRBAG_ACTIVE } SystemState;
volatile SystemState g_system_state = STATE_NORMAL;

SemaphoreHandle_t g_sem_airbag=NULL, g_sem_abs=NULL, g_sem_piloto=NULL;
QueueHandle_t g_queue_display=NULL;
SemaphoreHandle_t g_mutex_system_state=NULL, g_sem_report_trigger=NULL;
typedef enum { UPDATE_RPM, UPDATE_VELOCIDADE, UPDATE_TEMP } DisplaySource;
typedef struct { DisplaySource source; float value; } DisplayMessage;
volatile bool g_farol_ligado = false;

// =============================================================================
// --- VARIÁVEIS PARA ANÁLISE DE TEMPO ---
// =============================================================================
uint32_t g_computation_time_samples[MAX_SAMPLES]; // Vetor para amostras de CT
uint32_t g_response_time_samples[MAX_SAMPLES];  // Vetor para amostras de WCRT
volatile uint32_t g_sample_count = 0;
volatile uint64_t g_trigger_time_airbag_us=0, g_trigger_time_abs_us=0, g_trigger_time_piloto_us=0;

// =============================================================================
// --- PROTÓTIPOS E FUNÇÕES AUXILIARES ---
// =============================================================================
void init_hardware();
void gpio_callback_isr(uint gpio, uint32_t events);
void task_analysis_report(void *params);
bool analysis_trigger_callback(struct repeating_timer *t);
void sample_mic() { adc_fifo_drain(); adc_run(false); dma_channel_configure(g_dma_channel, &g_dma_cfg, g_mic_buffer, &adc_hw->fifo, MIC_SAMPLES, true); adc_run(true); dma_channel_wait_for_finish_blocking(g_dma_channel); adc_run(false); }
float mic_power() { float avg = 0.f; for (uint i = 0; i < MIC_SAMPLES; ++i) { avg += g_mic_buffer[i] * g_mic_buffer[i]; } avg /= MIC_SAMPLES; return sqrt(avg); }
void buzzer_set_siren_tone(uint freq) { uint slice_num = pwm_gpio_to_slice_num(PIN_BUZZER); if (freq == 0) { pwm_set_enabled(slice_num, false); } else { uint32_t wrap = 500000 / freq - 1; pwm_set_wrap(slice_num, wrap); pwm_set_chan_level(slice_num, pwm_gpio_to_channel(PIN_BUZZER), wrap / 2); pwm_set_enabled(slice_num, true); } }


// =============================================================================
// --- TAREFA DE ANÁLISE E RELATÓRIO ---
// =============================================================================
/*
 * Esta tarefa é ativada ao final da coleta de dados.
 * A sua única função é imprimir os dados coletados no terminal serial
 * no formato solicitado e depois travar o sistema.
 */
void task_analysis_report(void *params) {
    // Aguarda o gatilho que sinaliza o fim da coleta.
    xSemaphoreTake(g_sem_report_trigger, portMAX_DELAY);

    // --- INÍCIO DA SEÇÃO MODIFICADA ---
    printf("\n\n--- INICIO DO RELATORIO DE DADOS ---\n");
    
    // Imprime todos os valores de CT (Tempo de Execução) em uma linha, separados por espaço.
    printf("\nCT_DATA: ");
    for (int i = 0; i < MAX_SAMPLES; i++) {
        printf("%lu ", g_computation_time_samples[i]);
    }
    printf("\n");

    // Imprime todos os valores de WCRT (Tempo de Resposta) em uma linha, separados por espaço.
    printf("\nWCRT_DATA: ");
    for (int i = 0; i < MAX_SAMPLES; i++) {
        printf("%lu ", g_response_time_samples[i]);
    }
    printf("\n");

    printf("\n--- FIM DO RELATORIO DE DADOS ---\n");
    // --- FIM DA SEÇÃO MODIFICADA ---

    printf("\n--- Analise concluida. O sistema sera travado. ---\n");
    taskDISABLE_INTERRUPTS();
    while(1);
}

// =============================================================================
// --- TAREFAS DO SISTEMA (INSTRUMENTADAS PARA MEDIÇÃO) ---
// =============================================================================
void record_sample(uint32_t wcrt, uint32_t ct) {
    taskENTER_CRITICAL();
    if (g_sample_count < MAX_SAMPLES) {
        g_response_time_samples[g_sample_count] = wcrt;
        g_computation_time_samples[g_sample_count] = ct;
        g_sample_count++;
        if (g_sample_count == MAX_SAMPLES) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(g_sem_report_trigger, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    taskEXIT_CRITICAL();
}

void task_logica_airbag(void *params) {
    while (true) {
        xSemaphoreTake(g_sem_airbag, portMAX_DELAY);
        uint64_t trigger_time = g_trigger_time_airbag_us;
        uint64_t start_ct = time_us_64();
        
        if(xSemaphoreTake(g_mutex_system_state,pdMS_TO_TICKS(10))==pdTRUE){g_system_state=STATE_AIRBAG_ACTIVE;xSemaphoreGive(g_mutex_system_state);}
        printf("EVENTO CRITICO: COLISAO DETECTADA!\n");
        DisplayMessage msg_rpm={.source=UPDATE_RPM,.value=0.0f}, msg_vel={.source=UPDATE_VELOCIDADE,.value=0.0f};
        xQueueSend(g_queue_display,&msg_rpm,0);xQueueSend(g_queue_display,&msg_vel,0);
        taskENTER_CRITICAL(); for(int i=0;i<NEOPIXEL_COUNT;i++)npSetLED(i,255,255,255); npWrite();g_farol_ligado=true;gpio_put(PIN_LED_FAROL,g_farol_ligado); taskEXIT_CRITICAL();
        for(int i=0;i<10;i++){buzzer_set_siren_tone(900);vTaskDelay(pdMS_TO_TICKS(250));buzzer_set_siren_tone(600);vTaskDelay(pdMS_TO_TICKS(250));}
        buzzer_set_siren_tone(0);npClear();npWrite();
        if(xSemaphoreTake(g_mutex_system_state,pdMS_TO_TICKS(10))==pdTRUE){g_system_state=STATE_NORMAL;xSemaphoreGive(g_mutex_system_state);}
        
        uint64_t end_time = time_us_64();
        if (TASK_TO_ANALYZE == TASK_ID_AIRBAG) {
            record_sample(end_time - trigger_time, end_time - start_ct);
        }
    }
}

void task_logica_abs(void *params) {
    while (true) {
        xSemaphoreTake(g_sem_abs, portMAX_DELAY);
        uint64_t trigger_time = g_trigger_time_abs_us;
        uint64_t start_ct = time_us_64();
        bool should_run=false;if(xSemaphoreTake(g_mutex_system_state,pdMS_TO_TICKS(10))==pdTRUE){if(g_system_state==STATE_NORMAL){g_system_state=STATE_ABS_ACTIVE;should_run=true;}xSemaphoreGive(g_mutex_system_state);}
        if(should_run){for(int i=0;i<10;i++){buzzer_set_siren_tone(1500);for(int j=0;j<NEOPIXEL_COUNT;j++)npSetLED(j,200,100,0);npWrite();vTaskDelay(pdMS_TO_TICKS(100));SystemState cs;if(xSemaphoreTake(g_mutex_system_state,pdMS_TO_TICKS(10))==pdTRUE){cs=g_system_state;xSemaphoreGive(g_mutex_system_state);}if(cs!=STATE_ABS_ACTIVE)break;buzzer_set_siren_tone(0);npClear();npWrite();vTaskDelay(pdMS_TO_TICKS(100));}if(xSemaphoreTake(g_mutex_system_state,pdMS_TO_TICKS(10))==pdTRUE){if(g_system_state==STATE_ABS_ACTIVE)g_system_state=STATE_NORMAL;xSemaphoreGive(g_mutex_system_state);}}
        uint64_t end_time = time_us_64();
        if (TASK_TO_ANALYZE == TASK_ID_ABS) { record_sample(end_time - trigger_time, end_time - start_ct); }
    }
}

void task_comando_piloto(void *params){
    while (true) {
        xSemaphoreTake(g_sem_piloto, portMAX_DELAY);
        uint64_t trigger_time = g_trigger_time_piloto_us;
        uint64_t start_ct = time_us_64();
        g_farol_ligado = !g_farol_ligado; gpio_put(PIN_LED_FAROL, g_farol_ligado);
        uint64_t end_time = time_us_64();
        if (TASK_TO_ANALYZE == TASK_ID_PILOTO) { record_sample(end_time - trigger_time, end_time - start_ct); }
    }
}

void task_debounce_buttons(void *params){
    typedef enum{R,P,H}S;S s=R;int c=0;const int D=3;const TickType_t f=pdMS_TO_TICKS(20);TickType_t l=xTaskGetTickCount();
    while(true){
        vTaskDelayUntil(&l,f);
        uint64_t start_wcrt = time_us_64();
        uint64_t start_ct = start_wcrt;
        bool pr=(gpio_get(PIN_CMD_PILOTO)==0);
        switch(s){case R:if(pr){s=P;c=1;}break;case P:if(pr){c++;if(c>=D){g_trigger_time_piloto_us=time_us_64();xSemaphoreGive(g_sem_piloto);s=H;}}else{s=R;}break;case H:if(!pr){s=R;}break;}
        uint64_t end_time = time_us_64();
        if (TASK_TO_ANALYZE == TASK_ID_DEBOUNCE) { record_sample(end_time - start_wcrt, end_time - start_ct); }
    }
}

void task_monitor_rpm(void *params){
    TickType_t l=xTaskGetTickCount();const TickType_t f=pdMS_TO_TICKS(100);DisplayMessage m;m.source=UPDATE_RPM;
    while(true){
        vTaskDelayUntil(&l,f);
        uint64_t start_wcrt = time_us_64();
        uint64_t start_ct = start_wcrt;
        adc_select_input(MIC_CHANNEL);sample_mic();m.value=mic_power();xQueueSend(g_queue_display,&m,0);
        uint64_t end_time = time_us_64();
        if (TASK_TO_ANALYZE == TASK_ID_RPM) { record_sample(end_time - start_wcrt, end_time - start_ct); }
    }
}

void task_monitor_velocidade(void *params){
    TickType_t l=xTaskGetTickCount();const TickType_t f=pdMS_TO_TICKS(25);DisplayMessage m;m.source=UPDATE_VELOCIDADE;
    while(true){
        vTaskDelayUntil(&l,f);
        uint64_t start_wcrt = time_us_64();
        uint64_t start_ct = start_wcrt;
        adc_select_input(ADC_VELOCIDADE);m.value=(float)adc_read()*250.0f/4095.0f;xQueueSend(g_queue_display,&m,0);
        uint64_t end_time = time_us_64();
        if (TASK_TO_ANALYZE == TASK_ID_VELOCIDADE) { record_sample(end_time - start_wcrt, end_time - start_ct); }
    }
}

void task_monitor_temperatura(void *params){
    TickType_t l=xTaskGetTickCount();const TickType_t f=pdMS_TO_TICKS(500);DisplayMessage m;m.source=UPDATE_TEMP;const float C=3.3f/(1<<12);
    while(true){
        vTaskDelayUntil(&l,f);
        uint64_t start_wcrt = time_us_64();
        uint64_t start_ct = start_wcrt;
        adc_select_input(ADC_TEMP);float v=adc_read()*C;m.value=27.0f-(v-0.706f)/0.001721f;xQueueSend(g_queue_display,&m,0);
        uint64_t end_time = time_us_64();
        if (TASK_TO_ANALYZE == TASK_ID_TEMPERATURA) { record_sample(end_time - start_wcrt, end_time - start_ct); }
    }
}

void task_monitor_combustivel(void *params) {
    TickType_t l=xTaskGetTickCount();const TickType_t f=pdMS_TO_TICKS(100);
    while(true){
        vTaskDelayUntil(&l,f);
        uint64_t start_wcrt = time_us_64();
        uint64_t start_ct = start_wcrt;
        if(xSemaphoreTake(g_mutex_system_state,pdMS_TO_TICKS(10))==pdTRUE){if(g_system_state==STATE_NORMAL){adc_select_input(ADC_COMBUSTIVEL);float p=((float)adc_read()/4095.0f)*100.0f;npClear();int leds=(int)(p/4.0f);for(int i=0;i<leds;i++){if(i<5)npSetLED(i,0,0,50);else if(i<15)npSetLED(i,0,50,0);else npSetLED(i,50,0,0);}npWrite();}xSemaphoreGive(g_mutex_system_state);}
        uint64_t end_time = time_us_64();
        if (TASK_TO_ANALYZE == TASK_ID_COMBUSTIVEL) { record_sample(end_time - start_wcrt, end_time - start_ct); }
    }
}

void task_gerencia_display(void *params){
    DisplayMessage m;char sr[17]="RPM: ----",sv[17]="Vel: --- km/h",st[17]="Temp: --.- C";const TickType_t f=pdMS_TO_TICKS(100);TickType_t l=xTaskGetTickCount();
    while(true){
        vTaskDelayUntil(&l,f);
        uint64_t start_wcrt = time_us_64();
        uint64_t start_ct = start_wcrt;
        while(xQueueReceive(g_queue_display,&m,0)==pdPASS){switch(m.source){case UPDATE_RPM:sprintf(sr,"RPM: %.0f",m.value);break;case UPDATE_VELOCIDADE:sprintf(sv,"Vel: %.0f km/h",m.value);break;case UPDATE_TEMP:sprintf(st,"Temp: %.1f C",m.value);break;}}
        memset(g_display_buffer,0,ssd1306_buffer_length);ssd1306_draw_string(g_display_buffer,0,0,"STR Veicular");ssd1306_draw_string(g_display_buffer,0,16,sr);ssd1306_draw_string(g_display_buffer,0,32,sv);ssd1306_draw_string(g_display_buffer,0,48,st);render_on_display(g_display_buffer,&g_frame_area);
        uint64_t end_time = time_us_64();
        if (TASK_TO_ANALYZE == TASK_ID_DISPLAY) { record_sample(end_time - start_wcrt, end_time - start_ct); }
    }
}

// =============================================================================
// --- ROTINA DE INTERRUPÇÃO E MAIN ---
// =============================================================================

bool analysis_trigger_callback(struct repeating_timer *t) {
    BaseType_t xHigherPriorityTaskWoken=pdFALSE;TaskID id_to_trigger=(TaskID)(t->user_data);
    if (g_sample_count >= MAX_SAMPLES) { return false; }
    switch(id_to_trigger){case TASK_ID_AIRBAG:g_trigger_time_airbag_us=time_us_64();xSemaphoreGiveFromISR(g_sem_airbag,&xHigherPriorityTaskWoken);break;case TASK_ID_ABS:g_trigger_time_abs_us=time_us_64();xSemaphoreGiveFromISR(g_sem_abs,&xHigherPriorityTaskWoken);break;case TASK_ID_PILOTO:g_trigger_time_piloto_us=time_us_64();xSemaphoreGiveFromISR(g_sem_piloto,&xHigherPriorityTaskWoken);break;default:break;}
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);return true;
}

void gpio_callback_isr(uint gpio, uint32_t events){ 
    BaseType_t woken=pdFALSE; 
    if(gpio==PIN_SENSOR_COLISAO){g_trigger_time_airbag_us=time_us_64();xSemaphoreGiveFromISR(g_sem_airbag,&woken);}
    else if(gpio==PIN_SENSOR_ABS){g_trigger_time_abs_us=time_us_64();xSemaphoreGiveFromISR(g_sem_abs,&woken);}
    portYIELD_FROM_ISR(woken);
}

int main() {
    stdio_init_all();
    sleep_ms(2000); 
    init_hardware();
    
    printf("\n--== STR Veicular com FreeRTOS (v5.1.0 - Relatorio de Dados Brutos) ==--\n");
    printf("Analisando Tarefa ID: %d. Coletando %d amostras de CT e WCRT...\n", TASK_TO_ANALYZE, MAX_SAMPLES);

    g_sem_airbag=xSemaphoreCreateBinary();g_sem_abs=xSemaphoreCreateBinary();g_sem_piloto=xSemaphoreCreateBinary();
    g_sem_report_trigger=xSemaphoreCreateBinary();g_mutex_system_state=xSemaphoreCreateMutex();
    g_queue_display=xQueueCreate(10,sizeof(DisplayMessage));
    
    xTaskCreate(task_analysis_report,"ReportTask",2048,NULL,configMAX_PRIORITIES-1,NULL);
    xTaskCreate(task_logica_airbag,"AirbagTask",256,NULL,12,NULL);
    xTaskCreate(task_logica_abs,"ABSTask",256,NULL,11,NULL);
    xTaskCreate(task_monitor_rpm,"RPMTask",256,NULL,10,NULL);
    xTaskCreate(task_gerencia_display,"DisplayTask",512,NULL,9,NULL);
    xTaskCreate(task_monitor_velocidade,"VelTask",256,NULL,8,NULL);
    xTaskCreate(task_debounce_buttons,"DebounceTask",256,NULL,6,NULL);
    xTaskCreate(task_monitor_temperatura,"TempTask",256,NULL,5,NULL);
    xTaskCreate(task_comando_piloto,"PilotoTask",256,NULL,4,NULL);
    xTaskCreate(task_monitor_combustivel,"FuelTask",512,NULL,2,NULL);
    
    gpio_set_irq_enabled_with_callback(PIN_SENSOR_COLISAO,GPIO_IRQ_EDGE_FALL,true,&gpio_callback_isr);
    gpio_set_irq_enabled_with_callback(PIN_SENSOR_ABS,GPIO_IRQ_EDGE_FALL,true,&gpio_callback_isr);
    
    static struct repeating_timer timer;
    switch(TASK_TO_ANALYZE){case TASK_ID_AIRBAG:case TASK_ID_ABS:case TASK_ID_PILOTO:printf("Modo de analise de tarefa de botao ativado. Disparando a cada %d ms.\n", ANALYSIS_TRIGGER_PERIOD_MS);add_repeating_timer_ms(ANALYSIS_TRIGGER_PERIOD_MS,analysis_trigger_callback,(void*)TASK_TO_ANALYZE,&timer);break;default:break;}
    
    vTaskStartScheduler();
    while(true);
}

void init_hardware() {
    adc_init();
    adc_gpio_init(26+ADC_VELOCIDADE);adc_gpio_init(26+ADC_COMBUSTIVEL);
    adc_set_temp_sensor_enabled(true);adc_gpio_init(MIC_PIN);
    adc_fifo_setup(true,true,1,false,false);adc_set_clkdiv(ADC_CLOCK_DIV);
    g_dma_channel=dma_claim_unused_channel(true);g_dma_cfg=dma_channel_get_default_config(g_dma_channel);
    channel_config_set_transfer_data_size(&g_dma_cfg,DMA_SIZE_16);channel_config_set_read_increment(&g_dma_cfg,false);
    channel_config_set_write_increment(&g_dma_cfg,true);channel_config_set_dreq(&g_dma_cfg,DREQ_ADC);
    gpio_init(PIN_LED_FAROL);gpio_set_dir(PIN_LED_FAROL,GPIO_OUT);
    gpio_set_function(PIN_BUZZER,GPIO_FUNC_PWM);
    uint slice_num=pwm_gpio_to_slice_num(PIN_BUZZER);pwm_config config=pwm_get_default_config();
    pwm_config_set_clkdiv(&config,250.0f);pwm_init(slice_num,&config,false);
    gpio_init(PIN_SENSOR_COLISAO);gpio_set_dir(PIN_SENSOR_COLISAO,GPIO_IN);gpio_pull_up(PIN_SENSOR_COLISAO);
    gpio_init(PIN_SENSOR_ABS);gpio_set_dir(PIN_SENSOR_ABS,GPIO_IN);gpio_pull_up(PIN_SENSOR_ABS);
    gpio_init(PIN_CMD_PILOTO);gpio_set_dir(PIN_CMD_PILOTO,GPIO_IN);gpio_pull_up(PIN_CMD_PILOTO);
    i2c_init(I2C_PORT,400*1000);gpio_set_function(I2C_SDA,GPIO_FUNC_I2C);gpio_set_function(I2C_SCL,GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);gpio_pull_up(I2C_SCL);
    ssd1306_init(I2C_PORT,OLED_ADDR,128,64);
    g_frame_area=(struct render_area){.start_column=0,.end_column=ssd1306_width-1,.start_page=0,.end_page=ssd1306_n_pages-1};
    calculate_render_area_buffer_length(&g_frame_area);
    npInit(PIN_NEOPIXEL,NEOPIXEL_COUNT);npClear();npWrite();
}