
# Tarefa: Roteiro de FreeRTOS #1 - EmbarcaTech 2025

Autor: **Gabriel da Conceição Miranda**

Curso: Residência Tecnológica em Sistemas Embarcados

Instituição: EmbarcaTech - HBr

Brasília, 10 de junho de 2025

---

Sistema Multitarefa com FreeRTOS no RP2040
Este projeto demonstra a criação de um sistema embarcado multitarefa utilizando o sistema operacional de tempo real (RTOS) FreeRTOS na placa Raspberry Pi Pico (RP2040). O objetivo é controlar três periféricos de forma concorrente, com interação do usuário para gerenciar a execução das tarefas.

💡 Ideia da Tarefa (Objetivo)
O desafio central é desenvolver um sistema onde múltiplas ações ocorrem "simultaneamente" sem que uma interfira no tempo de execução da outra. Isso é alcançado através da divisão das funcionalidades em tarefas independentes, gerenciadas pelo escalonador (scheduler) do FreeRTOS. O sistema deve ser responsivo às interações do usuário (pressionamento de botões) para controlar as outras tarefas em tempo real.

Funcionalidades Principais
LED RGB Cíclico: Um LED RGB alterna ciclicamente entre as cores vermelho, verde e azul.
Buzzer Periódico: Um buzzer emite bipes sonoros em intervalos regulares.
Controle por Botões: Dois botões permitem ao usuário controlar as tarefas:
Botão A: Suspende ou retoma a tarefa do LED.
Botão B: Suspende ou retoma a tarefa do buzzer.
🛠️ Hardware e Software
Hardware:
Placa BitDogLab ou qualquer outra baseada no Raspberry Pi Pico (RP2040).
Cabo USB para gravação e alimentação.
Software:
FreeRTOS Kernel
Raspberry Pi Pico C/C++ SDK
CMake
ARM GCC Toolchain
Visual Studio Code
🚀 Como Compilar e Gravar
Este projeto foi desenvolvido utilizando o template rp2040-freertos-template, que automatiza o download das dependências.

Clone o projeto:
Bash

git clone https://github.com/EmbarcaTech-2025/tarefa-freertos-1-MirandaBiel.git
Abra no VS Code: Abra a pasta do projeto no Visual Studio Code.
Configure o CMake: Na primeira vez, a extensão CMake Tools irá configurar o projeto, baixando automaticamente o Pico SDK e o FreeRTOS. Selecione o kit GCC for arm-none-eabi.
Compile: Pressione F7 ou clique no botão Build na barra de status do VS Code.
Grave na Placa:
Coloque a placa em modo BOOTSEL (pressione o botão BOOTSEL e conecte o cabo USB).
A placa aparecerá como um drive USB chamado RPI-RP2.
Arraste o arquivo .uf2 gerado (localizado na pasta build) para dentro deste drive.
📐 Arquitetura do Código Solução
O sistema é dividido em três tarefas principais, além da função main que configura o ambiente.

C

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

// Handles das tarefas para controle externo
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
        vTaskDelay(pdMS_TO_TICKS(100)); // Bip curto de 100ms
        gpio_put(BUZZER_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(900)); // Resto do período de 1s
    }
}

// Tarefa para leitura dos botões (com lógica de toggle e debounce)
void vButtonTask(void *params) {
    bool led_suspended = false;
    bool buzzer_suspended = false;

    while (true) {
        // Lógica para o Botão A (LED)
        if (!gpio_get(BUTTON_A)) {
            // Inverte o estado da tarefa
            if (!led_suspended) {
                vTaskSuspend(xLedTaskHandle);
                led_suspended = true;
            } else {
                vTaskResume(xLedTaskHandle);
                led_suspended = false;
            }
            // Espera o botão ser solto para garantir uma ação por clique
            while (!gpio_get(BUTTON_A)) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }

        // Lógica para o Botão B (Buzzer)
        if (!gpio_get(BUTTON_B)) {
            if (!buzzer_suspended) {
                vTaskSuspend(xBuzzerTaskHandle);
                buzzer_suspended = true;
            } else {
                vTaskResume(xBuzzerTaskHandle);
                buzzer_suspended = false;
            }
            while (!gpio_get(BUTTON_B)) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }

        // Intervalo de verificação dos botões
        vTaskDelay(pdMS_TO_TICKS(50)); 
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
    xTaskCreate(vButtonTask, "Button Task", 256, NULL, 2, NULL); // Prioridade maior para responsividade

    // Inicia o agendador
    vTaskStartScheduler();

    while (true); // Nunca deve chegar aqui
}

Explicação do Código
main(): A função principal é responsável por:

Inicializar o hardware (GPIOs para os LEDs, buzzer e botões).
Criar as três tarefas usando xTaskCreate(). Note que os handles xLedTaskHandle e xBuzzerTaskHandle são passados como parâmetro para que a tarefa dos botões possa controlá-las.
Iniciar o escalonador com vTaskStartScheduler(), que passa o controle do processador para o FreeRTOS.
vLedTask e vBuzzerTask: São tarefas simples e periódicas. Elas executam sua lógica em um loop infinito, usando vTaskDelay() para controlar o tempo e, crucialmente, para devolver o controle da CPU ao escalonador, permitindo que outras tarefas rodem. vButtonTask: É a tarefa mais complexa. Ela possui prioridade mais alta (2 contra 1 das outras) para garantir que a leitura dos botões seja processada rapidamente.
Ela implementa uma lógica de toggle: o primeiro clique suspende a tarefa alvo, e o segundo a retoma. Variáveis booleanas (led_suspended, buzzer_suspended) guardam o estado atual.
Para robustez, após detectar um clique e realizar a ação, ela entra em um pequeno loop while (!gpio_get(...)) para esperar que o usuário solte o botão. Isso evita que um pressionamento longo acione a lógica múltiplas vezes.
Conceitos de FreeRTOS Aplicados
Multitarefa Preemptiva: O escalonador alterna entre as tarefas, dando a ilusão de paralelismo.
Prioridades de Tarefa: A tarefa dos botões é mais prioritária para garantir a melhor experiência de usuário.
Handles de Tarefa (TaskHandle_t): Usados como "identificadores" para que uma tarefa (a dos botões) possa manipular outra (suspender/retomar as do LED e buzzer).
Controle de Tarefas: As funções vTaskSuspend() e vTaskResume() são usadas para pausar e continuar a execução das tarefas dinamicamente.
Bloqueio e Atrasos: vTaskDelay() é usado para criar atrasos sem desperdiçar ciclos de CPU (diferente de um loop de espera, ou busy-waiting).

---

## 📜 Licença
GNU GPL-3.0.
