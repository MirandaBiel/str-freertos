# Projeto STR-FreeRTOS para RP2040 (BitDogLab)

Este projeto demonstra a utilização do FreeRTOS em uma placa baseada no **RP2040 (BitDogLab)** para controlar múltiplos periféricos de forma concorrente.  
O sistema gerencia um **LED RGB**, uma **matriz de LEDs NeoPixel**, um **buzzer**, um **display OLED** e a leitura de **botões**, com cada periférico sendo controlado por sua própria **tarefa dedicada**.

---

## 🧠 Arquitetura do Software

O projeto utiliza uma arquitetura **multitarefa**, dividindo o trabalho em "mini-programas" independentes chamados de **tarefas** (*tasks*).  
O **FreeRTOS** atua como um **agendador (scheduler)**, garantindo que cada tarefa receba tempo de processador para executar suas funções, criando a ilusão de que tudo acontece simultaneamente.

---

## 🔁 1. O Conceito Central: Tarefas (Tasks)

Uma **tarefa** é uma função C que roda "em paralelo" com outras. No nosso código, temos 4 tarefas principais, cada uma com uma responsabilidade única:

- `vLedTask`: Controla o LED RGB e a matriz NeoPixel.
- `vBuzzerTask`: Controla o buzzer.
- `vDisplayTask`: Controla o display OLED.
- `vButtonTask`: Lê os botões e se comunica com as outras tarefas.

Essa separação organiza o código de forma **modular e limpa**. Por exemplo, se o buzzer parar de funcionar, o problema estará isolado apenas na `vBuzzerTask`.

---

## 🧱 2. Criando as Tarefas: `xTaskCreate()`

As tarefas são instanciadas e configuradas dentro da função `main()` utilizando `xTaskCreate()`:

```c
xTaskCreate(
    vLedTask,           // 1. Ponteiro para a função da tarefa
    "LED Task",         // 2. Nome da tarefa (para depuração)
    512,                // 3. Tamanho da pilha (Stack Size em Words)
    NULL,               // 4. Parâmetro (não utilizado aqui)
    1,                  // 5. Prioridade da tarefa
    &xLedTaskHandle     // 6. Handle para controle da tarefa
);
```

### 🛠️ Parâmetros Importantes

- **Stack Size**: Define a quantidade de memória (em palavras de 4 bytes) disponível para variáveis locais. Tarefas mais complexas (como `vLedTask`) requerem mais memória.
- **Prioridade**: O FreeRTOS sempre executa a tarefa pronta de maior prioridade.  
  Exemplo de prioridades:
  - `vDisplayTask`: prioridade 3 (interface responsiva)
  - `vButtonTask`: prioridade 2
  - `vLedTask` e `vBuzzerTask`: prioridade 1
- **Handle**: Um "controle remoto" da tarefa, utilizado para pausar (`vTaskSuspend`) ou retomar (`vTaskResume`) sua execução.

---

## ⏱️ 3. Pausando o Tempo: `vTaskDelay()`

Dentro de cada tarefa, usamos `vTaskDelay()`:

- **Não é um `sleep()` bloqueante.**
- Permite ao FreeRTOS suspender a tarefa temporariamente e alocar o processador para outras tarefas.
- Essencial para manter o sistema multitarefa funcional e eficiente.

Toda tarefa com `while(1)` precisa de uma chamada de bloqueio (`vTaskDelay`, `xQueueReceive`, etc.) para não monopolizar a CPU.

---

## 📬 4. Comunicação Segura entre Tarefas: Filas (Queues)

Para comunicar tarefas de forma segura e eficiente, utilizamos **filas (queues)**, evitando o uso de variáveis globais e o risco de **race conditions**.

### 🔄 Fluxo de Comunicação com Fila:

- **Criação:** Na `main()`, criamos uma fila com `xQueueCreate()`, especificando seu tamanho e tipo de dados.
- **Envio:** `vButtonTask` envia uma mensagem com `xQueueSend()` quando um botão é pressionado.
- **Recebimento:** `vDisplayTask` permanece bloqueada em `xQueueReceive()` aguardando mensagens, acordando apenas quando necessário.

Esse padrão promove **baixo acoplamento**, **eficiência** e **segurança** entre tarefas.

---

## 🚀 5. O Ponto de Partida: `vTaskStartScheduler()`

No final da `main()`, chamamos `vTaskStartScheduler()`. Esta função:

- **Inicia o agendador do FreeRTOS**, que então passa a controlar completamente o processador.
- Qualquer código após essa chamada **não será executado**.

---

## 📦 Requisitos

- Placa **RP2040 (BitDogLab)**
- Ambiente de desenvolvimento com suporte a **C/C++ para RP2040**
- **FreeRTOS** portado para RP2040
- Bibliotecas para:
  - Controle de **NeoPixels**
  - Manipulação de **display OLED (I2C/SPI)**
  - Leitura de **botões**
  - **Buzzer PWM**
