# ====================================================================================
# Seção Padrão do VSCode para Pico
# (Não editar esta parte)
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

# Nome do projeto
set(ProjectName str-freertos)

# Localização do FreeRTOS
if (DEFINED ENV{FREERTOS_PATH})
    set(FREERTOS_PATH $ENV{FREERTOS_PATH})
else()
    set(FREERTOS_PATH ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS)
endif()

message("FreeRTOS Kernel localizado em ${FREERTOS_PATH}")

# Importar o SDK do Pico e o Kernel do FreeRTOS
include(pico_sdk_import.cmake)
include(${FREERTOS_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Definir o projeto e os padrões C/C++
project(${ProjectName} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Inicializar o SDK do Pico
pico_sdk_init()

# 1. CRIAR O EXECUTÁVEL com todos os seus arquivos fontes
add_executable(${ProjectName}
    src/main.c
    include/ssd1306_i2c.c   # Driver do display OLED
    include/neopixel.c      # Driver da matriz de LEDs
)

# 2. MODIFICAR O EXECUTÁVEL com as configurações de periféricos
pico_enable_stdio_usb(${ProjectName} 1)
pico_enable_stdio_uart(${ProjectName} 0)
pico_generate_pio_header(${ProjectName} ${CMAKE_CURRENT_LIST_DIR}/include/ws2818b.pio)

# 3. Adicionar os diretórios de include
target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR} # Para o ws2818b.pio.h gerado
    # Diretórios de include do Tracealyzer
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/include
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/config
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/streamports/Jlink_RTT/include # Adicionado para garantir que headers específicos do port sejam encontrados
)

# 4. Adicionar os arquivos fonte do Tracealyzer
target_sources(${ProjectName} PRIVATE
    # Arquivos C do Trace Recorder (da raiz da pasta TraceRecorder/)
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcAssert.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcCounter.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcDependency.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcDiagnostics.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcEntryTable.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcError.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcEvent.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcEventBuffer.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcExtension.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcHardwarePort.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcHeap.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcISR.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcInternalEventBuffer.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcInterval.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcKernelPort.c # Kernel Port para FreeRTOS
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcMultiCoreEventBuffer.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcObject.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcPrint.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcRunnable.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcSnapshotRecorder.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcStackMonitor.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcStateMachine.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcStaticBuffer.c
    # ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcStreamPort.c <-- LINHA REMOVIDA (CORREÇÃO)
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcStreamingRecorder.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcString.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcTask.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/trcTimestamp.c

    # Arquivos C do Stream Port J-Link RTT
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/streamports/Jlink_RTT/SEGGER_RTT.c
    ${CMAKE_CURRENT_LIST_DIR}/TraceRecorder/streamports/Jlink_RTT/trcStreamPort.c # O Stream Port específico para RTT
)

# 5. Linkar TODAS as bibliotecas de hardware e do FreeRTOS necessárias
target_link_libraries(${ProjectName}
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_pio
    hardware_clocks
    hardware_adc
    hardware_dma
    FreeRTOS-Kernel-Heap4
    cmsis_core
)

# 6. Gerar os arquivos de saída (.uf2, .bin, etc.)
pico_add_extra_outputs(${ProjectName})