# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
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

cmake_minimum_required(VERSION 3.12)

# Nome do projeto
set(ProjectName str-freertos)

# Caminho do FreeRTOS
if (DEFINED ENV{FREERTOS_PATH})
  set(FREERTOS_PATH $ENV{FREERTOS_PATH})
else()
  set(FREERTOS_PATH ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS)
endif()

message("FreeRTOS Kernel located in ${FREERTOS_PATH}")

# Importa pico-sdk e o kernel do FreeRTOS
include(pico_sdk_import.cmake)
include(${FREERTOS_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Define o projeto e configura padrões
project(${ProjectName} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Inicializa SDK do Pico
pico_sdk_init()

# 1. DEFINE O EXECUTÁVEL E SEUS ARQUIVOS FONTE
add_executable(${ProjectName}
    src/main.c
    include/ssd1306_i2c.c
    include/neopixel.c
)

# 2. GERA O HEADER DO PIO E O ADICIONA AO EXECUTÁVEL (AGORA JÁ EXISTENTE)
# Assumindo que ws2818b.pio está na pasta 'include'
pico_generate_pio_header(${ProjectName} ${CMAKE_CURRENT_LIST_DIR}/include/ws2818b.pio)

# Diretórios de include
target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_BINARY_DIR} # Necessário para encontrar o pio.h gerado
)

# Bibliotecas utilizadas
target_link_libraries(${ProjectName} 
    pico_stdlib 
    hardware_i2c
    hardware_pio
    hardware_clocks
    FreeRTOS-Kernel-Heap4
)

# Gera UF2, bin, etc.
pico_add_extra_outputs(${ProjectName})
