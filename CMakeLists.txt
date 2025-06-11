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
# Projeto: Simulador de Sistema Veicular em Tempo Real
# Arquivo de configuração do Build com CMake

# --- Seção Padrão do VSCode para Pico ---
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
# =================================================

set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

# Nome do projeto (CORRIGIDO PARA O NOME ORIGINAL)
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
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_BINARY_DIR} # Para o ws2818b.pio.h gerado
)

# 4. Linkar TODAS as bibliotecas de hardware e do FreeRTOS necessárias
target_link_libraries(${ProjectName} 
    pico_stdlib 
    hardware_spi
    hardware_i2c        # Para o display OLED
    hardware_pio        # Para a matriz NeoPixel
    hardware_clocks     # Dependência do PIO
    hardware_adc        # Para os sensores analógicos (Joystick, Temp)
    FreeRTOS-Kernel-Heap4
)

# 5. Gerar os arquivos de saída (.uf2, .bin, etc.)
pico_add_extra_outputs(${ProjectName})
