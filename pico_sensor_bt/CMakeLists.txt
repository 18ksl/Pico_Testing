cmake_minimum_required(VERSION 3.13)

# Import the Pico SDK
include(pico_sdk_import.cmake)

# Set board type to Pico W before project declaration
set(PICO_BOARD pico_w)

# Project name
project(pico_sensor_bt C CXX ASM)

# Set C standard
set(CMAKE_C_STANDARD 11)

# Initialize the Pico SDK
pico_sdk_init()

# Create the executable from main.c
add_executable(pico_sensor_bt
    main.c
)

# Enable stdio over USB and disable UART
pico_enable_stdio_usb(pico_sensor_bt 1)
pico_enable_stdio_uart(pico_sensor_bt 0)

# Link libraries for networking, Bluetooth, and CYW43
target_link_libraries(pico_sensor_bt
    pico_stdlib
    pico_cyw43_arch_lwip_threadsafe_background
    pico_btstack_classic
    pico_btstack_cyw43
    hardware_adc
    hardware_i2c
    pico_btstack_ble
)

target_include_directories(pico_sensor_bt PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}  # This ensures lwipopts.h is found
    /home/mithril/pico/pico-examples/pico_w/bt/standalone/
    )


# Add extra outputs for UF2 and bin files
pico_add_extra_outputs(pico_sensor_bt)
