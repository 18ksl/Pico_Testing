#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "btstack.h"

#define PIN_ADC 26
#define THRESHOLD_VOLTAGE 1.0f
#define ADC_CONVERSION_FACTOR (3.3f / (1 << 12))

// Bluetooth variables
static uint16_t rfcomm_channel_id = 0;
static uint8_t rfcomm_channel_nr = 1;
static bool bluetooth_connected = false;
static btstack_packet_callback_registration_t hci_event_callback_registration;

// Sensor variables
static bool last_sensor_state = false;

// HCI packet handler
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);
    
    bd_addr_t event_addr;
    
    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {
                case BTSTACK_EVENT_STATE:
                    if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                        bd_addr_t local_addr;
                        gap_local_bd_addr(local_addr);
                        printf("BTstack up and running on %s.\n", bd_addr_to_str(local_addr));
                        
                        // Make device discoverable and connectable
                        gap_discoverable_control(1);
                        gap_connectable_control(1);
                        gap_set_class_of_device(0x200404);
                        gap_set_local_name("PicoW-Sensor");
                    }
                    break;
                    
                case HCI_EVENT_PIN_CODE_REQUEST:
                    printf("Pin code request - using '0000'\n");
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
                    break;
                    
                default:
                    break;
            }
            break;
            
        default:
            break;
    }
}

// RFCOMM packet handler
static void rfcomm_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);
    
    bd_addr_t event_addr;
    
    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {
                case RFCOMM_EVENT_INCOMING_CONNECTION:
                    rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
                    rfcomm_channel_id = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
                    printf("RFCOMM channel %u requested for %s\n", rfcomm_channel_id, bd_addr_to_str(event_addr));
                    rfcomm_accept_connection(rfcomm_channel_id);
                    break;
                    
                case RFCOMM_EVENT_CHANNEL_OPENED:
                    if (rfcomm_event_channel_opened_get_status(packet)) {
                        printf("RFCOMM channel open failed, status 0x%02x\n", 
                               rfcomm_event_channel_opened_get_status(packet));
                        rfcomm_channel_id = 0;
                    } else {
                        rfcomm_channel_id = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                        printf("RFCOMM channel open succeeded. Channel ID %u\n", rfcomm_channel_id);
                        bluetooth_connected = true;
                    }
                    break;
                    
                case RFCOMM_EVENT_CHANNEL_CLOSED:
                    printf("RFCOMM channel closed\n");
                    rfcomm_channel_id = 0;
                    bluetooth_connected = false;
                    break;
                    
                default:
                    break;
            }
            break;
            
        default:
            break;
    }
}

void send_sensor_data(bool sensor_triggered) {
    if (bluetooth_connected && rfcomm_channel_id) {
        uint8_t data = sensor_triggered ? 1 : 0;
        rfcomm_send(rfcomm_channel_id, &data, 1);
        
        // Visual feedback on LED
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, sensor_triggered);
        
        printf("Sent: %s\n", sensor_triggered ? "TRIGGERED" : "NORMAL");
    }
}

int main() {
    stdio_init_all();
    
    printf("Pico W Bluetooth Sensor Starting...\n");
    
    // Initialize CYW43 (WiFi/Bluetooth chip)
    if (cyw43_arch_init()) {
        printf("Failed to initialize cyw43_arch\n");
        return -1;
    }
    
    // Initialize ADC
    adc_init();
    adc_gpio_init(PIN_ADC);
    adc_select_input(0); // ADC0 (GPIO26)
    
    printf("Initializing Bluetooth...\n");
    
    // Initialize BTstack
    l2cap_init();
    rfcomm_init();
    
    // Register packet handlers
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    
    // Register RFCOMM service
    rfcomm_register_service(rfcomm_packet_handler, rfcomm_channel_nr, 0xffff);
    
    // Turn on Bluetooth
    hci_power_control(HCI_POWER_ON);
    
    printf("Bluetooth initialized. Device name: PicoW-Sensor\n");
    printf("Threshold voltage: %.2fV\n", THRESHOLD_VOLTAGE);
    
    while (true) {
        // Read ADC
        uint16_t adc_raw = adc_read();
        float voltage = adc_raw * ADC_CONVERSION_FACTOR;
        bool sensor_triggered = voltage >= THRESHOLD_VOLTAGE;
        
        // Only send if state changed (reduces latency)
        if (sensor_triggered != last_sensor_state) {
            send_sensor_data(sensor_triggered);
            last_sensor_state = sensor_triggered;
        }
        
        // Process Bluetooth events (non-blocking)
        btstack_run_loop_execute();
        
        // Small delay to prevent overwhelming the CPU
        sleep_ms(5);
    }
    
    // Clean up
    cyw43_arch_deinit();
    return 0;
}
