#include "pico/stdlib.h"

int main() {
    const uint LED_PIN = 25; // On-board LED
    gpio_init(LED_BUILTIN);
    gpio_set_dir(LED_BUILTIN, true);

    while (true) {
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
    }
}

