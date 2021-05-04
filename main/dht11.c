#include <sys/cdefs.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <esp_log.h>
#include <math.h>
#include "driver/rmt.h"

static const char *TAG = "sensor";

RingbufHandle_t ringbuf_handle = NULL;

static const rmt_item32_t wire_sequence[] = {
        {{{10000, 0, 0, 1}}}, // 10ms low
};

static int checkParity(uint_least64_t allBits) {
    int parityCheck = allBits >> 32;
    parityCheck += allBits >> 24 & 255;
    parityCheck += allBits >> 16 & 255;
    parityCheck += allBits >> 8 & 255;
    parityCheck = parityCheck & 255;
    return parityCheck;
}

static void processResponse() {
    rmt_item32_t *rx_items = NULL;
    size_t rx_size = 0;

    rx_items = (rmt_item32_t *) xRingbufferReceive(ringbuf_handle, &rx_size, 200 / portTICK_PERIOD_MS);
    if (!rx_items) {
        // time out occurred, this indicates an unconnected / misconfigured bus
        ESP_LOGE(TAG, "TIMEOUT");
        return;
    }

    if (rx_size >= sizeof(rmt_item32_t)) {
        ESP_LOGI(TAG, "GOT RESPONSE!");

        uint_least64_t allBits = 0;

        for (int i = 0; i < (rx_size / sizeof(rmt_item32_t)); i++) {
            // in this case, duration0 is always the bit, duration1 should be the low signal duration between bits

            if (rx_items[i].level0 != 1) {
                // from experimentation, it will start detecting on the first rising edge
                ESP_LOGE(TAG, "INVALID RESPONSE DETECTED");
                vRingbufferReturnItem(ringbuf_handle, (void *) rx_items);
                return;
            }

            if (rx_items[i].duration0 >= 79 && rx_items[i].duration0 <= 85 && rx_items[i].level0 == 1) {
                // first rising edge should be about 80us
                ESP_LOGI(TAG, "START SIGNAL DETECTED");
                continue;
            } else if (rx_items[i].duration0 >= 68 && rx_items[i].duration0 <= 74 && rx_items[i].level0 == 1) {
                // bit 1, about 70us
                int x = i - 1;
                printf("%d", 1);
                allBits += pow(2, (39 - x));
                continue;
            } else if (rx_items[i].duration0 >= 23 && rx_items[i].duration0 <= 27) {
                // bit 0, about 26~28us
                printf("%d", 0);
                continue;
            } else {
                // for some reason, the first time after flashing, the signal is very weird
                ESP_LOGE(TAG, "INVALID DURATION DETECTED");
                printf("index %d, duration0: %d, level0: %d\n", i, rx_items[i].duration0, rx_items[i].level0);
                printf("index %d, duration1: %d, level1: %d\n", i, rx_items[i].duration1, rx_items[i].level1);
                vRingbufferReturnItem(ringbuf_handle, (void *) rx_items);
                return;
            }
        }


        printf("\n\nAll bits: %llu\n", allBits);

        int humidity = allBits >> 24;
        int temperature = allBits >> 8 & 65535;
        int parity = allBits & 255;

        printf("TEMPERATURE: %.1f\n", temperature / 10.0f);
        printf("HUMIDITY   : %.1f\n", humidity / 10.0f);

        if (checkParity(allBits) != parity) {
            ESP_LOGE(TAG, "PARITY CHECK FAIL");
        }

        vRingbufferReturnItem(ringbuf_handle, (void *) rx_items);
    }
}

static void readSensor() {
    // enable TX
    rmt_config_t rmt_tx_config = RMT_DEFAULT_CONFIG_TX(18, RMT_CHANNEL_0);
    rmt_tx_config.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;
    ESP_ERROR_CHECK(rmt_config(&rmt_tx_config));

    // request data
    ESP_ERROR_CHECK(
            rmt_write_items(RMT_CHANNEL_0, wire_sequence, sizeof(wire_sequence) / sizeof(wire_sequence[0]), true));
    rmt_tx_stop(RMT_CHANNEL_0);
    rmt_rx_start(RMT_CHANNEL_0, true); // !!!!

    // enable RX
    rmt_config_t rmt_rx_config = RMT_DEFAULT_CONFIG_RX(18, RMT_CHANNEL_0);
    ESP_ERROR_CHECK(rmt_config(&rmt_rx_config));

    rmt_get_ringbuf_handle(RMT_CHANNEL_0, &ringbuf_handle);

    processResponse();

    rmt_rx_stop(RMT_CHANNEL_0);
}

_Noreturn void app_main() {
    rmt_driver_install(RMT_CHANNEL_0, 512, 0);

    while (true) {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        readSensor();
    };
}