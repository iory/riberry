#ifndef FIRMWARE_UPDATE_H
#define FIRMWARE_UPDATE_H

#include <Arduino.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_system.h>

#include <LovyanGFX.hpp>

#include "mode.h"
#include "mode_type.h"

#define UART_NUM UART_NUM_1
#define I2C_NUM I2C_NUM_0

constexpr int I2C_SLAVE_FREQ_HZ = 1000000;  // 1 MHz (Fast-mode Plus)
constexpr int BUF_SIZE = 8192;
uint8_t dma_buffer[BUF_SIZE];

#ifdef ATOM_S3
constexpr bool use_i2c = true;
    #ifdef I2C_ADDR
static constexpr int i2c_slave_addr = I2C_ADDR;
    #else
static constexpr int i2c_slave_addr = 0x42;
    #endif  // end of I2C_ADDR

    #ifdef USE_GROVE
static constexpr int sda_pin = 2;
static constexpr int scl_pin = 1;
    #else
static constexpr int sda_pin = 38;
static constexpr int scl_pin = 39;
    #endif  // end of USE_GROVE

bool setup_i2c_dma(PrimitiveLCD& lcd) {
    esp_err_t res = i2c_driver_delete(i2c_port_t(I2C_NUM));
    if (res != ESP_OK && res != ESP_ERR_INVALID_ARG) {
        lcd.printColorText("I2C driver delete failed: ");
        lcd.printColorText(esp_err_to_name(res));
        lcd.printColorText("\n");
        delay(3000);
        return false;
    }

    gpio_reset_pin(gpio_num_t(sda_pin));
    gpio_reset_pin(gpio_num_t(scl_pin));

    i2c_config_t config = {
            .mode = I2C_MODE_SLAVE,
            .sda_io_num = sda_pin,
            .scl_io_num = scl_pin,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .slave =
                    {
                            .addr_10bit_en = 0,
                            .slave_addr = i2c_slave_addr,
                            .maximum_speed = I2C_SLAVE_FREQ_HZ,
                    },
            .clk_flags = 0,
    };

    i2c_param_config(I2C_NUM, &config);
    res = i2c_param_config(I2C_NUM, &config);
    if (res != ESP_OK) {
        lcd.drawBlack();
        lcd.printColorText("I2C param config failed:");
        lcd.printColorText(esp_err_to_name(res));
        delay(3000);
        return false;
    }
    res = i2c_driver_install(I2C_NUM, config.mode, BUF_SIZE * 2, BUF_SIZE * 2, 0);
    if (res != ESP_OK) {
        lcd.drawBlack();
        lcd.printColorText("I2C driver install failed: ");
        lcd.printColorText(esp_err_to_name(res));
        delay(3000);
        return false;
    }
    return true;
}

int i2c_read_with_dma(uint8_t* buffer,
                      size_t len,
                      uint8_t slave_addr = i2c_slave_addr,
                      unsigned long timeout = 500) {
    int total_received = 0;
    int received = 0;
    unsigned long start_time = millis();
    do {
        received = i2c_slave_read_buffer(I2C_NUM, buffer + total_received,
                                         min(len - total_received, (size_t)BUF_SIZE),
                                         pdMS_TO_TICKS(100));
        if (received > 0) {
            total_received += received;
            Serial.printf("I2C received: %d bytes, total: %d\n", received, total_received);
        }
        if (received == 0 && millis() - start_time < 200) {
            delay(10);
        }
    } while (received > 0 && total_received < len && (millis() - start_time) < timeout);
    return total_received;
}
#else
constexpr bool use_i2c = false;
bool setup_i2c_dma() { return false; }
int i2c_read_with_dma(uint8_t* buffer, size_t len, uint8_t slave_addr = 0) { return -1; }
#endif

void setup_uart_dma() {
    uart_config_t uart_config = {
            .baud_rate = 921600,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_set_rx_timeout(UART_NUM, 1);
}

void update_firmware(PrimitiveLCD& lcd) {
    uint32_t total = 0;
    int32_t totalLength = 0;
    if (use_i2c) {
        if (!setup_i2c_dma(lcd)) {
            return;
        }
    } else {
        setup_uart_dma();
    }

    lcd.drawBlack();
    lcd.printColorText("Firmware Update Mode\n");
    lcd.printColorText("Starting update...\n");
    delay(2000);
    const esp_partition_t* ota_partition = esp_ota_get_next_update_partition(NULL);
    if (ota_partition == NULL) {
        lcd.printColorText("OTA partition not found\n");
        delay(2000);
        return;
    } else {
        lcd.printColorText("OTA Partition: ");
        lcd.printColorText(ota_partition->label);
        lcd.printColorText("\n");
        lcd.printColorText("Type: 0x");
        lcd.printColorText(String(ota_partition->type, HEX).c_str());
        lcd.printColorText(", Subtype: 0x");
        lcd.printColorText(String(ota_partition->subtype, HEX).c_str());
        lcd.printColorText("\n");
    }
    delay(1000);
    lcd.drawBlack();
    lcd.printColorText("serial begin success\n");
    int byteLength = 0;
    unsigned long lastDataTime = millis();

    esp_ota_handle_t ota_handle;
    esp_err_t err;
    size_t available;
    int len;
    while (total < 4) {
        if (use_i2c) {
            len = i2c_read_with_dma(dma_buffer + total, 4 - total);
        } else {
            len = uart_read_bytes(UART_NUM, dma_buffer + total, BUF_SIZE - total,
                                  pdMS_TO_TICKS(10));
        }
        if (len > 0) total += len;
        if (millis() - lastDataTime > 5000) return;
    }
    totalLength = (uint32_t)dma_buffer[0] | ((uint32_t)dma_buffer[1] << 8) |
                  ((uint32_t)dma_buffer[2] << 16) | ((uint32_t)dma_buffer[3] << 24);
    total = 0;
    lcd.drawBlack();
    lcd.printColorText("Total length: ");
    lcd.printColorText(String(totalLength).c_str());
    lcd.printColorText(" bytes\n");
    delay(1000);
    err = esp_ota_begin(ota_partition, totalLength, &ota_handle);
    if (err != ESP_OK) {
        lcd.printColorText("OTA begin failed\n");
        lcd.printColorText(esp_err_to_name(err));
        delay(5000);
        return;
    }
    lcd.printColorText("OTA begin success\n");
    lastDataTime = millis();

    LGFX_Sprite sprite(&lcd);
    sprite.setColorDepth(16);
    sprite.createSprite(lcd.width(), 40);
    sprite.setPaletteColor(0, TFT_BLACK);
    sprite.setPaletteColor(1, TFT_GREEN);
    sprite.setTextSize(2);

    uint8_t lastProgress = 255;

    while (total < totalLength) {
        if (use_i2c) {
            len = i2c_read_with_dma(dma_buffer, BUF_SIZE);
        } else {
            len = uart_read_bytes(UART_NUM, dma_buffer, BUF_SIZE, pdMS_TO_TICKS(10));
        }
        if (len > 0) {
            total += len;
            lastDataTime = millis();
            esp_ota_write(ota_handle, dma_buffer, len);

            uint8_t progress = (uint8_t)((total * 100) / totalLength);

            if ((progress != lastProgress) || total >= totalLength) {
                sprite.fillSprite(0);
                int barWidth = (progress * lcd.width()) / 100;
                if (barWidth < 10) barWidth = 10;
                sprite.fillRect(0, 0, barWidth, 20, TFT_GREEN);
                sprite.setTextColor(TFT_WHITE, TFT_BLACK);
                sprite.setTextDatum(MC_DATUM);
                sprite.drawString(String(progress) + "%", lcd.width() / 2, 30);
                sprite.pushSprite(0, lcd.height() / 2 + 10);
                lastProgress = progress;
            }
        }
        if (millis() - lastDataTime > 5000) break;
    }
    if (total != totalLength) {
        lcd.drawBlack();
        lcd.printColorText("Download failed\n");
        char buf[64];
        snprintf(buf, sizeof(buf), "%d/%d\n", total, totalLength);
        lcd.printColorText(buf);
        if (total > totalLength) {
            lcd.printColorText("Too much data received\n");
        } else {
            lcd.printColorText("Few data received\n");
        }
        delay(4000);
        return;
    }
    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        lcd.printColorText("OTA end failed\n");
        lcd.printColorText(esp_err_to_name(err));
        delay(2000);
        return;
    }
    err = esp_ota_set_boot_partition(ota_partition);
    if (err != ESP_OK) {
        lcd.printColorText("Set boot partition failed\n");
        lcd.printColorText(esp_err_to_name(err));
        delay(2000);
        return;
    }
    lcd.printColorText("OTA success, restarting...\n");
    delay(3000);
    esp_restart();
}

#endif  // FIRMWARE_UPDATE_H
