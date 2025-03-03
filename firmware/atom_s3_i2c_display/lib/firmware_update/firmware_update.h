#ifndef FIRMWARE_UPDATE_H
#define FIRMWARE_UPDATE_H

#include <Arduino.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_system.h>

#include <LovyanGFX.hpp>

#include "communication_base.h"
#include "mode.h"
#include "mode_type.h"

#define UART_NUM UART_NUM_1
#define BUF_SIZE (8192)
uint8_t dma_buffer[BUF_SIZE];

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

void update_firmware(PrimitiveLCD& lcd, CommunicationBase& com) {
    uint32_t total = 0;
    int32_t totalLength = 0;
    setup_uart_dma();

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

    Stream* _stream = com.getStream();
    lcd.drawBlack();
    lcd.printColorText("serial begin success\n");
    int byteLength = 0;
    unsigned long lastDataTime = millis();

    esp_ota_handle_t ota_handle;
    esp_err_t err;
    size_t available;
    while (total < 4) {
        int len =
                uart_read_bytes(UART_NUM, dma_buffer + total, BUF_SIZE - total, pdMS_TO_TICKS(10));
        if (len > 0) total += len;
        if (millis() - lastDataTime > 5000) return;
    }
    totalLength = (uint32_t)dma_buffer[0] | ((uint32_t)dma_buffer[1] << 8) |
                  ((uint32_t)dma_buffer[2] << 16) | ((uint32_t)dma_buffer[3] << 24);
    total = 0;
    lcd.drawBlack();
    lcd.printColorText("Total length: ");
    lcd.printColorText(String(totalLength).c_str());
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
        int len = uart_read_bytes(UART_NUM, dma_buffer, BUF_SIZE, pdMS_TO_TICKS(10));
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
                sprite.drawString(String(progress) + "%", 100, 30);
                sprite.pushSprite(0, lcd.height() / 2 - 10);
                lastProgress = progress;
            }
        }
        if (millis() - lastDataTime > 5000) break;
    }
    if (total != totalLength) {
        lcd.printColorText("Download failed\n");
        delay(2000);
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
