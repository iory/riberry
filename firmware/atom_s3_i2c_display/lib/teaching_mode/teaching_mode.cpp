#include <teaching_mode.h>

TeachingMode::TeachingMode() : Mode("TeachingMode") {}

void TeachingMode::task(PrimitiveLCD &lcd, CommunicationBase &com) {
    while (running) {
        if (handleTimeout(lcd, com) || handleEmptyDisplay(lcd)) continue;
        int listSize = 2;
        // Draw string
        char **StrList = (char **)malloc(listSize * sizeof(char *));
        int modeCount = com.splitString(lcd.color_str, ',', StrList, listSize);
        lcd.drawBlack();
        lcd.printColorText(String(StrList[1]));
        // Draw AR Marker if found
        if (!String(StrList[0]).equals(String(""))) {
            int marker_id = atoi(StrList[0]);
            int width = lcd.width();
            int height = lcd.height();
            int size = 64;
            drawARMarker(marker_id, width - size - 5, height - size - 5, size, lcd);
        }
        delayWithTimeTracking(pdMS_TO_TICKS(1000));
    }
}

// (x, y) are the upper left coordinates of the marker.
void TeachingMode::drawARMarker(int marker_id, int x, int y, int size, PrimitiveLCD &lcd) {
    if (size < 8) {
        Serial.println("Error: Marker size must be at least 8.");
        return;
    }
    lcd.fillRect(x, y, size, size, TFT_BLACK);
    lcd.fillRect(x + size / 8, y + size / 8, size * 6 / 8, size * 6 / 8, TFT_WHITE);
    lcd.fillRect(x + size * 2 / 8, y + size * 2 / 8, size * 4 / 8, size * 4 / 8, TFT_BLACK);
    int pattern[4][4] = {{1, 0, 0, 1}, {0, 1, 1, 0}, {1, 1, 0, 0}, {0, 0, 1, 1}};
    int cellSize = size / 8;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            if (pattern[row][col] == 1) {
                lcd.fillRect(x + (col + 2) * cellSize, y + (row + 2) * cellSize, cellSize, cellSize,
                             TFT_WHITE);
            }
        }
    }

    if (marker_id >= 0) {
        float textSizeOrig = lcd.getTextSize();
        float text_size = size / 32.0;
        lcd.fillCircle(x + size / 2, y + size / 2, text_size * 5.0, TFT_BLACK);
        lcd.setTextSize(text_size);
        String color_id = String("\x1b[32m") + String(marker_id) + String("\x1b[39m");
        lcd.setCursor(x + size / 2 - lcd.textWidth(String(marker_id)) / 2,
                      y + size / 2 - lcd.textWidth(String("a")) / 2);
        lcd.printColorText(color_id);
        // Restore text settings
        lcd.setTextSize(textSizeOrig);
        lcd.setCursor(0, 0);
    }
}
