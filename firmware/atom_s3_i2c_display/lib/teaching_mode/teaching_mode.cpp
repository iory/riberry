#include <teaching_mode.h>

TeachingMode::TeachingMode()
  : Mode("TeachingMode") {
}

void TeachingMode::task(PrimitiveLCD &lcd, CommunicationBase &com) {
  while (true) {
    com.setRequestStr(getModeName());
    // Check for I2C timeout
    if (com.checkTimeout()) {
      lcd.drawNoDataReceived();
      lcd.printColorText(getModeName() + "\n");
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    // Display information
    else {
      lcd.drawBlack();
      if (lcd.color_str.isEmpty())
        lcd.printColorText("Waiting for " + getModeName());
      else {
        int listSize = 2;
        // Draw string
        char** StrList = (char**)malloc(listSize * sizeof(char*));
        int modeCount = com.splitString(lcd.color_str, ',', StrList, listSize);
        lcd.printColorText(String(StrList[1]));
        // Draw AR Marker if found
        if (!String(StrList[0]).equals(String(""))) {
          int marker_id = atoi(StrList[0]);
          int width = lcd.width();
          int height = lcd.height();
          int size = 64;
          drawARMarker(marker_id, width - size - 5, height - size - 5, size, lcd);
        }
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
}

// 引数: marker_id: マーカーのID, x: 左上のX座標, y: 左上のY座標, size: マーカーのサイズ
void TeachingMode::drawARMarker(int marker_id, int x, int y, int size, PrimitiveLCD &lcd) {
  if (size < 8) {
    Serial.println("Error: Marker size must be at least 8.");
    return;
  }

  // 黒背景を描画
  lcd.fillRect(x, y, size, size, TFT_BLACK);
  // 白い外枠を描画
  lcd.fillRect(x + size / 8, y + size / 8, size * 6 / 8, size * 6 / 8, TFT_WHITE);
  // 内部の黒枠を描画
  lcd.fillRect(x + size * 2 / 8, y + size * 2 / 8, size * 4 / 8, size * 4 / 8, TFT_BLACK);
  // ARマーカーの模様を描画
  // 模様を簡易的に固定パターンとして定義
  int pattern[4][4] = {
                       {1, 0, 0, 1},
                       {0, 1, 1, 0},
                       {1, 1, 0, 0},
                       {0, 0, 1, 1}
  };

  int cellSize = size / 8;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      if (pattern[row][col] == 1) {
        lcd.fillRect(x + (col + 2) * cellSize, y + (row + 2) * cellSize, cellSize, cellSize, TFT_WHITE);
      }
    }
  }

  if (marker_id >= 0) {
    float textSizeOrig = lcd.getTextSize();
    float text_size = size / 32.0;
    // 黒い円で内部を抜く
    lcd.fillCircle(x + size / 2, y + size / 2, text_size * 5.0, TFT_BLACK);
    // マーカーIDを描画
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