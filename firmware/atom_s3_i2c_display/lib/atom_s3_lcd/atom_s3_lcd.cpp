#include <atom_s3_lcd.h>

AtomS3LCD::AtomS3LCD()
  : qrCodeData(""), PrimitiveLCD() {
  setRotation(lcd_rotation);
  clear();
  setTextSize(1.5);
}

// e.g. atoms3lcd.drawImage(atoms3lcd.jpegBuf, atoms3lcd.jpegLength);
void AtomS3LCD::drawImage(uint8_t* jpegBuf, uint32_t jpegLength) {
  drawJpg(jpegBuf, jpegLength, 0, 0, 128, 128, 0, 0, ::JPEG_DIV_NONE);
}

void AtomS3LCD::drawQRcode(const String& qrCodeData) {
  if (qrCodeData.length() > 0) {
    // Draw QR code if qrCodeData is received
    fillScreen(color565(255, 255, 255));
    qrcode(qrCodeData.c_str(), SPACING, SPACING / 2, width() - SPACING * 2, QR_VERSION);
  }
  else {
    fillScreen(color565(255, 0, 0));  // Fill the screen with red
    setCursor(0, 0);
    println("No QR code data received.");
  }
}

void AtomS3LCD::printWaitMessage(int i2cAddress) {
  printColorText("Wait for I2C input.\n");
#ifdef USE_GROVE
  printColorText("\x1b[31mGROVE\x1b[39m Mode\n");
#else
  printColorText("\x1b[31mNOT GROVE\x1b[39m Mode\n");
#endif
  char log_msg[50];
  sprintf(log_msg, "I2C address \x1b[33m0x%02x\x1b[39m", i2cAddress);
  printColorText(log_msg);
}

void AtomS3LCD::drawNoDataReceived() {
  fillScreen(color565(255, 0, 0));  // Fill the screen with red
  setCursor(0, 0);
  printColorText("No data received.\n");
}

void AtomS3LCD::drawBlack() {
  fillScreen(color565(0, 0, 0));  // Fill the screen with black
  setCursor(0, 0);
}

void AtomS3LCD::resetColorStr() {
  color_str = "";
}

void AtomS3LCD::resetJpegBuf() {
  memset(jpegBuf, 0, sizeof(jpegBuf));
}

void AtomS3LCD::resetQRcodeData() {
  qrCodeData = "";
}

void AtomS3LCD::resetLcdData() {
  resetColorStr();
  resetJpegBuf();
  resetQRcodeData();
  mode_changed = true;
}

unsigned long AtomS3LCD::getLastDrawTime() {
  return lastDrawTime;
}

void AtomS3LCD::setLastDrawTime(unsigned long time) {
  lastDrawTime = time;
}
