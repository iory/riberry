#include <M5Unified.h>  // M5.update()のために必要
#include <WiFi.h>

// --- ROS / Rosserial Settings ---
#define ROSSERIAL_ARDUINO_TCP
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
std_msgs::Float32MultiArray status_msg;
ros::Publisher pub_status("motor_status", &status_msg);

// ==========================================
//  Wifi & ROS Configuration (要変更)
// ==========================================
const char* ssid = "";
const char* password = "";
// ROS Master (PC)のIPアドレス
IPAddress server(0, 0, 0, 0);
const uint16_t serverPort = 11411;

// --- Dynamixel Settings ---
#include <Dynamixel2Arduino.h>
#define DXL_SERIAL Serial1
#define SERIAL_CONFIG SERIAL_8N1
const long BAUDRATE = 2000000;
const int TIMEOUT = 100;  // ms
const uint8_t DXL_ID = 0;
const float DXL_PROTOCOL_VERSION = 2.0;

const int EN_PIN = 6;
const int RX_PIN = 5;
const int TX_PIN = 38;

Dynamixel2Arduino dxl(DXL_SERIAL, EN_PIN);

// --- Display Settings ---
// M5UnifiedのDisplayを使用
auto& lcd = M5.Display;
String previousMessage = "";

// --- Global Variables ---
ros::NodeHandle nh;
unsigned long lastCommandTime = 0;
const unsigned long SAFETY_TIMEOUT = 5000;  // 5秒で自動停止

unsigned long lastDisplayTime = 0;
const unsigned long DISPLAY_INTERVAL = 500;  // 500ms (0.5秒) に1回だけ描画

unsigned long lastPublishTime = 0;
const unsigned long PUBLISH_INTERVAL = 200;  // データ送信: 0.2秒 (5Hz)

String currentCmdText = "Wait";  // 現在のコマンド名を記憶

// --- Functions ---

void initLCD() {
    lcd.clear();
    lcd.setTextSize(1.2);
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
}

void printToLCD(const String& message) {
    if (message == previousMessage) {
        return;
    }
    previousMessage = message;
    lcd.fillScreen(lcd.color565(0, 0, 0));
    lcd.setCursor(0, 0);
    lcd.println(message);
}

void setupDXL() {
    DXL_SERIAL.begin(BAUDRATE, SERIAL_CONFIG, RX_PIN, TX_PIN, false, TIMEOUT);
    dxl.begin(BAUDRATE);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    dxl.torqueOff(DXL_ID);
    dxl.setOperatingMode(DXL_ID, OP_CURRENT);
    dxl.torqueOn(DXL_ID);

    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, DXL_ID, 0);
}

float readVoltage() {
    int32_t voltage_raw = dxl.readControlTableItem(ControlTableItem::PRESENT_INPUT_VOLTAGE, DXL_ID);
    return (float)voltage_raw / 10.0;
}

float readTemperature() {
    int32_t temperature = dxl.readControlTableItem(ControlTableItem::PRESENT_TEMPERATURE, DXL_ID);
    return (float)temperature;
}

float readCurrent() { return dxl.getPresentCurrent(DXL_ID, UNIT_MILLI_AMPERE); }

String getStatusHeader() {
    String header = "SSID: " + String(ssid) + "\n";
    header += "IP: " + WiFi.localIP().toString() + "\n";
    return header;
}

String getMotorInfo() {
    return String("\nVol: ") + String(readVoltage(), 1) + "V\n" + String("Cur: ") +
           String((int)readCurrent()) + "mA\n" + String("Tmp: ") + String((int)readTemperature()) +
           "C";
}

void controlMotor(int current_ma, String msg_text) {
    dxl.setGoalCurrent(DXL_ID, current_ma, UNIT_MILLI_AMPERE);
    currentCmdText = msg_text;
}

// ROS Callback
void messageCb(const std_msgs::String& msg) {
    String cmd = msg.data;
    lastCommandTime = millis();

    if (cmd == "Forward") {
        controlMotor(500, "Forward");
    } else if (cmd == "Stop") {
        controlMotor(0, "Stop");
    } else if (cmd == "Reverse") {
        controlMotor(-800, "Reverse");
    } else {
        controlMotor(0, "Unknown");
    }
}

ros::Subscriber<std_msgs::String> sub("motor_command", &messageCb);

// ==========================================
//  Main Setup & Loop
// ==========================================

void setup() {
    // M5Unified初期化
    auto cfg = M5.config();
    M5.begin(cfg);

    initLCD();
    setupDXL();

    printToLCD("Connecting to WiFi...\nSSID: " + String(ssid));

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        printToLCD("Connecting to WiFi...\n.");
    }

    printToLCD("WiFi Connected!\nSSID: " + String(ssid) + "\nIP: " + WiFi.localIP().toString() +
               "\nConnecting ROS...");

    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    nh.subscribe(sub);
    status_msg.data_length = 3;  // データは3個 (Voltage, Current, Temp)
    status_msg.data = (float*)malloc(sizeof(float) * 3);
    nh.advertise(pub_status);
}

void loop() {
    M5.update();

    // 1. WiFi未接続時
    if (WiFi.status() != WL_CONNECTED) {
        dxl.setGoalCurrent(DXL_ID, 0, UNIT_MILLI_AMPERE);

        // エラー時は即時表示してOK（頻繁に切り替わらないため）
        if (millis() - lastDisplayTime > DISPLAY_INTERVAL) {
            printToLCD("SSID: " + String(ssid) + "\nWiFi: Lost\nReconnecting...");
            lastDisplayTime = millis();
        }
        delay(100);
        return;
    }

    // 2. ROS未接続時
    if (!nh.connected()) {
        nh.spinOnce();
        dxl.setGoalCurrent(DXL_ID, 0, UNIT_MILLI_AMPERE);

        if (millis() - lastDisplayTime > DISPLAY_INTERVAL) {
            printToLCD(getStatusHeader() + "ROS: No Connection");
            lastDisplayTime = millis();
        }
        delay(10);
        return;
    }

    // 3. 正常接続時
    nh.spinOnce();  // ここで controlMotor が呼ばれるが、画面更新はされない

    // 安全装置チェック
    bool isSafeStop = (millis() - lastCommandTime > SAFETY_TIMEOUT);
    if (isSafeStop) {
        dxl.setGoalCurrent(DXL_ID, 0, UNIT_MILLI_AMPERE);
        currentCmdText = "(No Command)";
    }

    // --- データ送信処理 (5Hz) ---
    if (millis() - lastPublishTime > PUBLISH_INTERVAL) {
        status_msg.data[0] = readVoltage();      // float
        status_msg.data[1] = readCurrent();      // float (ROSへは小数のまま送信)
        status_msg.data[2] = readTemperature();  // float
        pub_status.publish(&status_msg);
        lastPublishTime = millis();
    }

    // 一定時間（0.5秒）経過していたら画面を更新する
    if (millis() - lastDisplayTime > DISPLAY_INTERVAL) {
        String status = getStatusHeader();

        if (isSafeStop) {
            status += "ROS: Connected (Idle)";
            status += "\n\nSAFETY STOP\n" + currentCmdText;
        } else {
            status += "ROS: Connected";
            status += "\n\nCMD: " + currentCmdText;
        }

        // ここで初めて電圧などを読んで表示（チカチカ防止）
        status += getMotorInfo();

        printToLCD(status);
        lastDisplayTime = millis();
    }

    delay(1);
}
