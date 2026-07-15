

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "driver/twai.h"

// ======================= WIFI CONFIG =======================
// Sửa đúng WiFi nhà bạn
const char* WIFI_SSID = "Student";
const char* WIFI_PASS = "";

// ESP32 sẽ phát AP dự phòng để bạn vẫn truy cập được web
const char* AP_SSID = "ESP32_BMS_REAL";
const char* AP_PASS = "12345678";

WebServer server(80);

// ======================= MQTT CONFIG VPS =======================
// VPS đã cấu hình Mosquitto MQTT Broker.
// ESP32 publish dữ liệu thật lên topic bms/data.
// NodeJS trên VPS subscribe topic này và lưu vào MySQL.
const char* MQTT_SERVER = "103.228.37.220";
const int   MQTT_PORT   = 1883;

const char* MQTT_USER = "bms_user";
const char* MQTT_PASS = "Bms12345678";

const char* DEVICE_ID = "BMS_01";
const char* MQTT_CLIENT_ID = "ESP32_BMS_01";
const char* MQTT_TOPIC_DATA = "bms/data";
const char* MQTT_TOPIC_STATUS = "bms/device01/status";
String MQTT_TOPIC_CMD = String("bms/cmd/") + String(DEVICE_ID);

WiFiClient espWifiClient;
PubSubClient mqttClient(espWifiClient);

uint32_t lastMqttPublishMs = 0;
uint32_t lastMqttReconnectMs = 0;
uint32_t lastWiFiReconnectMs = 0;
uint32_t mqttPublishOk = 0;
uint32_t mqttPublishFail = 0;

#define MQTT_PUBLISH_INTERVAL_MS 1000


// ======================= MCP2551 / ESP32 TWAI CONFIG =======================
// Không còn CS/INT/SPI như MCP2515.
// MCP2551 dùng TXD/RXD nối vào TWAI nội bộ ESP32.
#define CAN_TX_GPIO 22
#define CAN_RX_GPIO 21

// BMS của bạn đang dùng 500 kbps
#define CAN_SPEED_500K 1

// ======================= CAN ID CONFIG =======================
#define CAN_ID_BMS_BASIC_1     0x300
#define CAN_ID_BMS_BASIC_2     0x301
#define CAN_ID_BMS_BASIC_3     0x302
#define CAN_ID_BMS_CELL_STAT   0x305

#define CAN_ID_BMS_CELL_1_4    0x310
#define CAN_ID_BMS_CELL_5_8    0x311
#define CAN_ID_BMS_CELL_9_12   0x312
#define CAN_ID_BMS_CELL_13_16  0x313

#define GOT_BASIC1    0x0001
#define GOT_BASIC2    0x0002
#define GOT_BASIC3    0x0004
#define GOT_CELLSTAT  0x0008
#define GOT_CELL0     0x0010
#define GOT_CELL1     0x0020
#define GOT_CELL2     0x0040
#define GOT_CELL3     0x0080

#define FULL_MASK     0x00FF
#define CORE_MASK     (GOT_BASIC1 | GOT_BASIC2 | GOT_BASIC3 | GOT_CELLSTAT)

// ======================= GLOBAL STATE =======================
uint32_t rxCount = 0;
uint32_t rxTotal = 0;
uint32_t lastStatMs = 0;
uint32_t lastCanFrameMs = 0;
uint32_t lastFullDataMs = 0;
uint32_t lastCanRetryMs = 0;


uint32_t lastCoreDataOkMs = 0;


#define CORE_DATA_GRACE_MS 1500

uint8_t currentSeq = 255;
uint8_t lastPrintedSeq = 255;
uint16_t gotMask = 0;

bool bmsDataFull = false;
bool canReady = false;
bool twaiInstalled = false;

// ======================= SETTINGS FOR WEB =======================
// Các ngưỡng này dùng cho web tô màu/cảnh báo.
// Chưa phải lệnh cài trực tiếp xuống BMS thật.
float temp_warn = 45.0;
float temp_cutoff = 60.0;
uint16_t cell_ov_mv = 4200;
uint16_t cell_uv_mv = 3000;
float overcurr_a = 40.0;
uint8_t auto_chg = 0;
uint8_t auto_dsg = 0;

// ======================= REAL CUT-OFF OUTPUT - LEGACY =======================
// Hai chân này giữ lại cho logic ngắt sạc/ngắt xả cũ.
// Để tránh trùng với 4 relay tải thử nghiệm, CHG/DSG chuyển sang GPIO32/GPIO33.
// Nếu bạn không dùng relay CHG/DSG riêng thì không cần đấu hai chân này.
#define RELAY_CHG_GPIO 32
#define RELAY_DSG_GPIO 33
#define RELAY_ACTIVE_LOW true


#define LOAD_RELAY_1_GPIO 25
#define LOAD_RELAY_2_GPIO 26
#define LOAD_RELAY_3_GPIO 27
#define LOAD_RELAY_4_GPIO 14
#define LOAD_RELAY_ACTIVE_LOW false

// Bật kiểm tra an toàn trước khi cho phép đóng tải.
// Khi mới test LED/relay chưa đấu tải thật, có thể đổi false.
// Khi đấu điện trở tải công suất, nên để true.
#define LOAD_SAFETY_CHECK_ENABLED true

int currentLoadW = 0;

int requestedLoadW = 0;
uint32_t lastLoadRampMs = 0;
uint32_t lastLoadSafetyStepMs = 0;
#define LOAD_STEP_W 250
#define LOAD_RAMP_STEP_INTERVAL_MS 400     // thời gian tối thiểu giữa mỗi nấc tăng, để relay/tải ổn định
#define LOAD_SAFETY_STEP_INTERVAL_MS 500   // thời gian tối thiểu giữa mỗi nấc giảm khi mất an toàn

bool chgPathEnabled = true;
bool dsgPathEnabled = true;
bool softwareChgCut = false;
bool softwareDsgCut = false;
String cutoffReason = "";
uint32_t lastProtectionCheckMs = 0;

// ======================= ALERT LIST =======================
struct AlertItem {
  String msg;
  uint32_t t;
};

#define MAX_ALERTS 16
AlertItem alerts[MAX_ALERTS];
uint8_t alertCount = 0;

void addAlert(const String &msg) {
  if (alertCount > 0 && alerts[alertCount - 1].msg == msg) return;

  if (alertCount >= MAX_ALERTS) {
    for (uint8_t i = 1; i < MAX_ALERTS; i++) {
      alerts[i - 1] = alerts[i];
    }
    alertCount = MAX_ALERTS - 1;
  }

  alerts[alertCount].msg = msg;
  alerts[alertCount].t = millis();
  alertCount++;
}

void clearAlerts() {
  alertCount = 0;
}

// ======================= BMS DATA STRUCT =======================
typedef struct
{
  uint16_t voltage_001V;       // 0.01V
  int16_t  current_001A;       // 0.01A
  uint16_t capacity_001Ah;     // 0.01Ah
  uint16_t nominal_001Ah;      // 0.01Ah

  uint16_t cycles;
  uint16_t protect;

  uint8_t soc;
  uint8_t fet;
  uint8_t cellCount;
  uint8_t softwareVersion;

  int16_t temp_01C;            // 0.1°C

  uint16_t cellMv[16];
  uint16_t cellTotalMv;
  uint16_t cellMinMv;
  uint16_t cellMaxMv;
  uint16_t cellDiffMv;

  uint8_t seq;
} BmsCanData_t;

BmsCanData_t bms;

// ======================= ROOT PAGE =======================
const char ROOT_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ESP32 MCP2551 BMS API</title>
  <style>
    body{font-family:Arial;background:#07101d;color:#eef6ff;padding:24px;line-height:1.5}
    a{color:#19e6a2;display:block;margin:8px 0;text-decoration:none}
    code{background:#111a2e;padding:4px 8px;border-radius:6px}
  </style>
</head>
<body>
  <h2>ESP32 + MCP2551 + TWAI BMS API + MQTT</h2>
  <p>Web API đang chạy. ESP32 publish MQTT về VPS NodeJS qua Mosquitto nếu cấu hình đúng MQTT_SERVER.</p>
  <p>API chính: <code>/api/bms</code></p>
  <a href="/api/bms">Xem JSON BMS</a>
  <a href="/api/status">Xem trạng thái ESP32/CAN/MQTT</a>
  <a href="/api/settings">Xem settings</a>
</body>
</html>
)rawliteral";

// ======================= HELPER FUNCTIONS =======================
uint16_t getU16BE(const uint8_t *buf, uint8_t index)
{
  return ((uint16_t)buf[index] << 8) | buf[index + 1];
}

int16_t getS16BE(const uint8_t *buf, uint8_t index)
{
  return (int16_t)getU16BE(buf, index);
}

void clearBmsDataForNewSeq(uint8_t newSeq)
{
  memset(&bms, 0, sizeof(bms));

  currentSeq = newSeq;
  bms.seq = newSeq;
  gotMask = 0;
  bmsDataFull = false;
}

void printMissingFrames()
{
  if (!(gotMask & GOT_BASIC1))   Serial.println("MISS ID 0x300 BASIC1: VOLT/CUR/CAP/SOC/SEQ");
  if (!(gotMask & GOT_BASIC2))   Serial.println("MISS ID 0x301 BASIC2: NOMCAP/CYCLES/PROTECT/FET/CELLCOUNT");
  if (!(gotMask & GOT_BASIC3))   Serial.println("MISS ID 0x302 BASIC3: TEMP/SW");
  if (!(gotMask & GOT_CELLSTAT)) Serial.println("MISS ID 0x305 CELL_STAT: TOTAL/MIN/MAX/DIFF");
  if (!(gotMask & GOT_CELL0))    Serial.println("MISS ID 0x310 CELL 1-4");
  if (!(gotMask & GOT_CELL1))    Serial.println("MISS ID 0x311 CELL 5-8");
  if (!(gotMask & GOT_CELL2))    Serial.println("MISS ID 0x312 CELL 9-12");
  if (!(gotMask & GOT_CELL3))    Serial.println("MISS ID 0x313 CELL 13-16");
}

void printRawFrame(uint32_t id, uint8_t frameLen, const uint8_t *data)
{
  Serial.print("RAW ID=0x");
  Serial.print(id, HEX);
  Serial.print(" DLC=");
  Serial.print(frameLen);
  Serial.print(" DATA=");

  for (uint8_t i = 0; i < frameLen; i++) {
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }

  Serial.println();
}

void printBmsData()
{
  Serial.println("========== BMS CAN DATA ==========");

  Serial.print("BASIC,VOLT=");
  Serial.print(bms.voltage_001V / 100.0, 2);

  Serial.print(",CUR=");
  Serial.print(bms.current_001A / 100.0, 2);

  Serial.print(",CAP=");
  Serial.print(bms.capacity_001Ah / 100.0, 2);

  Serial.print(",NOMCAP=");
  Serial.print(bms.nominal_001Ah / 100.0, 2);

  Serial.print(",SOC=");
  Serial.print(bms.soc);

  Serial.print(",CYCLES=");
  Serial.print(bms.cycles);

  Serial.print(",PROTECT=");
  Serial.print(bms.protect);

  Serial.print(",FET=");
  Serial.print(bms.fet);

  Serial.print(",CELLCOUNT=");
  Serial.print(bms.cellCount);

  Serial.print(",SW=");
  Serial.print(bms.softwareVersion);

  Serial.print(",TEMP=");
  Serial.print(bms.temp_01C / 10.0, 1);

  Serial.print(",SEQ=");
  Serial.println(bms.seq);

  Serial.print("CELL");

  uint8_t n = bms.cellCount;
  if (n == 0 || n > 16) n = 16;

  for (uint8_t i = 0; i < n; i++) {
    Serial.print(",C");
    Serial.print(i + 1);
    Serial.print("=");
    Serial.print(bms.cellMv[i] / 1000.0, 3);
  }

  Serial.print(",TOTAL=");
  Serial.print(bms.cellTotalMv / 1000.0, 3);

  Serial.print(",MIN=");
  Serial.print(bms.cellMinMv / 1000.0, 3);

  Serial.print(",MAX=");
  Serial.print(bms.cellMaxMv / 1000.0, 3);

  Serial.print(",DIFF=");
  Serial.println(bms.cellDiffMv);

  Serial.print("gotMask=0x");
  Serial.println(gotMask, HEX);

  Serial.println("==================================");
}

// ======================= CAN PARSER =======================
void parseCanFrame(uint32_t id, uint8_t frameLen, const uint8_t *data)
{
  lastCanFrameMs = millis();

  if (frameLen != 8) {
    Serial.print("LEN ERROR ID=0x");
    Serial.println(id, HEX);
    return;
  }

  if (id == CAN_ID_BMS_BASIC_1) {
    uint8_t newSeq = data[7];

    // Mỗi 0x300 có SEQ mới thì coi như một bộ dữ liệu BMS mới.
    if (newSeq != currentSeq) {
      clearBmsDataForNewSeq(newSeq);
    }

    bms.voltage_001V   = getU16BE(data, 0);
    bms.current_001A   = getS16BE(data, 2);
    bms.capacity_001Ah = getU16BE(data, 4);
    bms.soc            = data[6];
    bms.seq            = newSeq;

    gotMask |= GOT_BASIC1;
  }
  else if (id == CAN_ID_BMS_BASIC_2) {
    bms.nominal_001Ah = getU16BE(data, 0);
    bms.cycles        = getU16BE(data, 2);
    bms.protect       = getU16BE(data, 4);
    bms.fet           = data[6];
    bms.cellCount     = data[7];

    gotMask |= GOT_BASIC2;
  }
  else if (id == CAN_ID_BMS_BASIC_3) {
    bms.temp_01C        = getS16BE(data, 0);
    bms.softwareVersion = data[2];

    gotMask |= GOT_BASIC3;
  }
  else if (id == CAN_ID_BMS_CELL_STAT) {
    bms.cellTotalMv = getU16BE(data, 0);
    bms.cellMinMv   = getU16BE(data, 2);
    bms.cellMaxMv   = getU16BE(data, 4);
    bms.cellDiffMv  = getU16BE(data, 6);

    gotMask |= GOT_CELLSTAT;
  }
  else if (id >= CAN_ID_BMS_CELL_1_4 && id <= CAN_ID_BMS_CELL_13_16) {
    uint8_t frameIndex = id - CAN_ID_BMS_CELL_1_4;
    uint8_t baseCell = frameIndex * 4;

    for (uint8_t i = 0; i < 4; i++) {
      uint8_t cellIndex = baseCell + i;

      if (cellIndex < 16) {
        bms.cellMv[cellIndex] = getU16BE(data, i * 2);
      }
    }

    if (frameIndex == 0) gotMask |= GOT_CELL0;
    if (frameIndex == 1) gotMask |= GOT_CELL1;
    if (frameIndex == 2) gotMask |= GOT_CELL2;
    if (frameIndex == 3) gotMask |= GOT_CELL3;
  }
  else {
    printRawFrame(id, frameLen, data);
  }

  // [FIX] Ghi nhận thời điểm gần nhất core data (BASIC1+BASIC2+BASIC3+CELLSTAT)
  // đầy đủ, độc lập với việc gotMask có thể bị xóa lại ngay sau đó khi SEQ mới
  // bắt đầu. isSafeToEnableLoad() sẽ dùng mốc thời gian này thay vì kiểm tra
  // gotMask tức thời để tránh false-trigger tắt relay giữa chừng.
  if ((gotMask & CORE_MASK) == CORE_MASK) {
    lastCoreDataOkMs = millis();
  }

  if ((gotMask & FULL_MASK) == FULL_MASK) {
    bmsDataFull = true;
    lastFullDataMs = millis();

    if (lastPrintedSeq != bms.seq) {
      printBmsData();
      lastPrintedSeq = bms.seq;
    }
  }
}

// ======================= REAL RELAY / CONTACTOR CONTROL =======================
void writeRelayPin(uint8_t pin, bool enabled)
{
  // enabled = cho phép dòng đi qua contactor/MOSFET driver.
  // Với module ACTIVE LOW: LOW = relay ON = contactor đóng = cho phép.
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(pin, enabled ? LOW : HIGH);
  } else {
    digitalWrite(pin, enabled ? HIGH : LOW);
  }
}

void applyRelayOutputs()
{
  writeRelayPin(RELAY_CHG_GPIO, chgPathEnabled);
  writeRelayPin(RELAY_DSG_GPIO, dsgPathEnabled);
}

void setChargePath(bool enabled, const String &reason)
{
  chgPathEnabled = enabled;
  softwareChgCut = !enabled;
  if (!enabled && reason.length()) cutoffReason = reason;
  applyRelayOutputs();

  Serial.print("CHG PATH = ");
  Serial.print(enabled ? "ON" : "OFF");
  Serial.print(" | reason=");
  Serial.println(reason);
}

void setDischargePath(bool enabled, const String &reason)
{
  dsgPathEnabled = enabled;
  softwareDsgCut = !enabled;
  if (!enabled && reason.length()) cutoffReason = reason;
  applyRelayOutputs();

  Serial.print("DSG PATH = ");
  Serial.print(enabled ? "ON" : "OFF");
  Serial.print(" | reason=");
  Serial.println(reason);
}

void initRelayOutputs()
{
  pinMode(RELAY_CHG_GPIO, OUTPUT);
  pinMode(RELAY_DSG_GPIO, OUTPUT);

  // Mặc định cho phép sạc/xả khi ESP32 khởi động.
  // Nếu muốn an toàn hơn, đổi 2 dòng dưới thành false và chỉ bật khi web xác nhận.
  chgPathEnabled = true;
  dsgPathEnabled = true;
  softwareChgCut = false;
  softwareDsgCut = false;
  cutoffReason = "";
  applyRelayOutputs();

  Serial.println("Relay/contactor outputs initialized");
  Serial.print("CHG relay GPIO: "); Serial.println(RELAY_CHG_GPIO);
  Serial.print("DSG relay GPIO: "); Serial.println(RELAY_DSG_GPIO);
}



// ======================= 4-LEVEL LOAD RELAY CONTROL =======================
void writeLoadRelayPin(uint8_t pin, bool enabled)
{
  if (LOAD_RELAY_ACTIVE_LOW) {
    digitalWrite(pin, enabled ? LOW : HIGH);
  } else {
    digitalWrite(pin, enabled ? HIGH : LOW);
  }
}

bool isLoadRelayOn(uint8_t pin)
{
  int v = digitalRead(pin);
  if (LOAD_RELAY_ACTIVE_LOW) {
    return v == LOW;
  }
  return v == HIGH;
}

void allLoadOff()
{
  writeLoadRelayPin(LOAD_RELAY_1_GPIO, false);
  writeLoadRelayPin(LOAD_RELAY_2_GPIO, false);
  writeLoadRelayPin(LOAD_RELAY_3_GPIO, false);
  writeLoadRelayPin(LOAD_RELAY_4_GPIO, false);

  currentLoadW = 0;
  requestedLoadW = 0; // [FIX #3] đồng bộ luôn yêu cầu về 0 để ramp không tự bật lại
  Serial.println("All load relays OFF");
}

void setLoadRelayLevel(int level)
{
  if (level < 0) level = 0;
  if (level > 4) level = 4;

  writeLoadRelayPin(LOAD_RELAY_1_GPIO, level >= 1);
  writeLoadRelayPin(LOAD_RELAY_2_GPIO, level >= 2);
  writeLoadRelayPin(LOAD_RELAY_3_GPIO, level >= 3);
  writeLoadRelayPin(LOAD_RELAY_4_GPIO, level >= 4);

  currentLoadW = level * LOAD_STEP_W;
}

void initLoadRelayOutputs()
{
  pinMode(LOAD_RELAY_1_GPIO, OUTPUT);
  pinMode(LOAD_RELAY_2_GPIO, OUTPUT);
  pinMode(LOAD_RELAY_3_GPIO, OUTPUT);
  pinMode(LOAD_RELAY_4_GPIO, OUTPUT);

  allLoadOff();

  Serial.println("4-level load relay outputs initialized");
  Serial.print("LOAD relay 1 GPIO: "); Serial.println(LOAD_RELAY_1_GPIO);
  Serial.print("LOAD relay 2 GPIO: "); Serial.println(LOAD_RELAY_2_GPIO);
  Serial.print("LOAD relay 3 GPIO: "); Serial.println(LOAD_RELAY_3_GPIO);
  Serial.print("LOAD relay 4 GPIO: "); Serial.println(LOAD_RELAY_4_GPIO);
}

bool hasCoreBmsData();  // forward declare (dùng trong isSafeToEnableLoad nếu cần)

bool isSafeToEnableLoad()
{
#if LOAD_SAFETY_CHECK_ENABLED
  uint32_t now = millis();

  // [FIX] Thay vì kiểm tra hasCoreBmsData() tức thời (dễ false-trigger vì
  // gotMask bị xóa về 0 mỗi khi SEQ mới bắt đầu), ta kiểm tra "độ mới"
  // (age) của lần cuối core data đầy đủ. Miễn là trong khoảng
  // CORE_DATA_GRACE_MS gần đây từng có đủ core data thì vẫn coi là an toàn.
  uint32_t coreAge = lastCoreDataOkMs > 0 ? now - lastCoreDataOkMs : 999999;

  if (coreAge > CORE_DATA_GRACE_MS) {
    Serial.println("Load blocked: core BMS data stale");
    return false;
  }

  uint32_t canAge = lastCanFrameMs > 0 ? now - lastCanFrameMs : 999999;

  if (lastCanFrameMs == 0 || canAge > 3000) {
    Serial.println("Load blocked: CAN/BMS data timeout");
    return false;
  }

  float tempC = bms.temp_01C / 10.0;
  float currentA = bms.current_001A / 100.0;
  float absCurrentA = fabs(currentA);

  if (tempC >= temp_cutoff) {
    Serial.println("Load blocked: temperature cutoff");
    return false;
  }

  if (bms.cellMinMv > 0 && bms.cellMinMv <= cell_uv_mv) {
    Serial.println("Load blocked: low cell voltage");
    return false;
  }

  // UV/OT/OC đang có bảo vệ thì không cho bật tải thử nghiệm.
  // OV không chặn tải, vì tải có thể giúp kéo điện áp xuống.
  if (bms.protect & (0x02 | 0x04 | 0x08)) {
    Serial.println("Load blocked: BMS protect active");
    return false;
  }

  if (absCurrentA >= overcurr_a) {
    Serial.println("Load blocked: over current threshold");
    return false;
  }
#endif

  return true;
}

bool isValidLoadLevel(int watt)
{
  return watt == 0 || watt == 250 || watt == 500 || watt == 750 || watt == 1000;
}


void setLoadLevel(int watt)
{
  if (!isValidLoadLevel(watt)) {
    Serial.print("Invalid load level: ");
    Serial.println(watt);
    return;
  }

  if (watt == 0) {
    allLoadOff();
    addAlert("Web command: LOAD OFF");
    Serial.println("Load level requested: 0W (OFF)");
    return;
  }

  if (!isSafeToEnableLoad()) {
    Serial.println("Unsafe to enable load right now. Yeu cau bi tu choi.");
    addAlert("LOAD BLOCKED - Dieu kien pin khong an toan luc yeu cau");
    // Giữ nguyên mức đang bật (nếu có), không đặt yêu cầu cao hơn mức an toàn.
    requestedLoadW = currentLoadW;
    return;
  }

  requestedLoadW = watt;
  lastLoadRampMs = 0; // cho phép manageLoadRamp() chạy ngay ở vòng loop kế tiếp

  Serial.print("Load level requested: ");
  Serial.print(watt);
  Serial.println("W (se tang dan tung nac 250W)");

  addAlert("Web command: REQUEST LOAD " + String(watt) + "W");
}

void manageLoadRamp()
{
  uint32_t now = millis();
  if (now - lastLoadRampMs < LOAD_RAMP_STEP_INTERVAL_MS) return;

  int currentLevel = currentLoadW / LOAD_STEP_W;
  int requestedLevel = requestedLoadW / LOAD_STEP_W;

  if (currentLevel == requestedLevel) return;

  lastLoadRampMs = now;

  if (requestedLevel < currentLevel) {
    // Giảm tải theo yêu cầu người dùng: luôn cho phép ngay.
    currentLevel--;
    setLoadRelayLevel(currentLevel);
    Serial.print("Load ramp DOWN (theo yeu cau) -> ");
    Serial.print(currentLoadW);
    Serial.println("W");
    addAlert("LOAD -> " + String(currentLoadW) + "W");
    return;
  }

  // requestedLevel > currentLevel: muốn tăng thêm 1 nấc.
  if (!isSafeToEnableLoad()) {
    Serial.print("Load ramp UP blocked: khong an toan de tang qua ");
    Serial.print(currentLoadW);
    Serial.println("W. Giu nguyen muc hien tai.");

    addAlert("LOAD HOLD - Khong an toan de tang len " + String((currentLevel + 1) * LOAD_STEP_W) + "W, giu " + String(currentLoadW) + "W");

    // Đồng bộ lại yêu cầu = mức đang giữ, để không cứ thử tăng lại mỗi vòng loop.
    requestedLoadW = currentLoadW;
    return;
  }

  currentLevel++;
  setLoadRelayLevel(currentLevel);

  Serial.print("Load ramp UP -> ");
  Serial.print(currentLoadW);
  Serial.println("W");

  addAlert("LOAD -> " + String(currentLoadW) + "W");
}


void maintainLoadSafety()
{
  if (currentLoadW <= 0) return;
  if (isSafeToEnableLoad()) return;

  uint32_t now = millis();
  if (now - lastLoadSafetyStepMs < LOAD_SAFETY_STEP_INTERVAL_MS) return;
  lastLoadSafetyStepMs = now;

  int level = (currentLoadW / LOAD_STEP_W) - 1;
  if (level < 0) level = 0;

  setLoadRelayLevel(level);
  requestedLoadW = currentLoadW; // đồng bộ để manageLoadRamp() không tự tăng lại mức cũ

  Serial.print("Load safety step-down -> ");
  Serial.print(currentLoadW);
  Serial.println("W");

  addAlert("LOAD AUTO STEP-DOWN - Bao ve tai thu nghiem, giam ve " + String(currentLoadW) + "W");
}


void checkSoftwareProtection()
{
  if ((gotMask & CORE_MASK) != CORE_MASK) return;

  uint32_t now = millis();
  if (now - lastProtectionCheckMs < 300) return;
  lastProtectionCheckMs = now;

  float currentA = bms.current_001A / 100.0;
  float absCurrentA = fabs(currentA);
  float tempC = bms.temp_01C / 10.0;

  if (tempC >= temp_warn) {
    addAlert("TEMP WARN - Nhiệt độ cao: " + String(tempC, 1) + "C");
  }

  if (tempC >= temp_cutoff) {
    addAlert("TEMP CUTOFF - Quá nhiệt, yêu cầu ngắt");
    if (auto_chg && chgPathEnabled) {
      setChargePath(false, "auto_temp_cutoff_charge");
    }
    if (auto_dsg && dsgPathEnabled) {
      setDischargePath(false, "auto_temp_cutoff_discharge");
    }
  }

  if (absCurrentA >= overcurr_a) {
    addAlert("OVERCURRENT - Quá dòng: " + String(currentA, 2) + "A");
    if (auto_dsg && dsgPathEnabled) {
      setDischargePath(false, "auto_overcurrent_discharge");
    }
  }

  if (bms.cellMaxMv > 0 && bms.cellMaxMv >= cell_ov_mv) {
    addAlert("CELL OV - Cell quá áp");
    if (auto_chg && chgPathEnabled) {
      setChargePath(false, "auto_cell_overvoltage_charge");
    }
  }

  if (bms.cellMinMv > 0 && bms.cellMinMv <= cell_uv_mv) {
    addAlert("CELL UV - Cell thấp áp");
    if (auto_dsg && dsgPathEnabled) {
      setDischargePath(false, "auto_cell_undervoltage_discharge");
    }
  }
}

void updateAlertsFromBms()
{
  bool balActive = bms.cellDiffMv > 35;

  if (bms.protect & 0x01) addAlert("OV PROT - Cell quá áp");
  if (bms.protect & 0x02) addAlert("UV PROT - Cell thấp áp");
  if (bms.protect & 0x04) addAlert("OT PROT - Quá nhiệt");
  if (bms.protect & 0x08) addAlert("OC PROT - Quá dòng");
  if (balActive && bms.cellDiffMv > 80) addAlert("CELL IMBALANCE - Lệch áp cell cao");
}

bool hasCoreBmsData()
{
  return (gotMask & CORE_MASK) == CORE_MASK;
}

bool hasFullBmsData()
{
  return (gotMask & FULL_MASK) == FULL_MASK;
}

String getBmsStatusText()
{
  if (bms.protect == 0) return "normal";

  String s = "warning";
  if (bms.protect & 0x01) s += ",ov";
  if (bms.protect & 0x02) s += ",uv";
  if (bms.protect & 0x04) s += ",ot";
  if (bms.protect & 0x08) s += ",oc";
  return s;
}

// ======================= MQTT FUNCTIONS =======================
void maintainWiFi()
{
  if (WiFi.status() == WL_CONNECTED) return;

  uint32_t now = millis();
  if (now - lastWiFiReconnectMs < 10000) return;
  lastWiFiReconnectMs = now;

  Serial.println("WiFi STA lost. Reconnecting...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void handleMqttCommand(char* topic, byte* payload, unsigned int length)
{
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  Serial.print("MQTT CMD topic=");
  Serial.print(topic);
  Serial.print(" payload=");
  Serial.println(msg);

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, msg);

  String cmdName = msg;
  if (!err) {
    if (doc["cmd"].is<const char*>()) cmdName = doc["cmd"].as<String>();
    if (doc["name"].is<const char*>()) cmdName = doc["name"].as<String>();

    if (cmdName == "set_load") {
      String deviceId = doc["device_id"] | "";
      int loadW = doc["load_w"] | 0;

      if (deviceId.length() > 0 && deviceId != String(DEVICE_ID)) {
        Serial.println("MQTT set_load ignored: device_id not match");
        return;
      }

      setLoadLevel(loadW);
      return;
    }

    if (cmdName == "settings") {
      if (doc["temp_warn"].is<float>()) temp_warn = doc["temp_warn"].as<float>();
      if (doc["temp_cutoff"].is<float>()) temp_cutoff = doc["temp_cutoff"].as<float>();
      if (doc["cell_ov_mv"].is<uint16_t>()) cell_ov_mv = doc["cell_ov_mv"].as<uint16_t>();
      if (doc["cell_uv_mv"].is<uint16_t>()) cell_uv_mv = doc["cell_uv_mv"].as<uint16_t>();
      if (doc["overcurr_a"].is<float>()) overcurr_a = doc["overcurr_a"].as<float>();
      if (doc["auto_chg"].is<int>()) auto_chg = doc["auto_chg"].as<int>() ? 1 : 0;
      if (doc["auto_dsg"].is<int>()) auto_dsg = doc["auto_dsg"].as<int>() ? 1 : 0;
      addAlert("MQTT settings updated from VPS");
      return;
    }
  }

  cmdName.toLowerCase();

  if (cmdName == "chg_on") {
    setChargePath(true, "web_chg_on");
    addAlert("Web command: CHG ON");
  }
  else if (cmdName == "chg_off") {
    setChargePath(false, "web_chg_off");
    addAlert("Web command: CHG OFF");
  }
  else if (cmdName == "dsg_on") {
    setDischargePath(true, "web_dsg_on");
    addAlert("Web command: DSG ON");
  }
  else if (cmdName == "dsg_off") {
    setDischargePath(false, "web_dsg_off");
    addAlert("Web command: DSG OFF");
  }
  else if (cmdName == "clear_alarm") {
    clearAlerts();
    cutoffReason = "";
  }
  else if (cmdName == "load_0" || cmdName == "load_off") {
    setLoadLevel(0);
  }
  else if (cmdName == "load_250") {
    setLoadLevel(250);
  }
  else if (cmdName == "load_500") {
    setLoadLevel(500);
  }
  else if (cmdName == "load_750") {
    setLoadLevel(750);
  }
  else if (cmdName == "load_1000") {
    setLoadLevel(1000);
  }
  else {
    addAlert("Unknown MQTT command: " + cmdName);
  }
}

void reconnectMqtt()
{
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqttClient.connected()) return;

  uint32_t now = millis();
  if (now - lastMqttReconnectMs < 5000) return;
  lastMqttReconnectMs = now;

  Serial.print("Connecting MQTT Broker: ");
  Serial.print(MQTT_SERVER);
  Serial.print(":");
  Serial.println(MQTT_PORT);

  bool ok = mqttClient.connect(
    MQTT_CLIENT_ID,
    MQTT_USER,
    MQTT_PASS,
    MQTT_TOPIC_STATUS,
    0,
    true,
    "offline"
  );

  if (ok) {
    Serial.println("MQTT connected OK");
    mqttClient.publish(MQTT_TOPIC_STATUS, "online", true);
    mqttClient.subscribe(MQTT_TOPIC_CMD.c_str());
    Serial.print("Subscribed CMD topic: ");
    Serial.println(MQTT_TOPIC_CMD);
  }
  else {
    Serial.print("MQTT connect failed, state=");
    Serial.println(mqttClient.state());
  }
}

void publishBmsToMqtt()
{
  if (!mqttClient.connected()) return;
  if (!hasCoreBmsData()) return;

  updateAlertsFromBms();
  checkSoftwareProtection();

  StaticJsonDocument<6144> doc;

  uint32_t now = millis();
  uint32_t canAge = lastCanFrameMs > 0 ? now - lastCanFrameMs : 999999;
  bool canOnline = lastCanFrameMs > 0 && canAge < 3000;
  bool balActive = bms.cellDiffMv > 35;

  float packV = bms.voltage_001V / 100.0;
  float currentA = bms.current_001A / 100.0;
  float tempC = bms.temp_01C / 10.0;
  float powerW = packV * currentA;

  doc["device_id"] = DEVICE_ID;
  doc["type"] = "bms_data";
  doc["seq"] = bms.seq;
  doc["uptime_ms"] = now;

  doc["can_online"] = canOnline;
  doc["can_ready"] = canReady;
  doc["can_age_ms"] = canAge;
  doc["data_core"] = hasCoreBmsData();
  doc["data_full"] = hasFullBmsData();
  doc["got_mask"] = gotMask;

  // Tên field theo server VPS/MySQL
  doc["voltage"] = packV;
  doc["current_amp"] = currentA;
  doc["current"] = currentA;
  doc["power_w"] = powerW;
  doc["power"] = powerW;
  doc["soc"] = bms.soc;
  doc["temperature"] = tempC;
  doc["mos_temp"] = tempC;
  doc["alarm_status"] = getBmsStatusText();

  // Tên field theo dashboard HTML cũ của bạn
  doc["pack_v"] = packV;
  doc["current_a"] = currentA;
  doc["temp_c"] = tempC;
  doc["delta_mv"] = bms.cellDiffMv;
  doc["cap_remain"] = bms.capacity_001Ah / 100.0;
  doc["cap_full"] = bms.nominal_001Ah / 100.0;

  if (currentA > 0.05) doc["charge_status"] = "charging";
  else if (currentA < -0.05) doc["charge_status"] = "discharging";
  else doc["charge_status"] = "standby";

  doc["capacity_ah"] = bms.capacity_001Ah / 100.0;
  doc["nominal_ah"] = bms.nominal_001Ah / 100.0;
  doc["cycles"] = bms.cycles;
  doc["protect"] = bms.protect;
  doc["fet"] = bms.fet;
  doc["cell_count"] = bms.cellCount;
  doc["cellCount"] = bms.cellCount;
  doc["software_version"] = bms.softwareVersion;

  doc["cell_total_v"] = bms.cellTotalMv / 1000.0;
  doc["cell_min_v"] = bms.cellMinMv / 1000.0;
  doc["cell_max_v"] = bms.cellMaxMv / 1000.0;
  doc["cell_diff_mv"] = bms.cellDiffMv;

  doc["chg_mos"] = (bms.fet & 0x01) ? true : false;
  doc["dsg_mos"] = (bms.fet & 0x02) ? true : false;
  doc["bal_active"] = balActive;

  doc["ov_prot"] = (bms.protect & 0x01) ? true : false;
  doc["uv_prot"] = (bms.protect & 0x02) ? true : false;
  doc["ot_prot"] = (bms.protect & 0x04) ? true : false;
  doc["oc_prot"] = (bms.protect & 0x08) ? true : false;

  // Trạng thái relay ngắt thật do ESP32 điều khiển
  doc["relay_chg_enabled"] = chgPathEnabled;
  doc["relay_dsg_enabled"] = dsgPathEnabled;
  doc["software_chg_cut"] = softwareChgCut;
  doc["software_dsg_cut"] = softwareDsgCut;
  doc["cutoff_reason"] = cutoffReason;
  doc["auto_chg"] = auto_chg;
  doc["auto_dsg"] = auto_dsg;

  // Trạng thái 4 relay tải thử nghiệm gửi lên VPS/web
  doc["load_level_w"] = currentLoadW;
  doc["requested_load_w"] = requestedLoadW;
  doc["load_power_est"] = currentLoadW;
  doc["load_relay_1"] = isLoadRelayOn(LOAD_RELAY_1_GPIO) ? "ON" : "OFF";
  doc["load_relay_2"] = isLoadRelayOn(LOAD_RELAY_2_GPIO) ? "ON" : "OFF";
  doc["load_relay_3"] = isLoadRelayOn(LOAD_RELAY_3_GPIO) ? "ON" : "OFF";
  doc["load_relay_4"] = isLoadRelayOn(LOAD_RELAY_4_GPIO) ? "ON" : "OFF";

  uint8_t n = bms.cellCount;
  if (n == 0 || n > 16) n = 16;

  JsonArray cells = doc.createNestedArray("cells");
  JsonArray cellMv = doc.createNestedArray("cell_mv");
  JsonArray cellMV2 = doc.createNestedArray("cell_mV");
  JsonArray cellsMv = doc.createNestedArray("cells_mv");

  for (uint8_t i = 0; i < n; i++) {
    cells.add(bms.cellMv[i]);
    cellMv.add(bms.cellMv[i]);
    cellMV2.add(bms.cellMv[i]);
    cellsMv.add(bms.cellMv[i]);

    String key = "cell" + String(i + 1) + "_mv";
    doc[key] = bms.cellMv[i];
  }

  JsonArray arr = doc.createNestedArray("alerts");
  for (uint8_t i = 0; i < alertCount; i++) {
    JsonObject a = arr.createNestedObject();
    a["msg"] = alerts[i].msg;
    a["t"] = alerts[i].t;
  }

  String payload;
  serializeJson(doc, payload);

  bool ok = mqttClient.publish(MQTT_TOPIC_DATA, payload.c_str());

  if (ok) {
    mqttPublishOk++;
    Serial.print("MQTT publish OK: ");
    Serial.println(payload);
  }
  else {
    mqttPublishFail++;
    Serial.println("MQTT publish FAILED");
  }
}

void publishBmsToMqttIfNeeded()
{
  if (!mqttClient.connected()) return;
  if (!hasCoreBmsData()) return;

  uint32_t now = millis();
  if (now - lastMqttPublishMs < MQTT_PUBLISH_INTERVAL_MS) return;

  lastMqttPublishMs = now;
  publishBmsToMqtt();
}


// ======================= TWAI / MCP2551 FUNCTIONS =======================
bool initTWAI()
{
  Serial.println("Init ESP32 TWAI for MCP2551...");

  if (twaiInstalled) {
    twai_stop();
    twai_driver_uninstall();
    twaiInstalled = false;
  }

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_GPIO,
    (gpio_num_t)CAN_RX_GPIO,
    TWAI_MODE_NORMAL
  );

  // 500 kbps
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

  // Nhận tất cả ID. Lọc ID sẽ làm sau nếu cần.
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
  if (ret != ESP_OK) {
    Serial.print("TWAI driver install failed. err=");
    Serial.println((int)ret);
    return false;
  }

  twaiInstalled = true;

  ret = twai_start();
  if (ret != ESP_OK) {
    Serial.print("TWAI start failed. err=");
    Serial.println((int)ret);
    twai_driver_uninstall();
    twaiInstalled = false;
    return false;
  }

  Serial.println("TWAI start OK");
  Serial.print("CAN TX GPIO: ");
  Serial.println(CAN_TX_GPIO);
  Serial.print("CAN RX GPIO: ");
  Serial.println(CAN_RX_GPIO);
  Serial.println("Waiting CAN frames from MCP2551...");

  return true;
}

void readCanFrames()
{
  uint16_t frameLimit = 50;

  while (frameLimit > 0) {
    twai_message_t message;

    esp_err_t ret = twai_receive(&message, 0);
    if (ret != ESP_OK) {
      break;
    }

    frameLimit--;

    // Bỏ qua remote frame
    if (message.rtr) {
      continue;
    }

    uint32_t cleanId = message.identifier;
    if (!message.extd) {
      cleanId = cleanId & 0x7FF;
    }

    uint8_t dlc = message.data_length_code;
    if (dlc > 8) dlc = 8;

    rxCount++;
    rxTotal++;

    parseCanFrame(cleanId, dlc, message.data);
  }
}

// ======================= WIFI =======================
void connectWiFiOrStartAP()
{
  // AP + STA: ESP32 luôn phát WiFi riêng, đồng thời vẫn kết nối WiFi nhà.
  WiFi.mode(WIFI_AP_STA);

  bool apOk = WiFi.softAP(AP_SSID, AP_PASS);

  Serial.println();
  Serial.println("ESP32 MCP2551 BMS Receiver + Web API Start");

  if (apOk) {
    Serial.println("WiFi AP started for backup access");
    Serial.print("AP SSID: ");
    Serial.println(AP_SSID);
    Serial.print("AP PASS: ");
    Serial.println(AP_PASS);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
  }
  else {
    Serial.println("WiFi AP start failed");
  }

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting WiFi STA: ");
  Serial.println(WIFI_SSID);

  uint32_t startMs = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 12000) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi STA connected OK");
    Serial.print("STA IP: ");
    Serial.println(WiFi.localIP());
  }
  else {
    Serial.println("WiFi STA failed. AP is still available.");
    Serial.println("Open AP web: http://192.168.4.1/");
  }
}

// ======================= WEB FUNCTIONS =======================
void addCors()
{
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void sendJson(const String &json)
{
  addCors();
  server.send(200, "application/json", json);
}

void handleRoot()
{
  addCors();
  server.send_P(200, "text/html; charset=utf-8", ROOT_PAGE);
}

void handleBmsApi()
{
  StaticJsonDocument<4096> doc;

  uint32_t now = millis();
  uint32_t canAge = lastCanFrameMs > 0 ? now - lastCanFrameMs : 999999;

  bool canOnline = lastCanFrameMs > 0 && canAge < 3000;
  bool fullNow = ((gotMask & FULL_MASK) == FULL_MASK);
  bool balActive = bms.cellDiffMv > 35;

  uint8_t n = bms.cellCount;
  if (n == 0 || n > 16) n = 16;

  updateAlertsFromBms();

  // Trạng thái chung
  doc["ok"] = true;
  doc["interface"] = "ESP32_TWAI_MCP2551";
  doc["can_online"] = canOnline;
  doc["can_ready"] = canReady;
  doc["data_full"] = fullNow;
  doc["bms_data_full"] = bmsDataFull;
  doc["got_mask"] = gotMask;
  doc["full_mask"] = FULL_MASK;
  doc["rx_total"] = rxTotal;
  doc["rx_count_last_sec"] = rxCount;
  doc["can_age_ms"] = canAge;
  doc["last_can_age_ms"] = canAge;
  doc["last_full_age_ms"] = lastFullDataMs > 0 ? now - lastFullDataMs : 999999;
  doc["uptime_ms"] = now;

  // Dữ liệu raw đúng đơn vị truyền CAN
  doc["voltage_001V"] = bms.voltage_001V;
  doc["current_001A"] = bms.current_001A;
  doc["capacity_001Ah"] = bms.capacity_001Ah;
  doc["nominal_001Ah"] = bms.nominal_001Ah;

  doc["cycles"] = bms.cycles;
  doc["protect"] = bms.protect;

  doc["soc"] = bms.soc;
  doc["fet"] = bms.fet;
  doc["cellCount"] = bms.cellCount;
  doc["cell_count"] = bms.cellCount;
  doc["softwareVersion"] = bms.softwareVersion;
  doc["software_version"] = bms.softwareVersion;

  doc["temp_01C"] = bms.temp_01C;
  doc["seq"] = bms.seq;

  doc["cellTotalMv"] = bms.cellTotalMv;
  doc["cellMinMv"] = bms.cellMinMv;
  doc["cellMaxMv"] = bms.cellMaxMv;
  doc["cellDiffMv"] = bms.cellDiffMv;

  // Dữ liệu đã đổi đơn vị để web dễ dùng
  doc["pack_v"] = bms.voltage_001V / 100.0;
  doc["current_a"] = bms.current_001A / 100.0;
  doc["cap_remain"] = bms.capacity_001Ah / 100.0;
  doc["cap_full"] = bms.nominal_001Ah / 100.0;
  doc["temp_c"] = bms.temp_01C / 10.0;

  doc["cell_total_v"] = bms.cellTotalMv / 1000.0;
  doc["cell_min_v"] = bms.cellMinMv / 1000.0;
  doc["cell_max_v"] = bms.cellMaxMv / 1000.0;
  doc["delta_mv"] = bms.cellDiffMv;

  // Giải mã FET theo quy ước hiện tại
  doc["chg_mos"] = (bms.fet & 0x01) ? true : false;
  doc["dsg_mos"] = (bms.fet & 0x02) ? true : false;
  doc["bal_active"] = balActive;

  // Giải mã protect theo quy ước:
  // bit 0 = OV, bit 1 = UV, bit 2 = OT, bit 3 = OC
  doc["ov_prot"] = (bms.protect & 0x01) ? true : false;
  doc["uv_prot"] = (bms.protect & 0x02) ? true : false;
  doc["ot_prot"] = (bms.protect & 0x04) ? true : false;
  doc["oc_prot"] = (bms.protect & 0x08) ? true : false;

  doc["voltage"] = bms.voltage_001V / 100.0;
  doc["current_amp"] = bms.current_001A / 100.0;
  doc["power_w"] = (bms.voltage_001V / 100.0) * (bms.current_001A / 100.0);
  doc["temperature"] = bms.temp_01C / 10.0;
  doc["mos_temp"] = bms.temp_01C / 10.0;
  doc["alarm_status"] = getBmsStatusText();

  float localCurrentA = bms.current_001A / 100.0;
  if (localCurrentA > 0.05) doc["charge_status"] = "charging";
  else if (localCurrentA < -0.05) doc["charge_status"] = "discharging";
  else doc["charge_status"] = "standby";

  doc["relay_chg_enabled"] = chgPathEnabled;
  doc["relay_dsg_enabled"] = dsgPathEnabled;
  doc["software_chg_cut"] = softwareChgCut;
  doc["software_dsg_cut"] = softwareDsgCut;
  doc["cutoff_reason"] = cutoffReason;

  doc["load_level_w"] = currentLoadW;
  doc["requested_load_w"] = requestedLoadW;
  doc["load_power_est"] = currentLoadW;
  doc["load_relay_1"] = isLoadRelayOn(LOAD_RELAY_1_GPIO) ? "ON" : "OFF";
  doc["load_relay_2"] = isLoadRelayOn(LOAD_RELAY_2_GPIO) ? "ON" : "OFF";
  doc["load_relay_3"] = isLoadRelayOn(LOAD_RELAY_3_GPIO) ? "ON" : "OFF";
  doc["load_relay_4"] = isLoadRelayOn(LOAD_RELAY_4_GPIO) ? "ON" : "OFF";

  // Mảng cell dạng mV. Gửi nhiều tên để HTML nào cũng đọc được.
  JsonArray cellMv = doc.createNestedArray("cellMv");
  JsonArray cellMv2 = doc.createNestedArray("cell_mv");
  JsonArray cellMv3 = doc.createNestedArray("cell_mV");
  JsonArray cells = doc.createNestedArray("cells");

  for (uint8_t i = 0; i < n; i++) {
    cellMv.add(bms.cellMv[i]);
    cellMv2.add(bms.cellMv[i]);
    cellMv3.add(bms.cellMv[i]);
    cells.add(bms.cellMv[i]);

    String key = "cell" + String(i + 1) + "_mv";
    doc[key] = bms.cellMv[i];
  }

  JsonArray arr = doc.createNestedArray("alerts");
  for (uint8_t i = 0; i < alertCount; i++) {
    JsonObject a = arr.createNestedObject();
    a["msg"] = alerts[i].msg;
    a["t"] = alerts[i].t;
  }

  String out;
  serializeJson(doc, out);
  sendJson(out);
}

void handleStatusApi()
{
  StaticJsonDocument<1024> doc;

  uint32_t now = millis();
  uint32_t canAge = lastCanFrameMs > 0 ? now - lastCanFrameMs : 999999;

  doc["ok"] = true;
  doc["interface"] = "ESP32_TWAI_MCP2551";
  doc["ap_ip"] = WiFi.softAPIP().toString();
  doc["sta_ip"] = WiFi.localIP().toString();
  doc["wifi_mode"] = "AP_STA";
  doc["can_ready"] = canReady;
  doc["can_online"] = lastCanFrameMs > 0 && canAge < 3000;
  doc["can_age_ms"] = canAge;
  doc["got_mask"] = gotMask;
  doc["data_full"] = ((gotMask & FULL_MASK) == FULL_MASK);
  doc["rx_total"] = rxTotal;
  doc["current_seq"] = currentSeq;
  doc["tx_gpio"] = CAN_TX_GPIO;
  doc["rx_gpio"] = CAN_RX_GPIO;
  doc["mqtt_server"] = MQTT_SERVER;
  doc["mqtt_port"] = MQTT_PORT;
  doc["mqtt_connected"] = mqttClient.connected();
  doc["mqtt_state"] = mqttClient.state();
  doc["mqtt_topic_data"] = MQTT_TOPIC_DATA;
  doc["mqtt_topic_cmd"] = MQTT_TOPIC_CMD;
  doc["mqtt_publish_ok"] = mqttPublishOk;
  doc["mqtt_publish_fail"] = mqttPublishFail;

  // [FIX] Thêm thông tin độ mới core data để dễ debug từ web/API.
  doc["core_data_age_ms"] = lastCoreDataOkMs > 0 ? (now - lastCoreDataOkMs) : 999999;
  doc["core_data_grace_ms"] = CORE_DATA_GRACE_MS;

  doc["load_level_w"] = currentLoadW;
  doc["requested_load_w"] = requestedLoadW;
  doc["load_relay_1"] = isLoadRelayOn(LOAD_RELAY_1_GPIO) ? "ON" : "OFF";
  doc["load_relay_2"] = isLoadRelayOn(LOAD_RELAY_2_GPIO) ? "ON" : "OFF";
  doc["load_relay_3"] = isLoadRelayOn(LOAD_RELAY_3_GPIO) ? "ON" : "OFF";
  doc["load_relay_4"] = isLoadRelayOn(LOAD_RELAY_4_GPIO) ? "ON" : "OFF";

  String out;
  serializeJson(doc, out);
  sendJson(out);
}

void handleSettingsApi()
{
  StaticJsonDocument<512> doc;

  doc["ok"] = true;
  doc["temp_warn"] = temp_warn;
  doc["temp_cutoff"] = temp_cutoff;
  doc["cell_ov_mv"] = cell_ov_mv;
  doc["cell_uv_mv"] = cell_uv_mv;
  doc["overcurr_a"] = overcurr_a;
  doc["auto_chg"] = auto_chg;
  doc["auto_dsg"] = auto_dsg;

  String out;
  serializeJson(doc, out);
  sendJson(out);
}

void handleSettingsSetApi()
{
  if (server.hasArg("temp_warn")) temp_warn = server.arg("temp_warn").toFloat();
  if (server.hasArg("temp_cutoff")) temp_cutoff = server.arg("temp_cutoff").toFloat();
  if (server.hasArg("cell_ov_mv")) cell_ov_mv = server.arg("cell_ov_mv").toInt();
  if (server.hasArg("cell_uv_mv")) cell_uv_mv = server.arg("cell_uv_mv").toInt();
  if (server.hasArg("overcurr_a")) overcurr_a = server.arg("overcurr_a").toFloat();
  if (server.hasArg("auto_chg")) auto_chg = server.arg("auto_chg").toInt() ? 1 : 0;
  if (server.hasArg("auto_dsg")) auto_dsg = server.arg("auto_dsg").toInt() ? 1 : 0;

  StaticJsonDocument<256> doc;
  doc["ok"] = true;
  doc["msg"] = "settings saved";

  String out;
  serializeJson(doc, out);
  sendJson(out);
}

void handleCmdApi()
{
  String name = server.arg("name");
  name.toLowerCase();

  String msg = "Command received";

  if (name == "clear_alarm") {
    clearAlerts();
    cutoffReason = "";
    msg = "Alarms cleared";
  }
  else if (name == "chg_on") {
    setChargePath(true, "local_web_chg_on");
    addAlert("Local web command: CHG ON");
    msg = "Charge path enabled";
  }
  else if (name == "chg_off") {
    setChargePath(false, "local_web_chg_off");
    addAlert("Local web command: CHG OFF");
    msg = "Charge path cut off";
  }
  else if (name == "dsg_on") {
    setDischargePath(true, "local_web_dsg_on");
    addAlert("Local web command: DSG ON");
    msg = "Discharge/load path enabled";
  }
  else if (name == "dsg_off") {
    setDischargePath(false, "local_web_dsg_off");
    addAlert("Local web command: DSG OFF");
    msg = "Discharge/load path cut off";
  }
  else if (name == "load_0" || name == "load_off") {
    setLoadLevel(0);
    msg = "Load level set to 0W";
  }
  else if (name == "load_250") {
    setLoadLevel(250);
    msg = "Load level set to 250W";
  }
  else if (name == "load_500") {
    setLoadLevel(500);
    msg = "Load level set to 500W";
  }
  else if (name == "load_750") {
    setLoadLevel(750);
    msg = "Load level set to 750W";
  }
  else if (name == "load_1000") {
    setLoadLevel(1000);
    msg = "Load level set to 1000W";
  }
  else if (name.length() > 0) {
    addAlert("Unknown local web command: " + name);
    msg = "Unknown command";
  }

  StaticJsonDocument<512> doc;
  doc["ok"] = true;
  doc["cmd"] = name;
  doc["msg"] = msg;
  doc["relay_chg_enabled"] = chgPathEnabled;
  doc["relay_dsg_enabled"] = dsgPathEnabled;
  doc["software_chg_cut"] = softwareChgCut;
  doc["software_dsg_cut"] = softwareDsgCut;
  doc["cutoff_reason"] = cutoffReason;
  doc["load_level_w"] = currentLoadW;
  doc["requested_load_w"] = requestedLoadW;
  doc["load_relay_1"] = isLoadRelayOn(LOAD_RELAY_1_GPIO) ? "ON" : "OFF";
  doc["load_relay_2"] = isLoadRelayOn(LOAD_RELAY_2_GPIO) ? "ON" : "OFF";
  doc["load_relay_3"] = isLoadRelayOn(LOAD_RELAY_3_GPIO) ? "ON" : "OFF";
  doc["load_relay_4"] = isLoadRelayOn(LOAD_RELAY_4_GPIO) ? "ON" : "OFF";

  String out;
  serializeJson(doc, out);
  sendJson(out);
}

void handleAlertsClearApi()
{
  clearAlerts();

  StaticJsonDocument<128> doc;
  doc["ok"] = true;
  doc["msg"] = "alerts cleared";

  String out;
  serializeJson(doc, out);
  sendJson(out);
}

void handleOptions()
{
  addCors();
  server.send(204);
}

void setupRoutes()
{
  server.on("/", HTTP_GET, handleRoot);

  server.on("/api/bms", HTTP_GET, handleBmsApi);
  server.on("/api/status", HTTP_GET, handleStatusApi);
  server.on("/api/settings", HTTP_GET, handleSettingsApi);
  server.on("/api/settings/set", HTTP_GET, handleSettingsSetApi);
  server.on("/api/cmd", HTTP_GET, handleCmdApi);
  server.on("/api/alerts/clear", HTTP_GET, handleAlertsClearApi);

  server.on("/api/bms", HTTP_OPTIONS, handleOptions);
  server.on("/api/status", HTTP_OPTIONS, handleOptions);
  server.on("/api/settings", HTTP_OPTIONS, handleOptions);
  server.on("/api/settings/set", HTTP_OPTIONS, handleOptions);
  server.on("/api/cmd", HTTP_OPTIONS, handleOptions);
  server.on("/api/alerts/clear", HTTP_OPTIONS, handleOptions);

  server.onNotFound([]() {
    StaticJsonDocument<256> doc;
    doc["ok"] = false;
    doc["error"] = "not found";
    doc["uri"] = server.uri();

    String out;
    serializeJson(doc, out);

    addCors();
    server.send(404, "application/json", out);
  });
}

// ======================= SETUP =======================
void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32 + MCP2551 BMS Receiver + Web Server + MQTT Start");

  memset(&bms, 0, sizeof(bms));
  initRelayOutputs();
  initLoadRelayOutputs();

  connectWiFiOrStartAP();

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(handleMqttCommand);
  mqttClient.setKeepAlive(30);
  mqttClient.setSocketTimeout(5);
  mqttClient.setBufferSize(6144);

  setupRoutes();
  server.begin();

  Serial.println("HTTP Web Server started");
  Serial.println("AP web: http://192.168.4.1/");
  Serial.println("API: /api/bms");
  Serial.print("MQTT broker: ");
  Serial.print(MQTT_SERVER);
  Serial.print(":");
  Serial.println(MQTT_PORT);
  Serial.print("MQTT topic data: ");
  Serial.println(MQTT_TOPIC_DATA);
  Serial.print("MQTT topic cmd: ");
  Serial.println(MQTT_TOPIC_CMD);

  canReady = initTWAI();
}

// ======================= LOOP =======================
void loop()
{
  // Luôn xử lý web trước để web không bị đơ
  server.handleClient();

  maintainWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    reconnectMqtt();
    mqttClient.loop();
  }

  // Nếu TWAI chưa start được, cứ 3 giây thử lại
  if (!canReady) {
    if (millis() - lastCanRetryMs >= 3000) {
      lastCanRetryMs = millis();
      canReady = initTWAI();
    }
    return;
  }

  readCanFrames();
  checkSoftwareProtection();
  maintainLoadSafety();
  manageLoadRamp(); // [FIX #3] tăng/giảm tải từng nấc 250W một cách an toàn

  publishBmsToMqttIfNeeded();

  if (millis() - lastStatMs >= 1000) {
    lastStatMs = millis();

    uint32_t canAge = lastCanFrameMs > 0 ? millis() - lastCanFrameMs : 999999;

    Serial.print("RX frames/s = ");
    Serial.println(rxCount);

    Serial.print("RX total = ");
    Serial.println(rxTotal);

    Serial.print("CAN age ms = ");
    Serial.println(canAge);

    Serial.print("Current SEQ = ");
    Serial.println(currentSeq);

    Serial.print("gotMask = 0x");
    Serial.println(gotMask, HEX);

    Serial.print("MQTT connected = ");
    Serial.println(mqttClient.connected() ? "YES" : "NO");

    Serial.print("MQTT publish OK/FAIL = ");
    Serial.print(mqttPublishOk);
    Serial.print("/");
    Serial.println(mqttPublishFail);

    if (gotMask == 0) {
      Serial.println("NO BMS CAN DATA YET");
    }
    else if ((gotMask & FULL_MASK) != FULL_MASK) {
      Serial.println("BMS CAN DATA NOT FULL YET");
      printMissingFrames();
    }

    rxCount = 0;
  }
}
