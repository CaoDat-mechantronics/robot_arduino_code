#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>

// ============================================================
// CẤU HÌNH ROBOT
// Đổi 2 dòng này cho từng robot.
// Ví dụ Robot 2:
//   ROBOT_ID   = "robot2";
//   ROBOT_NAME = "Robot 2";
// ============================================================

static const char* ROBOT_ID   = "robot1";
static const char* ROBOT_NAME = "Robot 1";

// Password Wi-Fi setup do ESP32 phát ra.
// Tối thiểu 8 ký tự.
static const char* SETUP_AP_PASSWORD = "robotsetup";

// Thời gian chờ kết nối Wi-Fi.
static const unsigned long WIFI_CONNECT_TIMEOUT_MS = 18000;

// ============================================================
// MQTT / HIVEMQ HEARTBEAT
// Chỉ cần điền đúng cùng HiveMQ Cloud đang dùng ở backend.
// Không thay đổi API REST hiện tại.
// ============================================================

static const char* MQTT_HOST = "YOUR_HIVEMQ_HOST";
static const uint16_t MQTT_PORT = 8883;
static const char* MQTT_USERNAME = "YOUR_HIVEMQ_USERNAME";
static const char* MQTT_PASSWORD = "YOUR_HIVEMQ_PASSWORD";

// ESP32 gửi heartbeat mỗi 10 giây.
static const unsigned long HEARTBEAT_INTERVAL_MS = 10000;

// Nếu MQTT bị rớt, thử nối lại tối đa mỗi 5 giây để không block portal.
static const unsigned long MQTT_RECONNECT_INTERVAL_MS = 5000;

// ============================================================
// IR SENSOR / MOTOR CONFIG
// ============================================================
// QUAN TRỌNG: project hiện tại không cung cấp GPIO thật của cảm biến/motor,
// vì vậy mặc định để -1 để firmware vẫn compile an toàn nhưng KHÔNG đọc sensor
// và KHÔNG chạy motor cho tới khi bạn điền đúng chân phần cứng.
//
// IR2 = cảm biến biên trái
// IR3 = cảm biến biên phải
// IR5 = cảm biến phát hiện món trên robot
static const int IR2_PIN = -1;
static const int IR3_PIN = -1;
static const int IR5_PIN = -1;

// Nếu module IR xuất LOW khi phát hiện vật/vạch thì để true.
static const bool IR2_ACTIVE_LOW = true;
static const bool IR3_ACTIVE_LOW = true;
static const bool IR5_ACTIVE_LOW = true;

// Publish snapshot cảm biến khoảng 20 Hz.
static const unsigned long SENSOR_PUBLISH_INTERVAL_MS = 50;

// Motor watchdog: quá thời gian này không nhận lệnh mới từ frontend -> STOP.
static const unsigned long MOTOR_WATCHDOG_MS = 350;

// Generic H-bridge: mỗi motor dùng IN1, IN2, PWM.
// Điền đúng GPIO theo driver thực tế rồi đổi MOTOR_DRIVER_ENABLED = true.
static const bool MOTOR_DRIVER_ENABLED = false;

static const int LEFT_MOTOR_IN1  = -1;
static const int LEFT_MOTOR_IN2  = -1;
static const int LEFT_MOTOR_PWM  = -1;

static const int RIGHT_MOTOR_IN1 = -1;
static const int RIGHT_MOTOR_IN2 = -1;
static const int RIGHT_MOTOR_PWM = -1;

// Nếu motor quay ngược chiều mong muốn thì đổi true/false cho từng bên.
static const bool LEFT_MOTOR_REVERSED = false;
static const bool RIGHT_MOTOR_REVERSED = false;

// ============================================================
// GLOBAL
// ============================================================

WebServer server(80);
DNSServer dnsServer;
Preferences preferences;

WiFiClientSecure mqttTlsClient;
PubSubClient mqttClient(mqttTlsClient);

String setupApSsid;
unsigned long lastHeartbeatMs = 0;
unsigned long lastMqttReconnectAttemptMs = 0;

// Theo dõi cạnh Wi-Fi để ngay khi STA vừa connected thì MQTT được nối
// và heartbeat đầu tiên được gửi ngay, không chờ chu kỳ 10 giây.
bool lastWifiConnectedForMqtt = false;
bool mqttConfigWarningPrinted = false;

// Realtime sensor state
bool ir2Detected = false;
bool ir3Detected = false;
bool ir5Detected = false;
bool hasFood = false;
unsigned long lastSensorPublishMs = 0;

// Task ESP32 chỉ lưu 3 giá trị theo yêu cầu.
int currentTable = 0;
int currentLine = 0;
int currentStopIndex = 0;

// Motor command từ frontend
int currentLeftMotor = 0;
int currentRightMotor = 0;
unsigned long lastMotorCommandMs = 0;

// ============================================================
// TRANG WEB SETUP
// HTML + CSS + JS nhúng trực tiếp trong file .ino
// ============================================================

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="vi">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Robot Wi-Fi Setup</title>

  <style>
    * {
      box-sizing: border-box;
    }

    :root {
      --bg: #f5f7fb;
      --card: #ffffff;
      --text: #18202a;
      --muted: #6b7280;
      --border: #dfe4ea;
      --primary: #111827;
      --success: #15803d;
      --danger: #b91c1c;
      --radius: 18px;
    }

    body {
      margin: 0;
      font-family: Arial, Helvetica, sans-serif;
      background: var(--bg);
      color: var(--text);
    }

    .page {
      width: min(900px, calc(100% - 28px));
      margin: 0 auto;
      padding: 28px 0 48px;
    }

    .header {
      margin-bottom: 22px;
    }

    .header h1 {
      margin: 0 0 8px;
      font-size: clamp(28px, 5vw, 42px);
    }

    .header p {
      margin: 0;
      color: var(--muted);
      line-height: 1.6;
    }

    .card {
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: var(--radius);
      padding: 20px;
      box-shadow: 0 10px 30px rgba(0,0,0,.04);
    }

    .top-row {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 14px;
      margin-bottom: 18px;
    }

    .robot-name {
      font-size: 22px;
      font-weight: 700;
    }

    .robot-id {
      color: var(--muted);
      font-size: 13px;
      margin-top: 5px;
    }

    .status-pill {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      padding: 7px 10px;
      border-radius: 999px;
      background: #f3f4f6;
      font-size: 13px;
      font-weight: 700;
      white-space: nowrap;
    }

    .dot {
      width: 9px;
      height: 9px;
      border-radius: 50%;
      background: #9ca3af;
    }

    .online .dot {
      background: var(--success);
    }

    .offline .dot {
      background: var(--danger);
    }

    .details {
      display: grid;
      grid-template-columns: repeat(2, minmax(0,1fr));
      gap: 12px;
      margin-bottom: 18px;
    }

    .detail {
      border: 1px solid var(--border);
      border-radius: 13px;
      padding: 13px;
      min-width: 0;
    }

    .detail span {
      display: block;
      color: var(--muted);
      font-size: 12px;
      margin-bottom: 6px;
    }

    .detail strong {
      display: block;
      overflow-wrap: anywhere;
    }

    button {
      width: 100%;
      min-height: 48px;
      border: 0;
      border-radius: 12px;
      padding: 12px 16px;
      font-size: 15px;
      font-weight: 700;
      cursor: pointer;
    }

    button.primary {
      background: var(--primary);
      color: white;
    }

    button.secondary {
      background: #edf0f4;
      color: var(--text);
    }

    button:disabled {
      opacity: .5;
      cursor: not-allowed;
    }

    .wifi-panel {
      margin-top: 18px;
      padding-top: 18px;
      border-top: 1px solid var(--border);
    }

    .section-title {
      margin-bottom: 12px;
      color: var(--muted);
      font-size: 13px;
      font-weight: 700;
      text-transform: uppercase;
      letter-spacing: .07em;
    }

    .wifi-list {
      display: grid;
      gap: 9px;
      margin: 14px 0;
      max-height: 300px;
      overflow-y: auto;
    }

    .wifi-option {
      border: 1px solid var(--border);
      background: #fff;
      color: var(--text);
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
      text-align: left;
    }

    .wifi-option.selected {
      border-color: #111827;
      background: #f3f4f6;
    }

    .wifi-meta {
      color: var(--muted);
      font-size: 12px;
      white-space: nowrap;
    }

    .form-group {
      margin-top: 14px;
    }

    label {
      display: block;
      margin-bottom: 7px;
      font-size: 13px;
      font-weight: 700;
    }

    input {
      width: 100%;
      min-height: 46px;
      padding: 10px 12px;
      border: 1px solid var(--border);
      border-radius: 11px;
      font-size: 16px;
      outline: none;
    }

    input:focus {
      border-color: #111827;
    }

    .message {
      display: none;
      margin-top: 14px;
      padding: 12px 14px;
      border-radius: 11px;
      line-height: 1.5;
    }

    .message.show {
      display: block;
    }

    .message.info {
      color: #1d4ed8;
      background: #eff6ff;
    }

    .message.success {
      color: #166534;
      background: #ecfdf3;
    }

    .message.error {
      color: #991b1b;
      background: #fef2f2;
    }

    .hint {
      color: var(--muted);
      font-size: 13px;
      line-height: 1.55;
      margin-top: 12px;
    }

    @media (max-width: 620px) {
      .page {
        width: min(100% - 18px, 620px);
        padding-top: 14px;
      }

      .card {
        padding: 16px;
        border-radius: 15px;
      }

      .top-row {
        align-items: flex-start;
      }

      .details {
        grid-template-columns: 1fr;
      }
    }
  </style>
</head>

<body>
  <main class="page">
    <header class="header">
      <h1>Cấu hình Robot</h1>
      <p>
        Trang này chạy trực tiếp trên ESP32 và chỉ dùng để
        cấu hình Wi-Fi cho robot.
      </p>
    </header>

    <section class="card">
      <div class="top-row">
        <div>
          <div class="robot-name" id="robotName">Robot</div>
          <div class="robot-id" id="robotId">...</div>
        </div>

        <div class="status-pill offline" id="statusPill">
          <span class="dot"></span>
          <span id="statusText">Đang kiểm tra</span>
        </div>
      </div>

      <div class="details">
        <div class="detail">
          <span>Wi-Fi</span>
          <strong id="wifiSsid">Chưa kết nối</strong>
        </div>

        <div class="detail">
          <span>STA IP</span>
          <strong id="staIp">-</strong>
        </div>

        <div class="detail">
          <span>Setup AP</span>
          <strong id="apSsid">-</strong>
        </div>

        <div class="detail">
          <span>Setup IP</span>
          <strong>192.168.4.1</strong>
        </div>
      </div>

      <button class="primary" id="wifiButton">
        Kết nối Wi-Fi
      </button>

      <div class="wifi-panel" id="wifiPanel" hidden>
        <div class="section-title">Chọn mạng Wi-Fi</div>

        <button class="secondary" id="scanButton">
          Quét Wi-Fi
        </button>

        <div class="wifi-list" id="wifiList"></div>

        <div id="passwordArea" hidden>
          <div class="form-group">
            <label for="selectedSsid">SSID</label>
            <input id="selectedSsid" readonly>
          </div>

          <div class="form-group">
            <label for="wifiPassword">Mật khẩu</label>
            <input
              id="wifiPassword"
              type="password"
              autocomplete="new-password"
              placeholder="Nhập mật khẩu Wi-Fi"
            >
          </div>

          <button
            class="primary"
            id="connectButton"
            style="margin-top:14px"
          >
            Kết nối
          </button>
        </div>

        <div class="message" id="message"></div>
      </div>

      <p class="hint">
        Khi chưa dùng backend, trang này chỉ cấu hình ESP32
        mà điện thoại hoặc máy tính đang kết nối trực tiếp.
      </p>
    </section>
  </main>

  <script>
    const el = (id) => document.getElementById(id);

    let selectedNetwork = null;

    function showMessage(text, type = "info") {
      const box = el("message");
      box.textContent = text;
      box.className = "message show " + type;
    }

    function clearMessage() {
      const box = el("message");
      box.textContent = "";
      box.className = "message";
    }

    async function api(url, options = {}) {
      const response = await fetch(url, options);

      let data = {};

      try {
        data = await response.json();
      } catch (_) {}

      if (!response.ok) {
        const error = new Error(
          data.message || "Request failed"
        );

        error.status = response.status;
        error.data = data;

        throw error;
      }

      return data;
    }

    function renderStatus(data) {
      el("robotName").textContent =
        data.name || "Robot";

      el("robotId").textContent =
        data.id || "";

      el("apSsid").textContent =
        data.ap_ssid || "-";

      const connected =
        Boolean(data.wifi_connected);

      el("wifiSsid").textContent =
        connected
          ? (data.ssid || "Đã kết nối")
          : "Chưa kết nối";

      el("staIp").textContent =
        connected
          ? (data.ip || "-")
          : "-";

      el("wifiButton").textContent =
        connected
          ? "Đổi Wi-Fi"
          : "Kết nối Wi-Fi";

      const pill = el("statusPill");

      pill.classList.remove(
        "online",
        "offline"
      );

      if (connected) {
        pill.classList.add("online");
        el("statusText").textContent =
          "Available";
      } else {
        pill.classList.add("offline");
        el("statusText").textContent =
          "Chưa có Wi-Fi";
      }
    }

    async function refreshStatus() {
      try {
        const data = await api("/api/status");
        renderStatus(data);
      } catch (_) {
        const pill = el("statusPill");

        pill.classList.remove("online");
        pill.classList.add("offline");

        el("statusText").textContent =
          "Không phản hồi";
      }
    }

    function selectNetwork(network, button) {
      selectedNetwork = network;

      document
        .querySelectorAll(".wifi-option")
        .forEach((item) => {
          item.classList.remove("selected");
        });

      button.classList.add("selected");

      el("selectedSsid").value =
        network.ssid;

      el("wifiPassword").value = "";

      el("passwordArea").hidden = false;

      clearMessage();

      setTimeout(() => {
        el("wifiPassword").focus();
      }, 50);
    }

    async function scanWifi() {
      el("scanButton").disabled = true;
      el("scanButton").textContent =
        "Đang quét...";

      el("wifiList").innerHTML = "";
      el("passwordArea").hidden = true;

      selectedNetwork = null;

      showMessage(
        "ESP32 đang quét các mạng Wi-Fi xung quanh...",
        "info"
      );

      try {
        const data =
          await api("/api/wifi/scan");

        const networks =
          Array.isArray(data.networks)
            ? data.networks
            : [];

        el("wifiList").innerHTML = "";

        if (!networks.length) {
          showMessage(
            "Không tìm thấy mạng Wi-Fi nào.",
            "error"
          );
          return;
        }

        clearMessage();

        networks.forEach((network) => {
          const button =
            document.createElement("button");

          button.className =
            "wifi-option";

          const security =
            network.secure
              ? "🔒"
              : "Mở";

          button.innerHTML =
            "<strong></strong>" +
            "<span class='wifi-meta'></span>";

          button
            .querySelector("strong")
            .textContent =
              network.ssid || "(SSID ẩn)";

          button
            .querySelector(".wifi-meta")
            .textContent =
              security +
              " · " +
              network.rssi +
              " dBm";

          button.addEventListener(
            "click",
            () => {
              selectNetwork(
                network,
                button
              );
            }
          );

          el("wifiList")
            .appendChild(button);
        });

      } catch (error) {
        showMessage(
          error.data?.message ||
            "Không thể quét Wi-Fi.",
          "error"
        );

      } finally {
        el("scanButton").disabled = false;
        el("scanButton").textContent =
          "Quét lại Wi-Fi";
      }
    }

    async function connectWifi() {
      if (!selectedNetwork) {
        showMessage(
          "Hãy chọn một Wi-Fi trước.",
          "error"
        );
        return;
      }

      const ssid =
        selectedNetwork.ssid;

      const password =
        el("wifiPassword").value;

      if (
        selectedNetwork.secure &&
        !password
      ) {
        showMessage(
          "Hãy nhập mật khẩu Wi-Fi.",
          "error"
        );
        return;
      }

      const params =
        new URLSearchParams();

      params.set("ssid", ssid);
      params.set("password", password);

      el("connectButton").disabled = true;

      el("connectButton").textContent =
        "Đang kết nối...";

      showMessage(
        "ESP32 đang thử kết nối tới " +
          ssid +
          "...",
        "info"
      );

      try {
        await api(
          "/api/wifi/connect",
          {
            method: "POST",

            headers: {
              "Content-Type":
                "application/x-www-form-urlencoded;charset=UTF-8"
            },

            body: params.toString()
          }
        );

        showMessage(
          "Kết nối Wi-Fi thành công.",
          "success"
        );

        await refreshStatus();

      } catch (error) {
        const code =
          error.data?.code || "";

        if (
          code ===
          "WIFI_AUTH_FAILED"
        ) {
          showMessage(
            "Mật khẩu Wi-Fi không đúng hoặc xác thực thất bại.",
            "error"
          );
        } else {
          showMessage(
            error.data?.message ||
              "Không thể kết nối Wi-Fi.",
            "error"
          );
        }

      } finally {
        el("connectButton").disabled =
          false;

        el("connectButton").textContent =
          "Kết nối";
      }
    }

    el("wifiButton")
      .addEventListener(
        "click",
        async () => {
          el("wifiPanel").hidden = false;
          clearMessage();
          await scanWifi();
        }
      );

    el("scanButton")
      .addEventListener(
        "click",
        scanWifi
      );

    el("connectButton")
      .addEventListener(
        "click",
        connectWifi
      );

    refreshStatus();

    setInterval(
      refreshStatus,
      2500
    );
  </script>
</body>
</html>
)HTML";

// ============================================================
// JSON HELPER
// ============================================================

String jsonEscape(const String& value) {
  String output;

  output.reserve(
    value.length() + 8
  );

  for (
    size_t i = 0;
    i < value.length();
    i++
  ) {
    char c = value[i];

    switch (c) {
      case '\\':
        output += "\\\\";
        break;

      case '"':
        output += "\\\"";
        break;

      case '\n':
        output += "\\n";
        break;

      case '\r':
        output += "\\r";
        break;

      case '\t':
        output += "\\t";
        break;

      default:
        output += c;
        break;
    }
  }

  return output;
}

void sendJson(
  int statusCode,
  const String& json
) {
  server.sendHeader(
    "Cache-Control",
    "no-store"
  );

  server.send(
    statusCode,
    "application/json; charset=utf-8",
    json
  );
}

// ============================================================
// MQTT HEARTBEAT
// ============================================================

int robotNumber() {
  String id = String(ROBOT_ID);
  id.toLowerCase();

  if (
    id == "robot2" ||
    id == "robot_2"
  ) {
    return 2;
  }

  return 1;
}

String robotDatabaseId() {
  return String("robot_") + String(robotNumber());
}

String robotStatusTopic() {
  return String("topic") + String(robotNumber()) + "/status";
}

String robotSensorsTopic() {
  return String("topic") + String(robotNumber()) + "/sensors";
}

String robotTaskTopic() {
  return String("topic") + String(robotNumber()) + "/task";
}

String robotMotorTopic() {
  return String("topic") + String(robotNumber()) + "/motor";
}

bool readIrDetected(int pin, bool activeLow) {
  if (pin < 0) {
    return false;
  }

  int raw = digitalRead(pin);
  return activeLow ? (raw == LOW) : (raw == HIGH);
}

void setupRealtimeHardware() {
  if (IR2_PIN >= 0) pinMode(IR2_PIN, INPUT);
  if (IR3_PIN >= 0) pinMode(IR3_PIN, INPUT);
  if (IR5_PIN >= 0) pinMode(IR5_PIN, INPUT);

  if (
    MOTOR_DRIVER_ENABLED &&
    LEFT_MOTOR_IN1 >= 0 && LEFT_MOTOR_IN2 >= 0 && LEFT_MOTOR_PWM >= 0 &&
    RIGHT_MOTOR_IN1 >= 0 && RIGHT_MOTOR_IN2 >= 0 && RIGHT_MOTOR_PWM >= 0
  ) {
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(LEFT_MOTOR_PWM, OUTPUT);

    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM, OUTPUT);

    Serial.println("[MOTOR] Generic H-bridge enabled");
  } else {
    Serial.println("[MOTOR] Disabled - hay dien GPIO va MOTOR_DRIVER_ENABLED=true");
  }

  if (IR2_PIN < 0 || IR3_PIN < 0 || IR5_PIN < 0) {
    Serial.println("[SENSOR] IR GPIO chua duoc cau hinh; mac dinh se doc false");
  }
}

void applyOneMotor(
  int in1,
  int in2,
  int pwmPin,
  int speedValue,
  bool reversed
) {
  if (in1 < 0 || in2 < 0 || pwmPin < 0) {
    return;
  }

  int speed = constrain(speedValue, -255, 255);
  if (reversed) speed = -speed;

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, speed);
  }
  else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, -speed);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  }
}

void applyMotorSpeeds(int left, int right) {
  currentLeftMotor = constrain(left, -255, 255);
  currentRightMotor = constrain(right, -255, 255);

  if (!MOTOR_DRIVER_ENABLED) {
    return;
  }

  applyOneMotor(
    LEFT_MOTOR_IN1,
    LEFT_MOTOR_IN2,
    LEFT_MOTOR_PWM,
    currentLeftMotor,
    LEFT_MOTOR_REVERSED
  );

  applyOneMotor(
    RIGHT_MOTOR_IN1,
    RIGHT_MOTOR_IN2,
    RIGHT_MOTOR_PWM,
    currentRightMotor,
    RIGHT_MOTOR_REVERSED
  );
}

void stopMotors() {
  applyMotorSpeeds(0, 0);
}

void saveCurrentTask() {
  preferences.begin("robot-task", false);
  preferences.putInt("table", currentTable);
  preferences.putInt("line", currentLine);
  preferences.putInt("stop", currentStopIndex);
  preferences.end();
}

void loadCurrentTask() {
  preferences.begin("robot-task", true);
  currentTable = preferences.getInt("table", 0);
  currentLine = preferences.getInt("line", 0);
  currentStopIndex = preferences.getInt("stop", 0);
  preferences.end();

  Serial.printf(
    "[TASK] Loaded table=%d line=%d stop_index=%d\n",
    currentTable,
    currentLine,
    currentStopIndex
  );
}

bool publishSensors() {
  if (!mqttClient.connected()) {
    return false;
  }

  String payload = "{";
  payload += "\"type\":\"sensors\",";
  payload += "\"robot\":" + String(robotNumber()) + ",";
  payload += "\"ir2\":" + String(ir2Detected ? "true" : "false") + ",";
  payload += "\"ir3\":" + String(ir3Detected ? "true" : "false") + ",";
  payload += "\"ir5\":" + String(ir5Detected ? "true" : "false") + ",";
  payload += "\"has_food\":" + String(hasFood ? "true" : "false") + ",";
  payload += "\"uptime_ms\":" + String(millis());
  payload += "}";

  return mqttClient.publish(
    robotSensorsTopic().c_str(),
    payload.c_str(),
    false
  );
}

void publishTaskAck(const String& commandId) {
  if (!mqttClient.connected()) {
    return;
  }

  String payload = "{";
  payload += "\"type\":\"command_ack\",";
  payload += "\"status\":\"command_received\",";
  payload += "\"robot\":" + String(robotNumber()) + ",";
  payload += "\"command_id\":\"" + jsonEscape(commandId) + "\",";
  payload += "\"table\":" + String(currentTable) + ",";
  payload += "\"line\":" + String(currentLine) + ",";
  payload += "\"stop_index\":" + String(currentStopIndex);
  payload += "}";

  mqttClient.publish(
    robotStatusTopic().c_str(),
    payload.c_str(),
    false
  );

  Serial.println("[TASK] ACK -> " + payload);
}

void mqttCallback(char* topicChars, byte* payloadBytes, unsigned int length) {
  String topic = String(topicChars);
  String text;
  text.reserve(length + 1);

  for (unsigned int i = 0; i < length; i++) {
    text += (char)payloadBytes[i];
  }

  if (topic == robotTaskTopic()) {
    StaticJsonDocument<384> doc;
    DeserializationError error = deserializeJson(doc, text);

    if (error) {
      Serial.println("[TASK] JSON error");
      return;
    }

    currentTable = doc["table"] | 0;
    currentLine = doc["line"] | 0;
    currentStopIndex = doc["stop_index"] | 0;
    String commandId = String((const char*)(doc["command_id"] | ""));

    saveCurrentTask();

    Serial.printf(
      "[TASK] Saved table=%d line=%d stop_index=%d\n",
      currentTable,
      currentLine,
      currentStopIndex
    );

    publishTaskAck(commandId);
    return;
  }

  if (topic == robotMotorTopic()) {
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, text);

    if (error) {
      return;
    }

    int left = doc["left"] | 0;
    int right = doc["right"] | 0;

    applyMotorSpeeds(left, right);
    lastMotorCommandMs = millis();
    return;
  }
}

void subscribeRealtimeTopics() {
  if (!mqttClient.connected()) {
    return;
  }

  mqttClient.subscribe(robotTaskTopic().c_str(), 1);
  mqttClient.subscribe(robotMotorTopic().c_str(), 0);

  Serial.println("[MQTT] SUB " + robotTaskTopic());
  Serial.println("[MQTT] SUB " + robotMotorTopic());
}

bool mqttConfigurationReady() {
  String host = String(MQTT_HOST);
  String user = String(MQTT_USERNAME);
  String pass = String(MQTT_PASSWORD);

  return
    host.length() > 0 &&
    user.length() > 0 &&
    pass.length() > 0 &&
    host != "YOUR_HIVEMQ_HOST" &&
    user != "YOUR_HIVEMQ_USERNAME" &&
    pass != "YOUR_HIVEMQ_PASSWORD";
}

void setupMqttClient() {
  // Prototype: dùng TLS nhưng bỏ verify certificate để không phải nhúng CA.
  // Khi production có thể thay bằng mqttTlsClient.setCACert(...).
  mqttTlsClient.setInsecure();

  mqttClient.setServer(
    MQTT_HOST,
    MQTT_PORT
  );

  mqttClient.setSocketTimeout(2);
  mqttClient.setBufferSize(1024);
  mqttClient.setCallback(mqttCallback);

  Serial.print("[MQTT] Broker: ");
  Serial.print(MQTT_HOST);
  Serial.print(":");
  Serial.println(MQTT_PORT);

  Serial.print("[MQTT] Status topic: ");
  Serial.println(robotStatusTopic());
  Serial.print("[MQTT] Sensors topic: ");
  Serial.println(robotSensorsTopic());
  Serial.print("[MQTT] Task topic: ");
  Serial.println(robotTaskTopic());
  Serial.print("[MQTT] Motor topic: ");
  Serial.println(robotMotorTopic());

  if (!mqttConfigurationReady()) {
    Serial.println(
      "[MQTT] ERROR: Chua cau hinh MQTT_HOST / MQTT_USERNAME / MQTT_PASSWORD"
    );
    Serial.println(
      "[MQTT] ESP32 khong doc duoc Environment Variables cua backend Render."
    );
    Serial.println(
      "[MQTT] Hay dien thong tin HiveMQ truc tiep vao firmware ESP32."
    );
    mqttConfigWarningPrinted = true;
  } else {
    Serial.println("[MQTT] Configuration ready");
  }
}

bool connectMqtt() {
  if (!mqttConfigurationReady()) {
    return false;
  }

  if (
    WiFi.status() !=
    WL_CONNECTED
  ) {
    return false;
  }

  if (mqttClient.connected()) {
    return true;
  }

  uint64_t chipId = ESP.getEfuseMac();

  String clientId =
    String("restaurant-") +
    String(ROBOT_ID) +
    "-" +
    String(
      (uint32_t)(chipId & 0xFFFFFFFF),
      HEX
    );

  Serial.print(
    "[MQTT] Connecting as "
  );
  Serial.println(clientId);

  bool connected =
    mqttClient.connect(
      clientId.c_str(),
      MQTT_USERNAME,
      MQTT_PASSWORD
    );

  if (connected) {
    Serial.println(
      "[MQTT] Connected"
    );

    subscribeRealtimeTopics();

    // Cho phép gửi heartbeat/sensor ngay sau khi reconnect.
    lastHeartbeatMs =
      millis() -
      HEARTBEAT_INTERVAL_MS;
    lastSensorPublishMs = 0;

    return true;
  }

  Serial.print(
    "[MQTT] Connect failed, state="
  );
  Serial.println(
    mqttClient.state()
  );

  return false;
}

bool publishHeartbeat() {
  if (
    WiFi.status() !=
    WL_CONNECTED ||
    !mqttClient.connected()
  ) {
    return false;
  }

  String payload = "{";

  payload +=
    "\"type\":\"heartbeat\",";

  payload +=
    "\"status\":\"heartbeat\",";

  payload +=
    "\"robot\":" +
    String(robotNumber()) +
    ",";

  payload +=
    "\"robot_id\":\"" +
    jsonEscape(
      robotDatabaseId()
    ) +
    "\",";

  payload +=
    "\"wifi_connected\":true,";

  payload +=
    "\"has_food\":" +
    String(hasFood ? "true" : "false") +
    ",";

  payload +=
    "\"ip\":\"" +
    WiFi
      .localIP()
      .toString() +
    "\",";

  payload +=
    "\"rssi\":" +
    String(
      WiFi.RSSI()
    ) +
    ",";

  payload +=
    "\"uptime_ms\":" +
    String(
      millis()
    );

  payload += "}";

  String topic =
    robotStatusTopic();

  bool ok =
    mqttClient.publish(
      topic.c_str(),
      payload.c_str(),
      false
    );

  if (ok) {
    Serial.print(
      "[HEARTBEAT] -> "
    );
    Serial.print(topic);
    Serial.print(" ");
    Serial.println(payload);
  } else {
    Serial.println(
      "[HEARTBEAT] Publish failed"
    );
  }

  return ok;
}

void serviceMqttHeartbeat() {
  bool wifiConnected =
    WiFi.status() == WL_CONNECTED;

  // ----------------------------------------------------------
  // Wi-Fi chưa có / vừa bị mất
  // ----------------------------------------------------------
  if (!wifiConnected) {
    if (lastWifiConnectedForMqtt) {
      Serial.println(
        "[HEARTBEAT] Wi-Fi lost -> stop heartbeat"
      );
    }

    lastWifiConnectedForMqtt = false;
    stopMotors();

    if (mqttClient.connected()) {
      mqttClient.disconnect();
    }

    return;
  }

  // ----------------------------------------------------------
  // MQTT chưa được cấu hình trong firmware
  // ----------------------------------------------------------
  if (!mqttConfigurationReady()) {
    if (!mqttConfigWarningPrinted) {
      Serial.println(
        "[MQTT] ERROR: MQTT config missing, cannot send heartbeat"
      );
      mqttConfigWarningPrinted = true;
    }
    return;
  }

  mqttConfigWarningPrinted = false;

  // ----------------------------------------------------------
  // CẠNH KẾT NỐI WI-FI:
  // Ngay lần loop đầu tiên sau khi Wi-Fi connected, nối MQTT và
  // gửi heartbeat NGAY LẬP TỨC. Không chờ 5s reconnect hoặc 10s.
  // ----------------------------------------------------------
  if (!lastWifiConnectedForMqtt) {
    lastWifiConnectedForMqtt = true;

    Serial.print(
      "[HEARTBEAT] Wi-Fi connected, IP="
    );
    Serial.println(
      WiFi.localIP()
    );

    if (connectMqtt()) {
      mqttClient.loop();

      if (publishHeartbeat()) {
        lastHeartbeatMs = millis();
        Serial.println(
          "[HEARTBEAT] Initial connected heartbeat sent"
        );
      }
    } else {
      // Nếu lần đầu chưa vào được broker thì cho phép retry ngay
      // ở các vòng sau theo chu kỳ MQTT_RECONNECT_INTERVAL_MS.
      lastMqttReconnectAttemptMs = millis();
    }

    return;
  }

  // ----------------------------------------------------------
  // Wi-Fi vẫn còn nhưng MQTT rớt: thử nối lại định kỳ.
  // Sau khi reconnect thành công cũng gửi heartbeat ngay.
  // ----------------------------------------------------------
  if (!mqttClient.connected()) {
    unsigned long now = millis();

    if (
      now - lastMqttReconnectAttemptMs >=
      MQTT_RECONNECT_INTERVAL_MS
    ) {
      lastMqttReconnectAttemptMs = now;

      if (connectMqtt()) {
        mqttClient.loop();

        if (publishHeartbeat()) {
          lastHeartbeatMs = millis();
          Serial.println(
            "[HEARTBEAT] Reconnect heartbeat sent"
          );
        }
      }
    }

    return;
  }

  mqttClient.loop();

  // ----------------------------------------------------------
  // Heartbeat định kỳ mỗi 10 giây sau bản tin đầu tiên.
  // ----------------------------------------------------------
  unsigned long now = millis();

  if (
    now - lastHeartbeatMs >=
    HEARTBEAT_INTERVAL_MS
  ) {
    if (publishHeartbeat()) {
      lastHeartbeatMs = now;
    }
  }
}

void serviceRealtimeSensorsAndMotor() {
  bool nextIr2 = readIrDetected(IR2_PIN, IR2_ACTIVE_LOW);
  bool nextIr3 = readIrDetected(IR3_PIN, IR3_ACTIVE_LOW);
  bool nextIr5 = readIrDetected(IR5_PIN, IR5_ACTIVE_LOW);

  bool changed =
    nextIr2 != ir2Detected ||
    nextIr3 != ir3Detected ||
    nextIr5 != ir5Detected;

  ir2Detected = nextIr2;
  ir3Detected = nextIr3;
  ir5Detected = nextIr5;
  hasFood = ir5Detected;

  unsigned long now = millis();

  if (
    mqttClient.connected() &&
    (changed || now - lastSensorPublishMs >= SENSOR_PUBLISH_INTERVAL_MS)
  ) {
    if (publishSensors()) {
      lastSensorPublishMs = now;
    }
  }

  if (
    (currentLeftMotor != 0 || currentRightMotor != 0) &&
    now - lastMotorCommandMs > MOTOR_WATCHDOG_MS
  ) {
    stopMotors();
    Serial.println("[MOTOR] Watchdog STOP - mat lenh frontend");
  }
}


// ============================================================
// NVS / PREFERENCES
// ============================================================

void saveWifiCredentials(
  const String& ssid,
  const String& password
) {
  preferences.begin(
    "robot-wifi",
    false
  );

  preferences.putString(
    "ssid",
    ssid
  );

  preferences.putString(
    "password",
    password
  );

  preferences.end();
}

bool loadWifiCredentials(
  String& ssid,
  String& password
) {
  preferences.begin(
    "robot-wifi",
    true
  );

  ssid =
    preferences.getString(
      "ssid",
      ""
    );

  password =
    preferences.getString(
      "password",
      ""
    );

  preferences.end();

  return ssid.length() > 0;
}

// ============================================================
// WIFI
// ============================================================

bool connectToWifi(
  const String& ssid,
  const String& password,
  unsigned long timeoutMs
) {
  if (ssid.isEmpty()) {
    return false;
  }

  // AP + STA đồng thời.
  WiFi.mode(WIFI_AP_STA);

  // Ngắt STA cũ nhưng KHÔNG tắt SoftAP.
  WiFi.disconnect(
    false,
    false
  );

  delay(150);

  Serial.print(
    "[WIFI] Connecting to: "
  );

  Serial.println(ssid);

  if (password.length() == 0) {
    WiFi.begin(
      ssid.c_str()
    );
  } else {
    WiFi.begin(
      ssid.c_str(),
      password.c_str()
    );
  }

  unsigned long start =
    millis();

  while (
    WiFi.status() != WL_CONNECTED &&
    millis() - start < timeoutMs
  ) {
    delay(250);
    yield();
  }

  if (
    WiFi.status() ==
    WL_CONNECTED
  ) {
    Serial.println(
      "[WIFI] Connected"
    );

    Serial.print(
      "[WIFI] STA IP: "
    );

    Serial.println(
      WiFi.localIP()
    );

    return true;
  }

  Serial.print(
    "[WIFI] Failed. status="
  );

  Serial.println(
    (int)WiFi.status()
  );

  return false;
}

// ============================================================
// SOFT AP
// ============================================================

void startSetupAccessPoint() {
  setupApSsid =
    String(ROBOT_ID) +
    "-setup";

  WiFi.mode(
    WIFI_AP_STA
  );

  bool ok =
    WiFi.softAP(
      setupApSsid.c_str(),
      SETUP_AP_PASSWORD,
      1,
      false,
      6
    );

  if (!ok) {
    Serial.println(
      "[AP] Failed to start"
    );

    return;
  }

  Serial.print(
    "[AP] SSID: "
  );

  Serial.println(
    setupApSsid
  );

  Serial.print(
    "[AP] IP: "
  );

  Serial.println(
    WiFi.softAPIP()
  );

  // Captive portal DNS.
  dnsServer.start(
    53,
    "*",
    WiFi.softAPIP()
  );
}

// ============================================================
// HTTP
// ============================================================

void handleRoot() {
  server.sendHeader(
    "Cache-Control",
    "no-store"
  );

  server.send_P(
    200,
    "text/html; charset=utf-8",
    INDEX_HTML
  );
}

void handleStatus() {
  bool connected =
    WiFi.status() ==
    WL_CONNECTED;

  String json = "{";

  json +=
    "\"id\":\"" +
    jsonEscape(
      String(ROBOT_ID)
    ) +
    "\",";

  json +=
    "\"name\":\"" +
    jsonEscape(
      String(ROBOT_NAME)
    ) +
    "\",";

  json +=
    "\"ap_ssid\":\"" +
    jsonEscape(
      setupApSsid
    ) +
    "\",";

  json +=
    "\"wifi_connected\":";

  json +=
    connected
      ? "true"
      : "false";

  json += ",";

  json +=
    "\"ssid\":\"";

  json +=
    connected
      ? jsonEscape(
          WiFi.SSID()
        )
      : "";

  json += "\",";

  json +=
    "\"ip\":\"";

  json +=
    connected
      ? WiFi
          .localIP()
          .toString()
      : "";

  json += "\",";

  json +=
    "\"rssi\":";

  json +=
    connected
      ? String(
          WiFi.RSSI()
        )
      : "0";

  json += "}";

  sendJson(
    200,
    json
  );
}

void handleWifiScan() {
  Serial.println(
    "[WIFI] Scan requested"
  );

  int count =
    WiFi.scanNetworks(
      false,
      true
    );

  if (count < 0) {
    sendJson(
      500,
      "{\"success\":false,"
      "\"message\":\"Wi-Fi scan failed\"}"
    );

    return;
  }

  struct WifiEntry {
    String ssid;
    int32_t rssi;
    bool secure;
  };

  WifiEntry entries[30];

  int entryCount = 0;

  int limit =
    min(
      count,
      30
    );

  for (
    int i = 0;
    i < limit;
    i++
  ) {
    String ssid =
      WiFi.SSID(i);

    if (
      ssid.length() == 0
    ) {
      continue;
    }

    int existing = -1;

    for (
      int j = 0;
      j < entryCount;
      j++
    ) {
      if (
        entries[j].ssid ==
        ssid
      ) {
        existing = j;
        break;
      }
    }

    int32_t rssi =
      WiFi.RSSI(i);

    bool secure =
      WiFi.encryptionType(i)
      != WIFI_AUTH_OPEN;

    if (
      existing >= 0
    ) {
      if (
        rssi >
        entries[existing].rssi
      ) {
        entries[existing].rssi =
          rssi;

        entries[existing].secure =
          secure;
      }
    } else if (
      entryCount < 30
    ) {
      entries[entryCount].ssid =
        ssid;

      entries[entryCount].rssi =
        rssi;

      entries[entryCount].secure =
        secure;

      entryCount++;
    }
  }

  // Sort RSSI mạnh -> yếu.
  for (
    int i = 0;
    i < entryCount - 1;
    i++
  ) {
    for (
      int j = i + 1;
      j < entryCount;
      j++
    ) {
      if (
        entries[j].rssi >
        entries[i].rssi
      ) {
        WifiEntry temp =
          entries[i];

        entries[i] =
          entries[j];

        entries[j] =
          temp;
      }
    }
  }

  String json =
    "{\"success\":true,"
    "\"networks\":[";

  for (
    int i = 0;
    i < entryCount;
    i++
  ) {
    if (i > 0) {
      json += ",";
    }

    json += "{";

    json +=
      "\"ssid\":\"" +
      jsonEscape(
        entries[i].ssid
      ) +
      "\",";

    json +=
      "\"rssi\":" +
      String(
        entries[i].rssi
      ) +
      ",";

    json +=
      "\"secure\":";

    json +=
      entries[i].secure
        ? "true"
        : "false";

    json += "}";
  }

  json += "]}";

  WiFi.scanDelete();

  sendJson(
    200,
    json
  );
}

void handleWifiConnect() {
  if (
    !server.hasArg("ssid")
  ) {
    sendJson(
      400,
      "{\"success\":false,"
      "\"code\":\"SSID_REQUIRED\","
      "\"message\":\"SSID is required\"}"
    );

    return;
  }

  String ssid =
    server.arg("ssid");

  String password =
    server.hasArg("password")
      ? server.arg("password")
      : "";

  bool connected =
    connectToWifi(
      ssid,
      password,
      WIFI_CONNECT_TIMEOUT_MS
    );

  if (!connected) {
    wl_status_t status =
      WiFi.status();

    if (
      status ==
      WL_CONNECT_FAILED
    ) {
      sendJson(
        401,
        "{\"success\":false,"
        "\"code\":\"WIFI_AUTH_FAILED\","
        "\"message\":\"Wi-Fi authentication failed\"}"
      );
    } else {
      sendJson(
        408,
        "{\"success\":false,"
        "\"code\":\"WIFI_CONNECT_FAILED\","
        "\"message\":\"Unable to connect to selected Wi-Fi\"}"
      );
    }

    return;
  }

  saveWifiCredentials(
    ssid,
    password
  );

  String json = "{";

  json +=
    "\"success\":true,";

  json +=
    "\"message\":\"Wi-Fi connected\",";

  json +=
    "\"ssid\":\"" +
    jsonEscape(
      WiFi.SSID()
    ) +
    "\",";

  json +=
    "\"ip\":\"" +
    WiFi
      .localIP()
      .toString() +
    "\"";

  json += "}";

  sendJson(
    200,
    json
  );
}

void handleCaptivePortal() {
  handleRoot();
}

void setupHttpServer() {
  server.on(
    "/",
    HTTP_GET,
    handleRoot
  );

  server.on(
    "/index.html",
    HTTP_GET,
    handleRoot
  );

  server.on(
    "/api/status",
    HTTP_GET,
    handleStatus
  );

  server.on(
    "/api/device",
    HTTP_GET,
    handleStatus
  );

  server.on(
    "/api/wifi/scan",
    HTTP_GET,
    handleWifiScan
  );

  server.on(
    "/api/wifi/connect",
    HTTP_POST,
    handleWifiConnect
  );

  // Captive portal probes.
  server.on(
    "/generate_204",
    HTTP_GET,
    handleCaptivePortal
  );

  server.on(
    "/gen_204",
    HTTP_GET,
    handleCaptivePortal
  );

  server.on(
    "/hotspot-detect.html",
    HTTP_GET,
    handleCaptivePortal
  );

  server.on(
    "/ncsi.txt",
    HTTP_GET,
    handleCaptivePortal
  );

  server.on(
    "/connecttest.txt",
    HTTP_GET,
    handleCaptivePortal
  );

  server.onNotFound(
    handleCaptivePortal
  );

  server.begin();

  Serial.println(
    "[HTTP] Server started"
  );
}

// ============================================================
// WIFI ĐÃ LƯU
// ============================================================

void trySavedWifi() {
  String savedSsid;
  String savedPassword;

  if (
    !loadWifiCredentials(
      savedSsid,
      savedPassword
    )
  ) {
    Serial.println(
      "[WIFI] No saved Wi-Fi"
    );

    return;
  }

  Serial.print(
    "[WIFI] Saved SSID: "
  );

  Serial.println(
    savedSsid
  );

  connectToWifi(
    savedSsid,
    savedPassword,
    10000
  );
}

// ============================================================
// ARDUINO
// ============================================================

void setup() {
  Serial.begin(
    115200
  );

  delay(400);

  Serial.println();
  Serial.println(
    "=============================="
  );

  Serial.print(
    "Robot ID: "
  );

  Serial.println(
    ROBOT_ID
  );

  Serial.print(
    "Robot Name: "
  );

  Serial.println(
    ROBOT_NAME
  );

  Serial.println(
    "=============================="
  );

  setupRealtimeHardware();
  loadCurrentTask();
  stopMotors();

  // SoftAP luôn bật để có thể cấu hình/đổi Wi-Fi.
  startSetupAccessPoint();

  // Thử kết nối Wi-Fi đã lưu.
  trySavedWifi();

  // Bật web server.
  setupHttpServer();

  // MQTT: heartbeat + sensors + task + motor realtime.
  setupMqttClient();

  Serial.println();
  Serial.println(
    "[READY]"
  );

  Serial.print(
    "Setup SSID: "
  );

  Serial.println(
    setupApSsid
  );

  Serial.println(
    "Setup URL : http://192.168.4.1"
  );

  if (
    WiFi.status() ==
    WL_CONNECTED
  ) {
    Serial.print(
      "STA IP    : "
    );

    Serial.println(
      WiFi.localIP()
    );
  }
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();

  // Heartbeat + MQTT realtime không làm thay đổi code dò Wi-Fi/captive portal.
  serviceMqttHeartbeat();
  serviceRealtimeSensorsAndMotor();

  delay(2);
}