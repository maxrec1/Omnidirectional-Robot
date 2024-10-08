#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <EEPROM.h>

#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#include "camera_pins.h"

const char* default_ssid = "myNext Hotels Free Wi-Fi";
const char* default_password = "12345678";

WebServer server(80)custom_ssid;
DNSServer dnsServer;
const byte DNS_PORT = 53;
String st;
String pwd;

void startCameraServer();
void setupLedFlash(int pin);
void writeEEPROMString(int addr, String data) {
  for (int i = 0; i < data.length(); i++) {
    EEPROM.write(addr + i, data[i]);
  }
  EEPROM.write(addr + data.length(), '\0'); // Null terminator
  EEPROM.commit();
}

String readEEPROMString(int addr, int len) {
  char data[len + 1];
  for (int i = 0; i < len; i++) {
    data[i] = EEPROM.read(addr + i);
  }
  data[len] = '\0';
  return String(data);
}

void handleRoot() {
  String page = "<html><body><h1>ESP32 Camera Setup</h1>";

  // Check if connected to Wi-Fi and show IP address
  if (WiFi.status() == WL_CONNECTED) {
    page += "<p>Connected to Wi-Fi: " + String(WiFi.SSID()) + "</p>";
    page += "<p>IP Address: " + WiFi.localIP().toString() + "</p>";
  } else {
    page += "<p>Not connected to Wi-Fi</p>";
  }

  page += "<form action=\"/scan\"><input type=\"submit\" value=\"Scan for Networks\"></form>";
  page += "</body></html>";

  server.send(200, "text/html", page);
}

void handleScan() {
  String page = "<html><body><h1>Select a network</h1>";
  page += "<form action=\"/set\"><label for=\"ssid\">SSID:</label>";
  page += "<select name=\"ssid\">";
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; ++i) {
    page += "<option value=\"" + WiFi.SSID(i) + "\">" + WiFi.SSID(i) + "</option>";
  }
  page += "</select><br>";
  page += "<label for=\"password\">Password:</label>";
  page += "<input type=\"text\" name=\"password\"><br>";
  page += "<input type=\"submit\" value=\"Set Wi-Fi\"></form>";
  page += "</body></html>";
  server.send(200, "text/html", page);
}

void handleSet() {
  st = server.arg("ssid");
  pwd = server.arg("password");

  writeEEPROMString(0, st);
  writeEEPROMString(100, pwd);

  server.send(200, "text/html", "<html><body><h1>Wi-Fi credentials saved! Rebooting...</h1></body></html>");

  delay(3000);
  ESP.restart();
}

void startAccessPoint() {
  WiFi.softAP("ESP32-Camera", "12345678");
  Serial.println("Starting Access Point...");
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  server.on("/", handleRoot);
  server.on("/scan", handleScan);
  server.on("/set", handleSet);
  server.begin();
  Serial.println("Server started, connect to AP to configure Wi-Fi.");
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  EEPROM.begin(512); // Initialize EEPROM

  // Load saved Wi-Fi credentials
  st = readEEPROMString(0, 100);
  pwd = readEEPROMString(100, 100);

  if (st.length() > 0 && pwd.length() > 0) {
    WiFi.begin(st.c_str(), pwd.c_str());
    Serial.println("Connecting to saved Wi-Fi...");
    
    unsigned long startAttemptTime = millis();

    // Attempt to connect to Wi-Fi for 10 seconds
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("");
      Serial.println("WiFi connected");

      // Camera initialization
      camera_config_t config;
      config.ledc_channel = LEDC_CHANNEL_0;
      config.ledc_timer = LEDC_TIMER_0;
      config.pin_d0 = Y2_GPIO_NUM;
      config.pin_d1 = Y3_GPIO_NUM;
      config.pin_d2 = Y4_GPIO_NUM;
      config.pin_d3 = Y5_GPIO_NUM;
      config.pin_d4 = Y6_GPIO_NUM;
      config.pin_d5 = Y7_GPIO_NUM;
      config.pin_d6 = Y8_GPIO_NUM;
      config.pin_d7 = Y9_GPIO_NUM;
      config.pin_xclk = XCLK_GPIO_NUM;
      config.pin_pclk = PCLK_GPIO_NUM;
      config.pin_vsync = VSYNC_GPIO_NUM;
      config.pin_href = HREF_GPIO_NUM;
      config.pin_sccb_sda = SIOD_GPIO_NUM;
      config.pin_sccb_scl = SIOC_GPIO_NUM;
      config.pin_pwdn = PWDN_GPIO_NUM;
      config.pin_reset = RESET_GPIO_NUM;
      config.xclk_freq_hz = 20000000;
      config.frame_size = FRAMESIZE_UXGA;
      config.pixel_format = PIXFORMAT_JPEG;
      config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
      config.fb_location = CAMERA_FB_IN_PSRAM;
      config.jpeg_quality = 12;
      config.fb_count = 1;

      if (psramFound()) {
        config.jpeg_quality = 10;
        config.fb_count = 2;
        config.grab_mode = CAMERA_GRAB_LATEST;
      } else {
        config.frame_size = FRAMESIZE_SVGA;
        config.fb_location = CAMERA_FB_IN_DRAM;
      }

      esp_err_t err = esp_camera_init(&config);
      if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
      }

      sensor_t *s = esp_camera_sensor_get();
      if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, -2);
      }
      s->set_framesize(s, FRAMESIZE_QVGA);

      startCameraServer();

      Serial.print("Camera Ready! Use 'http://");
      Serial.print(WiFi.localIP());
      Serial.println("' to connect");
    } else {
      Serial.println("Failed to connect to Wi-Fi, starting AP mode");
      startAccessPoint();
    }
  } else {
    Serial.println("No saved Wi-Fi credentials, starting AP mode");
    startAccessPoint();
  }
}

void loop() {
  
  if (WiFi.status() != WL_CONNECTED) {
    dnsServer.processNextRequest();
    server.handleClient();
  }
   delay(10000);
}
