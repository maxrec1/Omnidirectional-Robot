#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <MPU6050_light.h>
#include <Adafruit_SH110X.h>
#include "bitmaps.h"  // Import bitmap images

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// Define Movements
enum Movement {
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  FORWARD_LEFT,
  FORWARD_RIGHT,
  BACKWARD_LEFT,
  BACKWARD_RIGHT,
  TURN_LEFT,
  TURN_RIGHT,
  STOP
};

// MPU6050 object
MPU6050 mpu(Wire);

// Structure to send data
typedef struct struct_message {
  float x;
  float y;
  float z;
} struct_message;
struct_message myData;

// MAC address of the receiver ESP32
uint8_t receiverMACAddress[] = { 0x9C, 0x9C, 0x1F, 0xE9, 0xCE, 0x50 };

unsigned long timer = 0;
uint32_t LoopTimer;

Movement movement;  // Declare movement variable

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ESP-NOW sending result callback
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Function to convert Movement enum to string
const char *movementToString(Movement movement) {
  switch (movement) {
    case FORWARD: return "FORWARD";
    case BACKWARD: return "BACKWARD";
    case LEFT: return "LEFT";
    case RIGHT: return "RIGHT";
    case FORWARD_LEFT: return "FORWARD_LEFT";
    case FORWARD_RIGHT: return "FORWARD_RIGHT";
    case BACKWARD_LEFT: return "BACKWARD_LEFT";
    case BACKWARD_RIGHT: return "BACKWARD_RIGHT";
    case TURN_LEFT: return "TURN_LEFT";
    case TURN_RIGHT: return "TURN_RIGHT";
    case STOP: return "STOP";
    default: return "UNKNOWN";
  }
}

// Function to display arrow in screen
void drawArrow(Movement movement) {
  // Clear the display before drawing a new arrow
  display.clearDisplay();

  switch (movement) {
    case TURN_RIGHT:
      display.drawBitmap(0, 0, ArrowsallArray[1], SCREEN_WIDTH, SCREEN_HEIGHT, SH110X_WHITE);
      break;

    case TURN_LEFT:
      display.drawBitmap(0, 0, ArrowsallArray[0], SCREEN_WIDTH, SCREEN_HEIGHT, SH110X_WHITE);
      break;

    case FORWARD:
      display.drawBitmap(0, 0, ArrowsallArray[7], SCREEN_WIDTH, SCREEN_HEIGHT, SH110X_WHITE);
      break;

    case BACKWARD:
      display.drawBitmap(0, 0, ArrowsallArray[5], SCREEN_WIDTH, SCREEN_HEIGHT, SH110X_WHITE);
      break;

    case LEFT:
      display.drawBitmap(0, 0, ArrowsallArray[9], SCREEN_WIDTH, SCREEN_HEIGHT, SH110X_WHITE);
      break;

    case RIGHT:
      display.drawBitmap(0, 0, ArrowsallArray[8], SCREEN_WIDTH, SCREEN_HEIGHT, SH110X_WHITE);
      break;

    case FORWARD_LEFT:
      display.drawBitmap(0, 0, ArrowsallArray[4], SCREEN_WIDTH, SCREEN_HEIGHT, SH110X_WHITE);
      break;

    case FORWARD_RIGHT:
      display.drawBitmap(0, 0, ArrowsallArray[3], SCREEN_WIDTH, SCREEN_HEIGHT, SH110X_WHITE);
      break;

    case BACKWARD_LEFT:
      display.drawBitmap(0, 0, ArrowsallArray[6], SCREEN_WIDTH, SCREEN_HEIGHT, SH110X_WHITE);
      break;

    case BACKWARD_RIGHT:
      display.drawBitmap(0, 0, ArrowsallArray[2], SCREEN_WIDTH, SCREEN_HEIGHT, SH110X_WHITE);
      break;

    case STOP:
      display.drawBitmap(0, 0, ArrowsallArray[10], SCREEN_WIDTH, SCREEN_HEIGHT, SH110X_WHITE);
      break;

    default:
      Serial.println("Unknown!");
      break;
  }

  // Display the drawn bitmap
  display.display();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize MPU6050 Calibration
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}  // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets();  // gyro and accelero
  Serial.println("Done!");

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  delay(100);  // Give some time for the Wi-Fi stack to initialize

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Initialize OLED display
  display.begin(0x3C, true);  // I2C address for OLED
  display.clearDisplay();

  // Register the send callback
  esp_now_register_send_cb(onDataSent);

  // Initialize and register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMACAddress, 6);
  peerInfo.channel = 0;  // Set to 0 or specify the correct channel if needed
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;  // Explicitly set the interface to station mode

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  } else {
    Serial.println("Peer added successfully");
  }
}
void loop() {
  mpu.update();

  if ((millis() - timer) > 10) {  // send data every 10ms
    myData.x = mpu.getAngleX();
    myData.y = mpu.getAngleY();
    myData.z = mpu.getAngleZ();

    // Send data via ESP-NOW
    esp_err_t result = esp_now_send(receiverMACAddress, (uint8_t *)&myData, sizeof(myData));
    if (result != ESP_OK) {
      Serial.print("Error sending data: ");
      Serial.println(result);
    }

    // Print the myData info in a single line
    Serial.print("X: ");
    Serial.print(myData.x);
    Serial.print(" | Y: ");
    Serial.print(myData.y);
    Serial.print(" | Z: ");
    Serial.println(myData.z);  // This will print the Z value and move to the next line

    // Determine movement based on sensor data
    if (myData.x > 15) {
      movement = (myData.y > 15) ? BACKWARD_LEFT : (myData.y < -15) ? BACKWARD_RIGHT
                                                                    : BACKWARD;
    } else if (myData.x < -15) {
      movement = (myData.y > 15) ? FORWARD_LEFT : (myData.y < -15) ? FORWARD_RIGHT
                                                                   : FORWARD;
    } else if (myData.y > 15) {
      movement = LEFT;
    } else if (myData.y < -15) {
      movement = RIGHT;
    } else if (myData.z > 15) {
      movement = TURN_RIGHT;
    } else if (myData.z < -15) {
      movement = TURN_LEFT;
    } else {
      movement = STOP;
    }

    // Display movement on OLED
    drawArrow(movement);
    // Console log the current movement
    // Serial.print("Current movement: ");
    // Serial.println(movementToString(movement));

    timer = millis();
  }

  while (micros() - LoopTimer < 4000)
    ;
  LoopTimer = micros();
}
