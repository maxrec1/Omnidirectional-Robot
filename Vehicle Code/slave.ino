#include <WiFi.h>     // Include the WiFi library for ESP32
#include <esp_now.h>  // Include the ESP-NOW library

// Define struct for transmitted data
struct struct_message {
  float x;
  float y;
  float z;
} myData;

// Define Robot Movements as an enum for readability
enum Movements { STOP,
                 FORWARD,
                 BACKWARD,
                 LEFT,
                 RIGHT,
                 FORWARD_LEFT,
                 FORWARD_RIGHT,
                 BACKWARD_LEFT,
                 BACKWARD_RIGHT,
                 TURN_LEFT,
                 TURN_RIGHT };

// Motor struct to hold control pins
struct Motor {
  int IN1, IN2, EN;
};

// Motor control constants
#define MAX_MOTOR_SPEED 250
const int PWMFreq = 1000, PWMResolution = 8, SIGNAL_TIMEOUT = 2000;
unsigned long lastRecvTime = 0, movementStartTime = 0;

// Define motors
Motor motors[] = {
  { 5, 17, 16 },   // Front Left
  { 2, 4, 15 },    // Back Left
  { 25, 26, 13 },  // Front Right
  { 27, 14, 23 }   // Back Right
};

// Movement states for motors (in1, in2, speed)
const int movements[][4][3] = {
  { { LOW, LOW, 0 }, { LOW, LOW, 0 }, { LOW, LOW, 0 }, { LOW, LOW, 0 } },                                                                    // STOP
  { { HIGH, LOW, MAX_MOTOR_SPEED }, { HIGH, LOW, MAX_MOTOR_SPEED }, { HIGH, LOW, MAX_MOTOR_SPEED }, { HIGH, LOW, MAX_MOTOR_SPEED } },        // FORWARD
  { { LOW, HIGH, MAX_MOTOR_SPEED }, { LOW, HIGH, MAX_MOTOR_SPEED }, { LOW, HIGH, MAX_MOTOR_SPEED }, { LOW, HIGH, MAX_MOTOR_SPEED } },        // BACKWARD
  { { LOW, HIGH, MAX_MOTOR_SPEED }, { HIGH, LOW, MAX_MOTOR_SPEED }, { HIGH, LOW, MAX_MOTOR_SPEED }, { LOW, HIGH, MAX_MOTOR_SPEED } },        // LEFT
  { { HIGH, LOW, MAX_MOTOR_SPEED }, { LOW, HIGH, MAX_MOTOR_SPEED }, { LOW, HIGH, MAX_MOTOR_SPEED }, { HIGH, LOW, MAX_MOTOR_SPEED } },        // RIGHT
  { { LOW, LOW, MAX_MOTOR_SPEED / 2 }, { HIGH, LOW, MAX_MOTOR_SPEED / 2 }, { HIGH, LOW, MAX_MOTOR_SPEED }, { LOW, LOW, MAX_MOTOR_SPEED } },  // FORWARD_LEFT
  { { HIGH, LOW, MAX_MOTOR_SPEED }, { LOW, LOW, MAX_MOTOR_SPEED }, { LOW, LOW, MAX_MOTOR_SPEED / 2 }, { HIGH, LOW, MAX_MOTOR_SPEED / 2 } },  // FORWARD_RIGHT
  { { LOW, LOW, 0 }, { LOW, HIGH, MAX_MOTOR_SPEED }, { LOW, HIGH, MAX_MOTOR_SPEED }, { LOW, LOW, 0 } },                                      // BACKWARD_LEFT
  { { LOW, HIGH, MAX_MOTOR_SPEED }, { LOW, LOW, 0 }, { LOW, LOW, 0 }, { LOW, HIGH, MAX_MOTOR_SPEED } },                                      // BACKWARD_RIGHT
  { { HIGH, LOW, MAX_MOTOR_SPEED }, { HIGH, LOW, MAX_MOTOR_SPEED }, { LOW, HIGH, MAX_MOTOR_SPEED }, { LOW, HIGH, MAX_MOTOR_SPEED } },        // TURN_LEFT
  { { LOW, HIGH, MAX_MOTOR_SPEED }, { LOW, HIGH, MAX_MOTOR_SPEED }, { HIGH, LOW, MAX_MOTOR_SPEED }, { HIGH, LOW, MAX_MOTOR_SPEED } }         // TURN_RIGHT
};

// Array to hold direction names
const char *directions[] = {
  "Stopping", "Moving Forward", "Moving Backward", "Moving Left", "Moving Right",
  "Moving Forward Left", "Moving Forward Right", "Moving Backward Left",
  "Moving Backward Right", "Turning Left", "Turning Right"
};

// Callback function to receive data via ESP-NOW
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  lastRecvTime = millis();  // Update last received time
}

// Define PWM channels for each motor
const int pwmChannels[] = { 0, 1, 2, 3 };  // Channels 0-3 for 4 motors

// Compensation factors for each motor
const float motorCompensation[] = {
  0.9,  // Front Left // 90% capacity to compensate for weaker motor
  1.0,  // Back Left // Weaker motor, no compensation
  0.9,  // Front Right
  0.9   // Back Right
};

void moveRobot(Movements movement, int speed) {
  Serial.print(directions[movement]);  // Print the movement direction
  Serial.print(" at speed: ");         // Log the speed value
  Serial.println(speed);               // Print the actual speed

  for (int i = 0; i < 4; i++) {
    // Set the direction of the motor
    digitalWrite(motors[i].IN1, movements[movement][i][0]);
    digitalWrite(motors[i].IN2, movements[movement][i][1]);

    // Apply motor compensation factor and write the PWM value to the EN pin
    int adjustedSpeed = speed * motorCompensation[i];
    ledcWrite(motors[i].EN, map(adjustedSpeed, 0, 100, 0, MAX_MOTOR_SPEED));
  }
}

// Function to calculate motor speed based on sensor angles
int calculateSpeed(float angle, float maxAngle) {
  // Constrain the angle to the max range to avoid overflow
  float constrainedAngle = constrain(abs(angle), 0, maxAngle);
  // Map the angle to a speed range (e.g., 0 to 100% of motor power)
  int speed = map(constrainedAngle, 0, maxAngle, 0, MAX_MOTOR_SPEED);
  return speed;
}

// Sensor pins
const int sensorPins[] = { 18, 19, 21, 3 };  // Back, Left, Front, Right

// Distance threshold in cm
const int distanceThreshold = 16;  // Minimun detectable sensor distance is 15cm

// Function to read distance from EZ1 sensor
int readDistance(int sensorPin) {
  int cm = 0;
  unsigned long value = pulseIn(sensorPin, HIGH);
  value = value / 58.87;
  cm = int(value);
  return cm;
}


void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  delay(100);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  for (int i = 0; i < 4; i++) {
    pinMode(motors[i].IN1, OUTPUT);
    pinMode(motors[i].IN2, OUTPUT);
    // Attach PWM for each motor using the correct pin and new API
    ledcAttach(motors[i].EN, PWMFreq, PWMResolution);  // Attach PWM to the EN pin
  }

  movementStartTime = millis();
}

// Map sensor data to movement and control the robot
void loop() {

  // Read distances from sensors
  int distances[4];
  for (int i = 0; i < 4; i++) {
    distances[i] = readDistance(sensorPins[i]);
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(distances[i]);
  }

  // Collision avoidance logic
  bool canMoveForward = distances[2] > 15;   // Front sensor
  bool canMoveBackward = distances[0] > 15;  // Back sensor
  bool canMoveLeft = distances[1] > 15;      // Left sensor
  bool canMoveRight = distances[3] > 15;     // Right sensor

  if ((millis() - lastRecvTime) > SIGNAL_TIMEOUT) {
    moveRobot(STOP, 0);
    return;
  }

  Movements movement = STOP;
  int speed = 0;  // Initialize speed

  // Use the tilt angles to calculate the speed dynamically
  int speedX = calculateSpeed(myData.x, 45);  // Assuming a max tilt of 45 degrees
  int speedY = calculateSpeed(myData.y, 45);
  int speedZ = calculateSpeed(myData.z, 45);

  // Determine movement based on the tilt angle and set the dynamic speed
  if (myData.x > 15) {
    movement = (myData.y > 15) ? BACKWARD_LEFT : (myData.y < -15) ? BACKWARD_RIGHT
                                                                  : BACKWARD;
    speed = speedX;  // Use dynamic speed for backward movements
  } else if (myData.x < -15) {
    movement = (myData.y > 15) ? FORWARD_LEFT : (myData.y < -15) ? FORWARD_RIGHT
                                                                 : FORWARD;
    speed = speedX;  // Use dynamic speed for forward movements
  } else if (myData.y > 15) {
    movement = LEFT;
    speed = speedY;  // Use dynamic speed for left movements
  } else if (myData.y < -15) {
    movement = RIGHT;
    speed = speedY;  // Use dynamic speed for right movements
  } else if (myData.z > 15) {
    movement = TURN_RIGHT;
    speed = speedZ;  // Use dynamic speed for turning right
  } else if (myData.z < -15) {
    movement = TURN_LEFT;
    speed = speedZ;  // Use dynamic speed for turning left
  } else {
    movement = STOP;
    speed = 0;  // Set speed to 0 when stopping
  }

  moveRobot(movement, speed);
}
