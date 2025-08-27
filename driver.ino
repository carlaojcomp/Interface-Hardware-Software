/*
 * Arduino TTY Mouse Firmware
 * Compatible with arduino_tty_mouse.c Linux driver
 * 
 * Hardware connections:
 * - Pin 2: Left mouse button (connect to GND when pressed)
 * - Pin 3: Right mouse button (connect to GND when pressed)
 * - Pin A0: X-axis potentiometer (optional)
 * - Pin A1: Y-axis potentiometer (optional)
 * - Pin 13: Status LED
 * 
 * Protocol:
 * - Start byte: 0xAA
 * - Data byte: bit 0 = left button, bit 1 = right button, bit 7 = has movement
 * - X movement: signed byte (-128 to 127) if has movement
 * - Y movement: signed byte (-128 to 127) if has movement
 * - Checksum: XOR of all data bytes
 * - End byte: 0x55
 */

// Pin definitions
#define LEFT_BUTTON_PIN 2
#define RIGHT_BUTTON_PIN 3
#define X_AXIS_PIN A0
#define Y_AXIS_PIN A1
#define LED_PIN 13

// Protocol constants
#define PROTOCOL_START_BYTE 0xAA
#define PROTOCOL_END_BYTE 0x55
#define BUTTON_LEFT_MASK 0x01
#define BUTTON_RIGHT_MASK 0x02
#define HAS_MOVEMENT_MASK 0x80

// State variables
bool leftButtonPressed = false;
bool rightButtonPressed = false;
int lastXValue = 512;  // Center position for potentiometer
int lastYValue = 512;  // Center position for potentiometer
unsigned long lastSendTime = 0;
bool connected = false;
bool enabled = false;

void setup() {
  // Initialize serial communication at 9600 baud
  Serial.begin(9600);
  
  // Configure pins
  pinMode(LEFT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize analog reference
  analogReference(DEFAULT);
  
  // Read initial potentiometer values
  lastXValue = analogRead(X_AXIS_PIN);
  lastYValue = analogRead(Y_AXIS_PIN);
  
  // Indicate startup
  blinkLED(3);
  
  Serial.println("Arduino TTY Mouse v1.0 Ready");
}

void loop() {
  // Handle incoming commands
  handleSerialCommands();
  
  // Only send mouse data if enabled
  if (enabled) {
    // Read button states
    bool currentLeftButton = !digitalRead(LEFT_BUTTON_PIN);  // Active low
    bool currentRightButton = !digitalRead(RIGHT_BUTTON_PIN); // Active low
    
    // Read potentiometer values for movement
    int currentXValue = analogRead(X_AXIS_PIN);
    int currentYValue = analogRead(Y_AXIS_PIN);
    
    // Calculate movement
    int deltaX = (currentXValue - lastXValue) / 4;  // Scale down movement
    int deltaY = (currentYValue - lastYValue) / 4;
    
    // Limit movement range
    deltaX = constrain(deltaX, -127, 127);
    deltaY = constrain(deltaY, -127, 127);
    
    // Check if we need to send an update
    bool buttonChanged = (currentLeftButton != leftButtonPressed) || 
                        (currentRightButton != rightButtonPressed);
    bool hasMovement = (abs(deltaX) > 2) || (abs(deltaY) > 2);  // Dead zone
    
    if (buttonChanged || hasMovement || (millis() - lastSendTime > 100)) {
      sendMousePacket(currentLeftButton, currentRightButton, deltaX, deltaY, hasMovement);
      
      // Update state
      leftButtonPressed = currentLeftButton;
      rightButtonPressed = currentRightButton;
      if (hasMovement) {
        lastXValue = currentXValue;
        lastYValue = currentYValue;
      }
      lastSendTime = millis();
      
      // Update LED to show activity
      digitalWrite(LED_PIN, enabled && connected);
    }
  }
  
  delay(10);  // Small delay to prevent overwhelming the serial port
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "PING") {
      Serial.println("DEUS DA COMPUTARIA");
      connected = true;
    }
    else if (command == "RESET") {
      Serial.println("REBOBINAR");
      enabled = false;
      connected = false;
      leftButtonPressed = false;
      rightButtonPressed = false;
      digitalWrite(LED_PIN, LOW);
    }
    else if (command == "ENABLE") {
      Serial.println("PRADO-SENSEI");
      enabled = true;
      connected = true;
      digitalWrite(LED_PIN, HIGH);
    }
    else if (command == "DISABLE") {
      Serial.println("DISABLED");
      enabled = false;
      digitalWrite(LED_PIN, LOW);
    }
    else if (command == "STATUS") {
      Serial.print("STATUS:");
      Serial.print(connected ? "CONNECTED" : "DISCONNECTED");
      Serial.print(",");
      Serial.print(enabled ? "ENABLED" : "DISABLED");
      Serial.print(",LEFT:");
      Serial.print(leftButtonPressed ? "1" : "0");
      Serial.print(",RIGHT:");
      Serial.println(rightButtonPressed ? "1" : "0");
    }
  }
}

void sendMousePacket(bool leftBtn, bool rightBtn, int8_t deltaX, int8_t deltaY, bool hasMovement) {
  uint8_t dataBytes[4];
  uint8_t dataCount = 0;
  
  // Start byte
  Serial.write(PROTOCOL_START_BYTE);
  
  // Status byte
  uint8_t statusByte = 0;
  if (leftBtn) statusByte |= BUTTON_LEFT_MASK;
  if (rightBtn) statusByte |= BUTTON_RIGHT_MASK;
  if (hasMovement) statusByte |= HAS_MOVEMENT_MASK;
  
  Serial.write(statusByte);
  dataBytes[dataCount++] = statusByte;
  
  // Movement data (only if has movement)
  if (hasMovement) {
    Serial.write((uint8_t)deltaX);
    Serial.write((uint8_t)deltaY);
    dataBytes[dataCount++] = (uint8_t)deltaX;
    dataBytes[dataCount++] = (uint8_t)deltaY;
  }
  
  // Calculate checksum (XOR of all data bytes)
  uint8_t checksum = 0;
  for (int i = 0; i < dataCount; i++) {
    checksum ^= dataBytes[i];
  }
  Serial.write(checksum);
  
  // End byte
  Serial.write(PROTOCOL_END_BYTE);
}

void blinkLED(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}