/*
Amateur Ham radio satellite antenna auto rotator, based on ESP32.
Supports remote control via Bluetooth using Android app LOOK4SAT.
RS485 decoder should support PELCO-D protocol, set baud rate to 2400, address code to 01.
Display: SSD1306 128*64
Connections: ESP32 Serial - MAX3485 TTL to 485 module - RS485 decoder - PTZ mount (I'm using YAAN-3040).
BY BA7JJQ
*/

#include "BluetoothSerial.h"
#include <SoftwareSerial.h>
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Define the rotation speeds for pan and tilt directions. Please manually control the PTZ to rotate a certain distance and calculate the rotation speed (the larger the stroke, the more accurate the result).
const float PAN_SPEED = 7.88439306; // Rotation speed in degrees per second for the pan direction. For reference: 301 PTZ moves 348°/62.156 seconds (0° to 348°, total 348°), and Yaan 3040 moves 341°/43.250 seconds (0° to 341°, total 341°).
const float TILT_SPEED = 5.09107113; // Rotation speed in degrees per second for the tilt direction. For reference: 301 PTZ moves 69.8°/57.235 seconds (-33.5° to 36.3°, total 69.8°), and Yaan 3040 moves 91.4°/17.953 seconds (-4.2° to 87.2°, total 91.4°).

// Define the sensitivity: the PTZ will only start rotating when the target position and the current position exceed the sensitivity threshold. Recommended to set between 2-5°.
const int SENSITIVITY = 2;
// Define the lead: each time the PTZ rotates, the antenna will rotate 1-2° ahead of the target position to achieve as complete coverage as possible.
const int LEAD = 1;

// Define the limits for pan and tilt directions of the PTZ.
const int PAN_MIN = 0;
const int PAN_MAX = 341;
const int TILT_MIN = -4;
const int TILT_MAX = 87;

// Your ESP32 Bluetooth MAC address.
String BTMAC = "00:00:00:00:00:00";

// Define the initial pan and tilt angles of the PTZ.
float currentPan = 0;
float currentTilt = -4.2;

// Define PELCO-D commands to control the PTZ.
const byte LEFT[7] = {0xFF, 0x01, 0x00, 0x04, 0x3F, 0x00, 0x44};
const byte RIGHT[7] = {0xFF, 0x01, 0x00, 0x02, 0x3F, 0x00, 0x42};
const byte UP[7] = {0xFF, 0x01, 0x00, 0x10, 0x00, 0x3F, 0x50};
const byte DOWN[7] = {0xFF, 0x01, 0x00, 0x08, 0x00, 0x3F, 0x48};
const byte ALLSTOP[7] = {0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01};

// Define the Bluetooth serial port.
BluetoothSerial SerialBT;

// Define the pins of TTL to RS485 module: RX connected to pin 16, TX connected to pin 17.
SoftwareSerial SerialRotator(16, 17);

// Define the manual control pins:
const int manualPin = 26;    // switch between manual and auto mode
const int upPin = 13;         // Up key connected to GPIO 13
const int downPin = 12;       // Down key connected to GPIO 12
const int leftPin = 14;       // Left key connected to GPIO 14
const int rightPin = 27;      // Right key connected to GPIO 27

int targetPan;
int targetTilt;
int satPan;
int satTilt;
unsigned long upPressed = 0;
unsigned long downPressed = 0;
unsigned long leftPressed = 0;
unsigned long rightPressed = 0;
bool manualState = false;
bool lastUpState = HIGH;
bool lastDownState = HIGH;
bool lastLeftState = HIGH;
bool lastRightState = HIGH;
bool buttonPressed = false;

unsigned long debounceDelay = 50; // Set the button debounce delay to 50ms

// Define the SSD1306 screen object
Adafruit_SSD1306 display(128, 64, &Wire, -1);

void setup() {
  // Initialize Bluetooth serial
  SerialBT.begin("ESP-Rotator");
  Serial.begin(2400);
  Serial.println("Booting up, please connect Bluetooth!");

  // Initialize software serial
  SerialRotator.begin(2400);
  // Initialize display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // Initialize buttons
  pinMode(manualPin, INPUT_PULLUP);
  pinMode(upPin, INPUT_PULLUP);
  pinMode(downPin, INPUT_PULLUP);
  pinMode(leftPin, INPUT_PULLUP);
  pinMode(rightPin, INPUT_PULLUP);
  // Set font
  display.setTextSize(1);
  display.setTextColor(WHITE);

  // Booting screen
  display.clearDisplay();
  delay(2000);
  display.setCursor(5, 0);
  display.print(" SATTELLITE TRACKER");
  display.setCursor(52, 25);
  display.print("v1.0");
  display.setCursor(36, 40);
  display.print("BY BA7JJQ");
  display.setCursor(13, 55);

  // Display local Bluetooth address
  display.print(BTMAC);
  display.display();
  delay(1000);

  // Initialize PTZ unit: rotate left 360°, then down 90° to ensure PTZ reaching limits.
  delay(1000);
  Serial.println("Initializing PTZ unit, estimated time: " + String(PAN_MAX/PAN_SPEED + TILT_MAX/TILT_SPEED + 6) + " seconds, please wait!");
  sendCommand(LEFT);
  initialPan();
  sendCommand(ALLSTOP);
  delay(1000);
  Serial.println("Horizontal calibration completed, starting tilt calibration, about " + String(TILT_MAX/TILT_SPEED + 3) + " seconds left, please wait!");
  sendCommand(DOWN);
  initialTilt();
  sendCommand(ALLSTOP);
  Serial.println("PTZ unit initialization completed!");

  // Display "DONE!" on screen
  display.clearDisplay();
  display.setCursor(57, 32);
  display.print("DONE!");
  display.display();
  delay(1000);
}

void loop() {
  // Determine automatic/manual mode (toggle switch)
  if (digitalRead(manualPin) == LOW) { // If the switch is toggled to manual mode
    manualMode();
  }
  else {
    autoMode();
  }
}

void autoMode() {
  // Listen for data on Bluetooth serial
  if (SerialBT.available() >= 8) {
    char startCode = SerialBT.read();
    // If the start code is correct, read azimuth and elevation angles
    if (startCode == 'W') {
      targetPan = SerialBT.parseInt();
      satPan = targetPan;
      SerialBT.read();
      targetTilt = SerialBT.parseInt(); 
      satTilt = targetTilt; 
    }
  }
  // Display current position and target satellite position on the screen
  displayPosition(currentPan, currentTilt, satPan, satTilt);
  
  // If the target position exceeds the limits, stop the PTZ unit at the limit
  if (targetPan < PAN_MIN) targetPan = PAN_MIN;
  if (targetPan > PAN_MAX) targetPan = PAN_MAX;
  if (targetTilt < TILT_MIN) targetTilt = TILT_MIN;
  if (targetTilt > TILT_MAX) targetTilt = TILT_MAX;
  Serial.println("Current position: Pan:" + String(currentPan) + " Tilt:" + String(currentTilt) + " Satellite position: Pan:" + String(targetPan) + " Tilt:" + String(targetTilt));

  // Determine if rotation is needed in the horizontal direction
  if (abs(targetPan - currentPan) >= SENSITIVITY) {
    // Calculate rotation time
    float panTime = 1000 * abs(targetPan - currentPan) / PAN_SPEED + 1000 * LEAD / PAN_SPEED;
    unsigned long startTime = millis(); 
    // If left rotation is needed, send left command
    if (targetPan < currentPan) {
      sendCommand(LEFT);
      Serial.println("Rotating left, estimated time: " + String(panTime/1000) + " seconds");
      display.fillRect(0, 0, 2, 64, WHITE);     // Display left on screen
      display.display();
      while (millis() - startTime < panTime) {
        // Wait for rotation to complete
      }
      sendCommand(ALLSTOP);
      currentPan = targetPan - LEAD;
      if (currentPan < PAN_MIN) currentPan = PAN_MIN;
      Serial.println("Arrived! Current azimuth angle: " + String(currentPan));
    }
    // Otherwise, if right rotation is needed, send right command
    else {
      sendCommand(RIGHT);
      Serial.println("Rotating right, estimated time: " + String(panTime/1000) + " seconds");
      display.fillRect(126, 0, 128, 64, WHITE); // Display right on screen
      display.display();
      while (millis() - startTime < panTime) {
        // Wait for rotation to complete
      }
      sendCommand(ALLSTOP);
      currentPan = targetPan + LEAD;
      if (currentPan > PAN_MAX) currentPan = PAN_MAX;
      Serial.println("Arrived! Current azimuth angle: " + String(currentPan));
    }
  }

  // Determine if rotation is needed in the vertical direction
  if (abs(targetTilt - currentTilt) >= SENSITIVITY && satTilt >= 0) {
    // Calculate rotation time
    float tiltTime = 1000 * abs(targetTilt - currentTilt) / TILT_SPEED + 1000 * LEAD / TILT_SPEED;
    unsigned long startTime = millis(); // Record start time
    // If upward rotation is needed, send up command
    if (targetTilt > currentTilt) {
      sendCommand(UP);
      Serial.println("Rotating up, estimated time: " + String(tiltTime/1000) + " seconds");
      display.fillRect(0, 0, 128, 2, WHITE);    // Display up on screen
      display.display();
      while (millis() - startTime < tiltTime) {
        // Wait for rotation to complete
      }
      sendCommand(ALLSTOP);
      currentTilt = targetTilt + LEAD;
      if (currentTilt < TILT_MIN) currentTilt = TILT_MIN;
      Serial.println("Arrived! Current elevation angle: " + String(currentTilt));
    }
    // Otherwise, if downward rotation is needed, send down command
    else {
      sendCommand(DOWN);
      Serial.println("Rotating down, estimated time: " + String(tiltTime/1000) + " seconds");
      display.fillRect(0, 62, 128, 64, WHITE);  // Display down on screen
      display.display();
      while (millis() - startTime < tiltTime) {
        // Wait for rotation to complete
      }
      sendCommand(ALLSTOP);
      currentTilt = targetTilt - LEAD;
      if (currentTilt > TILT_MAX) currentTilt = TILT_MAX;
      Serial.println("Arrived! Current elevation angle: " + String(currentTilt));
    }
  }
}

void manualMode() {
  // Read the status of each button
  manualPosition(currentPan, currentTilt);
  int upReading = digitalRead(upPin);
  int downReading = digitalRead(downPin);
  int leftReading = digitalRead(leftPin);
  int rightReading = digitalRead(rightPin);
  
  // Check if the up button state has changed
  if (upReading != lastUpState) {
    if (upReading == LOW) {
      delay(debounceDelay); 
      sendCommand(UP);
      display.fillRect(0, 0, 128, 2, WHITE);    // Display up on screen
      display.display();
      if (digitalRead(upPin) == LOW) { // Confirm button is still pressed
        buttonPressed = true;
        upPressed = millis();
        Serial.println("Rotating up");
      }
    }
  }

  // Check if the down button state has changed
  if (downReading != lastDownState) {
    if (downReading == LOW) {
      delay(debounceDelay); 
      sendCommand(DOWN);
      display.fillRect(0, 62, 128, 64, WHITE);  // Display down on screen
      display.display();
      if (digitalRead(downPin) == LOW) { // Confirm button is still pressed
        buttonPressed = true;
        downPressed = millis();
        Serial.println("Rotating down");
      }
    }
  }

  // Check if the left button state has changed
  if (leftReading != lastLeftState) {
    if (leftReading == LOW) {
      delay(debounceDelay); 
      sendCommand(LEFT);
      display.fillRect(0, 0, 2, 64, WHITE);     // Display left on screen
      display.display();
      if (digitalRead(leftPin) == LOW) { // Confirm button is still pressed
        buttonPressed = true;
        leftPressed = millis();
        Serial.println("Rotating left");
      }
    }
  }

  // Check if the right button state has changed
  if (rightReading != lastRightState) {
    if (rightReading == LOW) {
      delay(debounceDelay); 
      sendCommand(RIGHT);
      display.fillRect(126, 0, 128, 64, WHITE); // Display right on screen
      display.display();
      if (digitalRead(rightPin) == LOW) { // Confirm button is still pressed
        buttonPressed = true;
        rightPressed = millis();
        Serial.println("Rotating right");
      }
    }
  }

  // Update the previous button states
  lastUpState = upReading;
  lastDownState = downReading;
  lastLeftState = leftReading;
  lastRightState = rightReading;

  // If the button was pressed and released, output the press duration and reset it
  if (upPressed > 0 && upReading == HIGH && buttonPressed) {
    buttonPressed = false;
    Serial.print("Rotated up ");
    Serial.print(millis() - upPressed);
    Serial.println(" milliseconds.");
    float result = (millis() - upPressed) / 1000.0;
    float newTilt = currentTilt + result * TILT_SPEED;
    // Add limits
    if (newTilt > TILT_MAX) {
        currentTilt = TILT_MAX;
    } else {
        currentTilt = newTilt;
    }
    // Stop action
    sendCommand(ALLSTOP);
    Serial.println("Current azimuth angle: " + String(currentPan) + " Current elevation angle: " + String(currentTilt));
    upPressed = 0;
  }

  if (downPressed > 0 && downReading == HIGH && buttonPressed) {
    buttonPressed = false;
    Serial.print("Rotated down ");
    Serial.print(millis() - downPressed);
    Serial.println(" milliseconds.");
    float result = (millis() - downPressed) / 1000.0;
    float newTilt = currentTilt - result * TILT_SPEED;
    // Add limits
    if (newTilt < TILT_MIN) {
        currentTilt = TILT_MIN;
    } else {
        currentTilt = newTilt;
    }
    // Stop action
    sendCommand(ALLSTOP);
    Serial.println("Current azimuth angle: " + String(currentPan) + " Current elevation angle: " + String(currentTilt));
    downPressed = 0;
  }

  if (leftPressed > 0 && leftReading == HIGH && buttonPressed) {
    buttonPressed = false;
    Serial.print("Rotated left ");
    Serial.print(millis() - leftPressed);
    Serial.println(" milliseconds.");
    float result = (millis() - leftPressed) / 1000.0;
    float newPan = currentPan - result * PAN_SPEED;
    // Add limits
    if (newPan < PAN_MIN) {
        currentPan = PAN_MIN;
    } else {
        currentPan = newPan;
    }
    // Stop action
    sendCommand(ALLSTOP);
    Serial.println("Current azimuth angle: " + String(currentPan) + " Current elevation angle: " + String(currentTilt));
    leftPressed = 0;
  }

  if (rightPressed > 0 && rightReading == HIGH && buttonPressed) {
    buttonPressed = false;
    Serial.print("Rotated right ");
    Serial.print(millis() - rightPressed);
    Serial.println(" milliseconds.");
    float result = (millis() - rightPressed) / 1000.0;
    float newPan = currentPan + result * PAN_SPEED;
    // Add limits
    if (newPan > PAN_MAX) {
        currentPan = PAN_MAX;
    } else {
        currentPan = newPan;
    }
    // Stop action
    sendCommand(ALLSTOP);
    Serial.println("Current azimuth angle: " + String(currentPan) + " Current elevation angle: " + String(currentTilt));
    rightPressed = 0;
  }
}

// Send motion command to the 485 decoder via serial port
void sendCommand(const byte command[]) {
  for (int i = 0; i < 7; i++) {
    SerialRotator.write(command[i]);
  }
}

// Automode display: show PTZ position and satellite position on the screen
void displayPosition(int currentPan, int currentTilt, int satPan, int satTilt) {
  // Clear display.
  display.clearDisplay();

  // Draw split line.
  display.drawLine(0, 32, 128, 32, WHITE);

  // Display current location.
  display.setCursor(42, 6);
  display.print("CURRENT");
  display.setCursor(14, 18);
  display.print("PAN:");
  display.print(currentPan);
  display.setCursor(64, 18);
  display.print(" TILT:");
  display.print(currentTilt);

  // Display sat location.
  display.setCursor(34, 38);
  display.print("SATTELLITE");
  display.setCursor(14, 50);
  display.print("PAN:");
  display.print(satPan);
  display.setCursor(64, 50);
  display.print(" TILT:");
  display.print(satTilt);
  
  display.display();

// Countdown display for pan initialization
void initialPan() {
  // Clear the screen
  display.clearDisplay();

  // Display "INITIALIZING PAN" on the first line
  display.setCursor(16, 0);
  display.print("INITIALIZING PAN");

  // Display "PLEASE WAIT..." on the second line
  display.setCursor(23, 20);
  display.print("PLEASE WAIT...");
  
  // Refresh the screen
  display.display();

  // Display countdown on the third line
  for (int i = PAN_MAX/PAN_SPEED + 3; i >= 0; i--) {
    // Clear the third line
    display.fillRect(3, 40, 128, 64, BLACK);

    // Display the countdown
    display.setCursor(57, 40);
    display.setTextSize(2);
    display.print(i);
    display.fillRect(0, 0, 2, 64, WHITE);
    
    // Refresh the screen
    display.display();

    // Delay for 1 second
    delay(1000);
  }
}

// Countdown display for tilt initialization
void initialTilt() {
  // Clear the screen
  display.clearDisplay();

  // Display "INITIALIZING TILT" on the first line
  display.setCursor(13, 0);
  display.print("INITIALIZING TILT");

  // Display "PLEASE WAIT..." on the second line
  display.setCursor(23, 20);
  display.print("PLEASE WAIT...");

  // Refresh the screen
  display.display();

  // Display countdown on the third line
  for (int i = TILT_MAX/TILT_SPEED + 3; i >= 0; i--) {
    // Clear the third line
    display.fillRect(57, 40, 80, 54, BLACK);

    // Display the countdown
    display.setCursor(57, 40);
    display.setTextSize(2);
    display.print(i);
    display.fillRect(0, 62, 128, 64, WHITE);
    
    // Refresh the screen
    display.display();

    // Restore font size to 1
    display.setTextSize(1);
    // Delay for 1 second
    delay(1000);
  }
}
