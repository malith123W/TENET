#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>

#define SERVOMIN 170
#define SERVOMAX 650

// First color sensor pins
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

// Second color sensor pins
#define S4 9
#define S5 10
#define S6 11
#define S7 12
#define sensorOut2 3

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int servoCount = 5; // Number of servos you have
const int servoPins[] = {0, 1, 2, 3, 4}; // Define the PCA9685 pins for your servos

int defaultPositions[servoCount] = {90, 90, 80, 90, 90}; // Default positions for each servo
int servoAngles[servoCount]; // Store current servo angles

// Calibration variables
int redMin = 1024, redMax = 0;
int greenMin = 1024, greenMax = 0;
int blueMin = 1024, blueMax = 0;

void calibrateSensor();
void moveServo(int servoIndex, int targetAngle);
void setDefaultPositions();
void arm_mortion();
void colour1();
void colour2();
int getColorValue(int s2, int s3, int sensor);
int angleToPulse(int angle);
void setup() {
    Serial.begin(9600);
    pwm.begin();
    pwm.setPWMFreq(60);

    // Initialize servos to default positions
    for (int i = 0; i < servoCount; i++) {
        servoAngles[i] = defaultPositions[i];
        pwm.setPWM(servoPins[i], 0, angleToPulse(servoAngles[i]));
    }

    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(sensorOut, INPUT);

    digitalWrite(S0, HIGH);
    digitalWrite(S1, LOW);

    pinMode(S4, OUTPUT);
    pinMode(S5, OUTPUT);
    pinMode(S6, OUTPUT);
    pinMode(S7, OUTPUT);
    pinMode(sensorOut2, INPUT);

    digitalWrite(S4, HIGH);
    digitalWrite(S5, LOW);

    Serial.println("Starting calibration. Place the sensor over a white surface...");
    delay(30);
    calibrateSensor();
    Serial.println("Calibration complete!");
}

void moveServo(int servoIndex, int targetAngle) {
    for (int i = servoAngles[servoIndex]; i != targetAngle; i += (targetAngle > i) ? 1 : -1) {
        servoAngles[servoIndex] = i;
        pwm.setPWM(servoPins[servoIndex], 0, angleToPulse(i));
        delay(10);
    }
}

void setDefaultPositions() {
    for (int i = 0; i < servoCount; i++) {
        moveServo(i, defaultPositions[i]);
    }
}

void arm_mortion() {
    moveServo(2, 40);
    delay(300);

    moveServo(0, 150);
    delay(300);

    moveServo(2, 85);
    delay(600);

    colour1();

    moveServo(0, 120);
    delay(300);
    setDefaultPositions();

    Serial.println("Done");
    delay(1000);
}

void colour1() {
    int red = getColorValue(S2, S3, sensorOut);
    int green = getColorValue(S2, S3, sensorOut);
    int blue = getColorValue(S2, S3, sensorOut);

    red = map(red, redMin, redMax, 0, 255);
    green = map(green, greenMin, greenMax, 0, 255);
    blue = map(blue, blueMin, blueMax, 0, 255);

    Serial.print("Red: "); Serial.print(red);
    Serial.print(" | Green: "); Serial.print(green);
    Serial.print(" | Blue: "); Serial.println(blue);

    // Improved color detection for yellow
    if (red > 150 && green > 150 && blue < 100) {
        Serial.println("Detected Color: yellow");
        moveServo(1, 0);
    } else {
        Serial.println("Detected Color: NOT yellow");
        moveServo(1, 180);
    }
    delay(300);
}

void colour2() {
    int red = getColorValue(S6, S7, sensorOut2);
    int green = getColorValue(S6, S7, sensorOut2);
    int blue = getColorValue(S6, S7, sensorOut2);

    red = map(red, redMin, redMax, 0, 255);
    green = map(green, greenMin, greenMax, 0, 255);
    blue = map(blue, blueMin, blueMax, 0, 255);

    Serial.print("Red: "); Serial.print(red);
    Serial.print(" | Green: "); Serial.print(green);
    Serial.print(" | Blue: "); Serial.println(blue);

    if (red < redMax) {
        Serial.println("Detected Color: RED");
        moveServo(3, 0);
    } else {
        Serial.println("Detected Color: NOT RED");
        moveServo(4, 0);
    }
    delay(300);
}

void calibrateSensor() {
    for (int i = 0; i < 30; i++) { // Increased calibration iterations
        int red = getColorValue(S2, S3, sensorOut);
        int green = getColorValue(S2, S3, sensorOut);
        int blue = getColorValue(S2, S3, sensorOut);

        redMin = min(redMin, red);
        redMax = max(redMax, red);
        greenMin = min(greenMin, green);
        greenMax = max(greenMax, green);
        blueMin = min(blueMin, blue);
        blueMax = max(blueMax, blue);
        delay(10); // Small delay to stabilize the sensor between readings
    }
}

int getColorValue(int s2, int s3, int sensor) {
    digitalWrite(s2, LOW);
    digitalWrite(s3, LOW);
    delay(100); // Wait for sensor to stabilize
    return pulseIn(sensor, LOW); // Read the pulse width
}

int angleToPulse(int angle) {
    return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void loop() {
    if (Serial.available()) {
        String receivedData = Serial.readStringUntil('\n');
        receivedData.trim(); // Remove leading/trailing whitespace
        Serial.println(receivedData);

        if (receivedData == "Arrived") {
            arm_mortion();
        } else if (receivedData == "Arrived2" ) {
            colour2();
            Serial.println("release");
        } else {
            Serial.println("NO");
            setDefaultPositions();
        }
    }
}
