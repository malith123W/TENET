#include <Arduino.h>
#include <HardwareSerial.h>
#include "ultrasonic.h"
#include "coloursensor.h"
#include "ISensor.h"

Sensor sensor;
HardwareSerial mySerial(1);

// Motor Driver Pins and other definitions remain unchanged
#define AIN1 27
#define AIN2 26
#define PWMA 14
#define BIN1 25
#define BIN2 33
#define PWMB 32
#define PWM_FREQ 1000
#define PWM_RES 8
#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1

#define ENCODER_A1 13
#define ENCODER_B1 22
#define ENCODER_A2 21
#define ENCODER_B2 23

volatile int encoderCount1 = 0;
volatile int encoderCount2 = 0;

#define TARGET_DISTANCE_CM 30
#define WHEEL_DIAMETER_CM 6.4
#define WHEEL_CIRCUMFERENCE_CM (WHEEL_DIAMETER_CM * 3.14159)
#define ENCODER_PULSES_PER_REV 11
#define GEAR_RATIO 21.3
#define PULSES_PER_WHEEL_REV (ENCODER_PULSES_PER_REV * GEAR_RATIO)
#define TARGET_PULSES ((TARGET_DISTANCE_CM / WHEEL_CIRCUMFERENCE_CM) * PULSES_PER_WHEEL_REV)
#define TURN_PULSES 160

void IRAM_ATTR encoder1ISR() { encoderCount1++; }
void IRAM_ATTR encoder2ISR() { encoderCount2++; }

#define KP 2.31
#define KI 0.0
#define KD 0.95

int lastError = 0;
int integral = 0;

void moveForward(int baseSpeed) {
    int error = encoderCount1 - encoderCount2;
    integral += error;
    int derivative = error - lastError;
    int correction = KP * error + KI * integral + KD * derivative;
    
    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;
    
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);
    
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, leftSpeed);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, rightSpeed);
    
    lastError = error;
}
void moveBackward(int baseSpeed) {
    int error = encoderCount1 - encoderCount2;
    integral += error;
    int derivative = error - lastError;
    
    int correction = KP * error + KI * integral + KD * derivative;
    
    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;
    
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, leftSpeed);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, rightSpeed);

    lastError = error;
}

void stopMotors() {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
}

void turnRight90Degrees() {
    encoderCount1 = 0;
    encoderCount2 = 0;
    Serial.println("Turning Right");
    int turnSpeed = 60;
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, turnSpeed);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, turnSpeed);
    while (encoderCount1 < TURN_PULSES || encoderCount2 < TURN_PULSES) {}
    stopMotors();
    Serial.println("Turn Completed");
}

void turnLeft90Degrees() {
    encoderCount1 = 0;
    encoderCount2 = 0;
    Serial.println("Starting left turn");
    int turnSpeed = 60;
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, turnSpeed);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, turnSpeed);
    while (encoderCount1 < TURN_PULSES-5 && encoderCount2 < TURN_PULSES-5) {
        Serial.print("Turn Left - Encoder1: ");
        Serial.print(encoderCount1);
        Serial.print(" Encoder2: ");
        Serial.println(encoderCount2);
        delay(10);
    }
    stopMotors();
    Serial.println("Left turn completed");
}

// Improved ultrasonic distance function


void setup() {
// Setup remains unchanged
Serial.begin(9600);
sensor.begin();
setupUltrasonic();
setupcoloursensor();

mySerial.begin(9600, SERIAL_8N1, 16, 17);
ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RES);
ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RES);

ledcAttachPin(PWMA, PWM_CHANNEL_A);
ledcAttachPin(PWMB, PWM_CHANNEL_B);

pinMode(AIN1, OUTPUT);
pinMode(AIN2, OUTPUT);
pinMode(PWMA, OUTPUT);
pinMode(BIN1, OUTPUT);
pinMode(BIN2, OUTPUT);
pinMode(PWMB, OUTPUT);

pinMode(ENCODER_A1, INPUT_PULLUP);
pinMode(ENCODER_B1, INPUT_PULLUP);
pinMode(ENCODER_A2, INPUT_PULLUP);
pinMode(ENCODER_B2, INPUT_PULLUP);

attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoder1ISR, RISING);
attachInterrupt(digitalPinToInterrupt(ENCODER_A2), encoder2ISR, RISING);

Serial.println("Setup Complete");

    Serial.println("Setup Complete");
}

void loop() {
    int sensorValue = sensor.readValue();
    Serial.println(sensorValue);
    int forwardCount = 0;
    float distance = getDistance();
    delay(1000);

   

    // Initial move forward with increased speed

    for (int i=0 ; i<5 ; i++){encoderCount1 = 0;
        encoderCount2 = 0;
        moveForward(60);  // Changed from 80 to 60 as requested
        while (sensorValue > 500) {
            sensorValue = sensor.readValue();
            Serial.println(sensorValue);
            delay(1000);
        }
        stopMotors();
        delay(1000);
    
        // Precise distance movement using encoders
        encoderCount1 = 0;
        encoderCount2 = 0;
        moveForward(80);  // Changed from 30 to 60
        while (encoderCount1 < TARGET_PULSES || encoderCount2 < TARGET_PULSES) {
            // PID correction is handled in moveForward function
        }
        stopMotors();
        delay(1000);
        
        turnRight90Degrees();
        stopMotors();
        delay(500);
        
        // Main movement with improved encoder correction
        encoderCount1 = 0;
        encoderCount2 = 0;
        while (true) {
            float distance = getDistance();
            Serial.print("Distance: ");
            Serial.println(distance);
            
            moveForward(60);  // Increased from original
            
            // Enhanced encoder-based correction
            int error = encoderCount1 - encoderCount2;
          
            
            if (distance < 18) {
                stopMotors();
                delay(1000);
                forwardCount = (encoderCount1 + encoderCount2) / 2;
                Serial.print("Arrived at obstacle. Forward count: ");
                Serial.println(forwardCount);
                mySerial.println("Arrived");
                break;
            }
            delay(50);
        }
    
        // Message waiting and backward movement remain unchanged
        Serial.println("Waiting for 'done' message");
        String message = "";
        while (message != "Done") {
            if (mySerial.available()) {
                message = mySerial.readStringUntil('\n');
                message.trim();
                Serial.print("Received message: ");
                Serial.println(message);
            }
            delay(100);
        }
    
        if (message == "Done") {
            Serial.println("Moving backward");
            encoderCount1 = 0;
            encoderCount2 = 0;
            moveBackward(80);
            
            while (abs(encoderCount1) < forwardCount || abs(encoderCount2) < forwardCount) {
                Serial.print("Encoder1: ");
                Serial.print(encoderCount1);
                Serial.print(" Encoder2: ");
                Serial.println(encoderCount2);
                delay(50);
            }
            stopMotors();
            delay(5000);
            Serial.println("Backward movement complete");
    
            Serial.println("Turning left 90 degrees");
            turnLeft90Degrees();
            Serial.println("Turn complete");
            delay(1000);
        }}
        stopMotors();
        moveForward(60);  // Changed from 30 to 60
        while (encoderCount1 < TARGET_PULSES || encoderCount2 < TARGET_PULSES) {
            // PID correction is handled in moveForward function
        }
        moveForward(60);
        while (encoderCount1 < TARGET_PULSES || encoderCount2 < TARGET_PULSES) {
            // PID correction is handled in moveForward function
        }
        moveForward(60);
        while (encoderCount1 < TARGET_PULSES || encoderCount2 < TARGET_PULSES) {
            // PID correction is handled in moveForward function
        }
        moveForward(60);
        while (encoderCount1 < TARGET_PULSES || encoderCount2 < TARGET_PULSES) {
            // PID correction is handled in moveForward function
        }


        stopMotors();
        delay(500);
        turnRight90Degrees();
        moveForward(60);  // Changed from 30 to 60
        while (encoderCount1 < TARGET_PULSES || encoderCount2 < TARGET_PULSES) {
            // PID correction is handled in moveForward function
        }
        moveForward(60);  // Changed from 30 to 60
        while (encoderCount1 < TARGET_PULSES || encoderCount2 < TARGET_PULSES) {
            // PID correction is handled in moveForward function
        }
        moveForward(60);
        while (encoderCount1 < TARGET_PULSES || encoderCount2 < TARGET_PULSES) {
            // PID correction is handled in moveForward function
        }

        stopMotors();
        delay(500);
        turnLeft90Degrees();
        stopMotors();
        delay(500);
         while (true) {
            float distance = getDistance();
            Serial.print("Distance: ");
            Serial.println(distance);
            encoderCount1 = 0;
            encoderCount2 = 0;
            
            moveForward(60);  // Increased from original
            
            // Enhanced encoder-based correction
            while (true ){
                if (distance < 10) {
                    stopMotors();
                    delay(1000);
                    forwardCount = (encoderCount1 + encoderCount2) / 2;
                    Serial.print("Arrived at obstacle. Forward count: ");
                    Serial.println(forwardCount);
                    mySerial.println("Arrived");
                    break;
                }
            }
            
           
            delay(50);
            encoderCount1 = 0;
            encoderCount2 = 0;
            moveBackward(80);
            
            while (abs(encoderCount1) < forwardCount || abs(encoderCount2) < forwardCount) {
                Serial.print("Encoder1: ");
                Serial.print(encoderCount1);
                Serial.print(" Encoder2: ");
                Serial.println(encoderCount2);
                delay(50);
            turnRight90Degrees();
            moveForward(60);
            while (encoderCount1 < TARGET_PULSES || encoderCount2 < TARGET_PULSES) {
                // PID correction is handled in moveForward function
            }
            turnLeft90Degrees();
            moveForward(60);

            
            while(true) {
                if (distance < 10) {
                    stopMotors();
                    delay(1000);
                    forwardCount = (encoderCount1 + encoderCount2) / 2;
                    Serial.print("Arrived at obstacle. Forward count: ");
                    Serial.println(forwardCount);
                    mySerial.println("Arrived2");
                    break;
                }

            }
           
            
           
            
        }
        stopMotors();
        delay(100000);
       


    
}
}