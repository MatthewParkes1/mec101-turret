/*
 * Choose communication mode define here:
 *    I2C_MODE    : I2C mode, default pin: MU_SDA <==> ARDUINO_SDA, MU_SCL <==> ARDUINO_SCL
 *    SERIAL_MODE : Serial mode, default pin: MU_TX <==> ARDUINO_PIN3, MU_RX <==> ARDUINO_PIN2
 */
#define I2C_MODE
#include <Servo.h>
//#define SERIAL_MODE

/*
 * Choose MU address here: 0x60, 0x61, 0x62, 0x63
 *        default address: 0x60
 */
#define MU_ADDRESS 0x60

#include <Arduino.h>
#include <MuVisionSensor.h>

#ifdef I2C_MODE
#include <Wire.h>
#endif
#ifdef SERIAL_MODE
#include <SoftwareSerial.h>
#define TX_PIN 2
#define RX_PIN 3
SoftwareSerial mySerial(RX_PIN, TX_PIN);
#endif
MuVisionSensor Mu(MU_ADDRESS);
Servo servo_Y;
Servo servo_X;

int pumpEnable = 6;
int pumpPin1 = 4;
int pumpPin2 = 5;
void writeX(int deg) {
  servo_X.write(-1 * (deg + 90));
}

void writeY(int deg) {
  servo_Y.write(-1 * (deg + 90));
}
void writeCord(int x, int y) {
  servo_X.write(-1 * x + 90);
  servo_Y.write(-1 * y + 90);
}
void setup() {
  digitalWrite(pumpEnable, HIGH);
  digitalWrite(pumpPin1, pumpPin1);
  digitalWrite(pumpPin2, LOW);
  servo_Y.attach(9, 500, 2500);
  servo_X.attach(10, 500, 2500);
  // put your setup code here, to run once:
  Serial.begin(9600);
  uint8_t err = 0;
#ifdef I2C_MODE
  Wire.begin();
  err = Mu.begin(&Wire);  // initialized MU on I2C port
#elif defined SERIAL_MODE
  mySerial.begin(9600);
  err = Mu.begin(&mySerial);  // initialized MU on soft serial port
#endif
  if (err == MU_OK) {
    Serial.println("MU initialized.");
  } else {
    do {
      Serial.println("fail to initialize MU! Please check protocol "
                     "version or make sure MU is working on the "
                     "correct port with correct mode.");
      delay(5000);
    } while (1);
  }
  // enable vision: body detect
  Mu.VisionBegin(VISION_BODY_DETECT);  // enable vision body
  Serial.println("Filling Buffer");
}
int currentX = 90;
int currentY = 90;
int toDegrees(int value, bool y) {

  int coord = (value - 50);
  // int fovY = 53.9999999999;
  // int fovX = 72;
  double ppdY = 2.16;
  double ppdX = 2.88;
  int degrees = 0;
  if (y) {
    degrees = (-1 * (coord * ppdY));
  } else {
    degrees = coord * ppdX;
  }
  return degrees;
}

int framesUntracked = 0;
int xBuffer[3];
int bufferFramesRemaining = 3;
int startedAt = 0;
int framesCentered = 0;
int current = 90;
void loop() {
  // put your main code here, to run repeatedly:



  if (Mu.GetValue(VISION_BODY_DETECT, kStatus)) {  // update vision result and get status, 0: undetected, other: detected

    framesUntracked = 0;
    if (bufferFramesRemaining > 0) {
      xBuffer[2 - bufferFramesRemaining] = Mu.GetValue(VISION_BODY_DETECT, kXValue) - 50;
      bufferFramesRemaining--;
      if (bufferFramesRemaining == 0) {
        Serial.println("Buffer saturated");
      }
      return;
    }
    int x = Mu.GetValue(VISION_BODY_DETECT, kXValue) - 50;

    int y = Mu.GetValue(VISION_BODY_DETECT, kYValue) - 50;  // get vision result: y axes value

    int width = Mu.GetValue(VISION_BODY_DETECT, kWidthValue);  // get vision result: width value

    int height = Mu.GetValue(VISION_BODY_DETECT, kHeightValue);  // get vision result: height value

    y *= -1;
    y += 10;
    x += 0;
    //Serial.println(x);
    
    if (abs(x) > 5) {
      if (x > 0) {
        if (!servo_X.attached())
          servo_X.attach(10);
        servo_X.write(85);
        //Serial.println("RIGHT");
        framesCentered = 0;
        current = 85;
      } else {
        if (!servo_X.attached())
          servo_X.attach(10);
        servo_X.write(92);
       // Serial.println("LEFT");
        framesCentered = 0;
        current = 92;
      }
    } else {
      if (current != 90) {
        servo_X.write(90);
        current = 90;
      }
      //Serial.println("STAY");
      framesCentered++;
      //servo_X.detach();
    }
    if (framesCentered == 5 && startedAt == 0) {
      //Fire
      Serial.println("FIRE");
      digitalWrite(pumpPin1, HIGH);
      startedAt = millis();
    }
    if (startedAt != 0 && millis() - startedAt >= 1000) {
      digitalWrite(pumpPin1, LOW);
      startedAt = 0;
      framesCentered = 0;
    }
    servo_Y.write(60);
  } else {
    //servo_X.detach();
    digitalWrite(pumpPin1, LOW);
    framesUntracked++;
      if (current != 90) {
        servo_X.write(90);
        current = 90;
      }
    if (framesUntracked >= 5) {
      servo_Y.write(90);
      Serial.println("RETURN");
    }
  }

  // Serial.print("fps = ");
  // Serial.println(1000 / (millis() - time_start));
  // Serial.println();
}
