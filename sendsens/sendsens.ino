
#include "I2Cdev.h"
#include  <SoftwareSerial.h>
#include "freeram.h"
#include "mpu.h"

int ret;
int ax, ay, az, gx, gy, gz;
int yaw, pitch, roll;

#define LED_PIN 13
bool blinkState = false;
bool startSerial = false;
void setup() {    
        Fastwire::setup(400, true);
        ret = mympu_open(200);
        Serial.begin(57600);
        pinMode(LED_PIN, OUTPUT);
}

int prespin[] = {0, 1, 2, 3, 4};
int flpin[] = {5, 8, 9, 10, 11};

void loop() {

     ret = mympu_update();

//     Serial.write("w");
//     yaw = map((mympu.ypr[0]), -180, 180, 0, 255);
//     pitch = map((mympu.ypr[1]), -90, 90, 0, 255);
//     roll = map((mympu.ypr[2]), -180, 180, 0, 255);
//     gx = constrain(map((mympu.gyro[0]), -180, 180, 0, 255), 0, 255);
//     gy = constrain(map((mympu.gyro[1]), -180, 180, 0, 255), 0 ,255);
//     gz = constrain(map((mympu.gyro[2]), -180, 180, 0, 255), 0, 255);
//        Serial.write((uint8_t)(yaw >> 8)); Serial.write((uint8_t)(yaw & 0xFF));
//        Serial.write((uint8_t)(pitch >> 8)); Serial.write((uint8_t)(pitch & 0xFF));
//        Serial.write((uint8_t)(roll >> 8)); Serial.write((uint8_t)(roll & 0xFF));
//        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
//        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
//        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
//
//
//
//        Serial.write((uint8_t)(constrain(map(analogRead(flpin[0]), 610, 450, 0, 255), 0, 255)));
//        Serial.write((uint8_t)(constrain(map(analogRead(flpin[1]), 660, 500, 0, 255), 0, 255)));
//        Serial.write((uint8_t)(constrain(map(analogRead(flpin[2]), 650, 490, 0, 255), 0, 255)));
//        Serial.write((uint8_t)(constrain(map(analogRead(flpin[3]), 630, 470, 0, 255), 0, 255)));
//        Serial.write((uint8_t)(constrain(map(analogRead(flpin[4]), 640, 480, 0, 255), 0, 255)));
//    
//        for(int i = 0; i<5; i++){
//            Serial.write((uint8_t)(map(analogRead(prespin[i]), 685, 20, 0, 255)));
//        }
//        Serial.write("\n");

// Debug print
        Serial.println();
        Serial.print(mympu.ypr[0]);
        Serial.print("  ");
        Serial.print(mympu.ypr[1] );
        Serial.print("  ");
        Serial.print(mympu.ypr[2]);
        Serial.print("  ");
        Serial.print(mympu.gyro[0]);
        Serial.print("  ");
        Serial.print(mympu.gyro[1]);
        Serial.print("  ");
        Serial.print(mympu.gyro[2]);
        Serial.print("  ");
        
        for(int i = 0; i<5; i++){
            Serial.print(analogRead(flpin[i]));
            Serial.print("  ");
        }
        for(int i = 0; i<5; i++){
            Serial.print(analogRead(prespin[i]));
            Serial.print("  ");
        }
        

    // blink LED to indicate activity
//    blinkState = !blinkState;
//    digitalWrite(LED_PIN, blinkState);
    delay(1);
  }
