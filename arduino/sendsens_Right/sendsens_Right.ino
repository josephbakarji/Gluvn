
#include "I2Cdev.h"
#include  <SoftwareSerial.h>
#include "freeram.h"
#include "mpu.h"

int ret;
unsigned int gx, gy, gz;
unsigned int yaw, pitch, roll;

#define LED_PIN 13
bool blinkState = false;
bool startSerial = false;
void setup() {    
        Fastwire::setup(400, true);
        ret = mympu_open(200);
        Serial.begin(115200);
        pinMode(LED_PIN, OUTPUT);
}

int prespin[] = {0, 1, 2, 3, 4};
int flpin[] = {5, 8, 9, 10, 11};

void loop() {

     ret = mympu_update();

     Serial.write("w");

     yaw = (uint16_t) (mympu.ypr[0] * 32767.0/180.0) ;
     pitch = (uint16_t) (mympu.ypr[1] * 32767.0/90.0) ;
     roll = (uint16_t) (mympu.ypr[2] * 32767.0/180.0) ;
     gx = (uint16_t) (constrain( mympu.gyro[0] * 32767.0/180.0,  0, 65535.0 ) ) ;
     gy = (uint16_t) (constrain( mympu.gyro[1] * 32767.0/180.0,  0, 65535.0 ) );
     gz = (uint16_t) (constrain( mympu.gyro[2] * 32767.0/180.0,  0, 65535.0 ) );
     

     
        Serial.write((uint8_t)(yaw >> 8)); Serial.write((uint8_t)(yaw & 255));
        Serial.write((uint8_t)(pitch >> 8)); Serial.write((uint8_t)(pitch & 255));
        Serial.write((uint8_t)(roll >> 8)); Serial.write((uint8_t)(roll & 255));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 255));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 255));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 255));


        Serial.write((uint8_t)(constrain(map(analogRead(flpin[0]), 516, 616, 0, 255), 0, 255)));
        Serial.write((uint8_t)(constrain(map(analogRead(flpin[1]), 506, 610, 0, 255), 0, 255)));
        Serial.write((uint8_t)(constrain(map(analogRead(flpin[2]), 536, 630, 0, 255), 0, 255)));
        Serial.write((uint8_t)(constrain(map(analogRead(flpin[3]), 505, 610, 0, 255), 0, 255)));
        Serial.write((uint8_t)(constrain(map(analogRead(flpin[4]), 503, 610, 0, 255), 0, 255)));
    
        for(int i = 0; i<5; i++){
            Serial.write((uint8_t)(constrain(map(analogRead(prespin[i]), 675, 300, 0, 255), 0, 255)));
        }
        Serial.write("\n");

// Debug print

//Serial.print(mympu.ypr[0]);
//Serial.print("\t");
//Serial.print(mympu.ypr[1]);
//Serial.print("\t");
//Serial.print(mympu.ypr[2]);
//Serial.print("\t");
//Serial.print(mympu.gyro[0]);
//Serial.print("\t");
//Serial.print(mympu.gyro[1]);
//Serial.print("\t");
//Serial.print(mympu.gyro[2]);
//Serial.print("\t");
//
//for(int i = 0; i<5; i++){
//  Serial.print("\t");
//    Serial.print(analogRead(flpin[i]));
//}
//for(int i = 0; i<5; i++){
//  Serial.print("\t");
//    Serial.print(analogRead(prespin[i]));
//}
//Serial.print("\n");

    // blink LED to indicate activity
//    blinkState = !blinkState;
//    digitalWrite(LED_PIN, blinkState);
    delay(1);
  }
