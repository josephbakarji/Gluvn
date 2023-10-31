
#include "I2Cdev.h"
#include  <SoftwareSerial.h>
#include "freeram.h"
#include "mpu.h"
#include "inv_mpu.h"
#include "math.h"
#include "calibration.h"


#define NUM_SENSORS 5

int ret;

char* hand = "l"; // Choose either 'r' or 'l'
int use_gyro = false;
bool calibration_mode = true; // only works if write_serial

// print_mode = 0 -> doesn't print
// print_mode = 1 -> prints raw values
// print_mode = 2 -> prints calibrated values
bool write_serial = false; 
int print_mode = 0;

const long interval = 4;  // Interval at which to read sensors and send data (milliseconds)
int gyro_max = 2000; // Range is +/- 2000 deg/s by default 
int accel_max = 32767.0; // Range is +/- 2g by default

int prespin[] = {0, 1, 2, 3, 4};
int flpin[] = {5, 8, 9, 10, 11};

uint16_t flexValues[5];
uint16_t pressValues[5];
int gx, gy, gz, roll, pitch, yaw, ax, ay, az;
uint16_t gx_cal, gy_cal, gz_cal, roll_cal, pitch_cal, yaw_cal, ax_cal, ay_cal, az_cal;

unsigned long previousMillis = 0;


const int* MIN_FLEX;
const int* MAX_FLEX;
const int* MIN_PRESS;
const int* MAX_PRESS;


// #define LED_PIN 13
void setup() {

        Fastwire::setup(400, true);
        ret = mympu_open(200);
        Serial.begin(115200);
        // pinMode(LED_PIN, OUTPUT);

        if(hand == "r"){
          MIN_FLEX = R_MIN_FLEX;
          MAX_FLEX = R_MAX_FLEX;
          MIN_PRESS = R_MIN_PRESS;
          MAX_PRESS = R_MAX_PRESS;
        } else if(hand == "l") {
          MIN_FLEX = L_MIN_FLEX;
          MAX_FLEX = L_MAX_FLEX;
          MIN_PRESS = L_MIN_PRESS;
          MAX_PRESS = L_MAX_PRESS;
        }

        unsigned short gyro_fsr;
        unsigned char accel_fsr;
        mpu_get_gyro_fsr(&gyro_fsr);
        mpu_get_accel_fsr(&accel_fsr);

}


//////////////////////////////

void loop() {


    unsigned long currentMillis = millis();
    
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      ret = mympu_update();
      // TODO: add error handling, check ret.
      

      // Read values
      for (int i = 0; i < 5; i++) {
          flexValues[i] = analogRead(flpin[i]);
          pressValues[i] = analogRead(prespin[i]);
      }
      yaw = mympu.ypr[0];
      pitch = mympu.ypr[1];
      roll = mympu.ypr[2];
      gx = mympu.gyro[0];
      gy = mympu.gyro[1];
      gz = mympu.gyro[2];
      ax = mympu.accel[0];
      ay = mympu.accel[1];
      az = mympu.accel[2];

      yaw_cal = (uint16_t)((yaw+180.0) * 32767.0/180.0);
      pitch_cal = (uint16_t)((pitch+90.0) * 32767.0/90.0);
      roll_cal = (uint16_t)((roll+180.0) * 32767.0/180.0);

      gx_cal = (uint16_t)(constrain( (gx+gyro_max) * 32767.0/gyro_max, 0, 65535.0 ) );
      gy_cal = (uint16_t)(constrain( (gy+gyro_max) * 32767.0/gyro_max, 0, 65535.0 ) );
      gz_cal = (uint16_t)(constrain( (gz+gyro_max) * 32767.0/gyro_max, 0, 65535.0 ) );
      ax_cal = (uint16_t)( (ax+accel_max)  ); // Assumes accel_max = 32767.0;
      ay_cal = (uint16_t)( (ay+accel_max)  );
      az_cal = (uint16_t)( (az+accel_max)  );
    

      if(write_serial) {

        Serial.write(hand);

      // Write to serial
        Serial.write((uint8_t)(yaw_cal >> 8)); Serial.write((uint8_t)(yaw_cal & 255));
        Serial.write((uint8_t)(pitch_cal >> 8)); Serial.write((uint8_t)(pitch_cal & 255));
        Serial.write((uint8_t)(roll_cal >> 8)); Serial.write((uint8_t)(roll_cal & 255));

        if(use_gyro){
          Serial.write((uint8_t)(gx_cal >> 8)); Serial.write((uint8_t)(gx_cal & 255));
          Serial.write((uint8_t)(gy_cal >> 8)); Serial.write((uint8_t)(gy_cal & 255));
          Serial.write((uint8_t)(gz_cal >> 8)); Serial.write((uint8_t)(gz_cal & 255));
        } else {
          Serial.write((uint8_t)(ax_cal >> 8)); Serial.write((uint8_t)(ax_cal & 255));
          Serial.write((uint8_t)(ay_cal >> 8)); Serial.write((uint8_t)(ay_cal & 255));
          Serial.write((uint8_t)(az_cal >> 8)); Serial.write((uint8_t)(az_cal & 255));
        }


        if(!calibration_mode){
          // Send calibrated pressue and flex sensors
          for(int i = 0; i<NUM_SENSORS; i++){
            Serial.write((uint8_t)(constrain(map(flexValues[i], MIN_FLEX[i], MAX_FLEX[i], 0, 255), 0, 255)));
          }
          for(int i = 0; i<NUM_SENSORS; i++){
            Serial.write((uint8_t)(constrain(map(pressValues[i], MIN_PRESS[i], MAX_PRESS[i], 0, 255), 0, 255)));
          }

        } else {
          
          // Send un-calibrated pressure and flex sensors
          for(int i = 0; i<NUM_SENSORS; i++){
            Serial.write((uint8_t)(flexValues[i] >> 8)); 
            Serial.write((uint8_t)(flexValues[i] & 225));
          }
          for(int i = 0; i<NUM_SENSORS; i++){
            Serial.write((uint8_t)(pressValues[i] >> 8)); 
            Serial.write((uint8_t)(pressValues[i] & 225));
          }

        }

        Serial.write("\n");

      }


      // PRINTING
      if(print_mode>0){
        if(print_mode==1){

        // FLEX
        for(int i = 0; i<NUM_SENSORS; i++){
          Serial.print(flexValues[i]); Serial.print("\t");
        }

        // PRESS
        for(int i = 0; i<NUM_SENSORS; i++){
          Serial.print(pressValues[i]); Serial.print("\t");
        }

        // Pitch Roll Yaw
        Serial.print(yaw); Serial.print("\t");
        Serial.print(roll); Serial.print("\t");
        Serial.print(pitch); Serial.print("\t");
        // GYRO
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.print(gz); Serial.print("\t");
        // ACCELEROMETER
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");

        Serial.print("\n");

      } else if(print_mode==2) {

          for(int i = 0; i<NUM_SENSORS; i++){
            Serial.print(constrain(map(flexValues[i], MIN_FLEX[i], MAX_FLEX[i], 0, 255), 0, 255));
            Serial.print("\t");
          }
          for(int i = 0; i<NUM_SENSORS; i++){
            Serial.print(constrain(map(pressValues[i], MIN_PRESS[i], MAX_PRESS[i], 0, 255), 0, 255));
            Serial.print("\t");
          }

          //  Yaw Roll Pitch 
          Serial.print(yaw_cal); Serial.print("\t");
          Serial.print(roll_cal); Serial.print("\t");
          Serial.print(pitch_cal); Serial.print("\t");
          // GYRO
          Serial.print(gx_cal); Serial.print("\t");
          Serial.print(gy_cal); Serial.print("\t");
          Serial.print(gz_cal); Serial.print("\t");
          // ACCELEROMETER
          Serial.print(ax_cal); Serial.print("\t");
          Serial.print(ay_cal); Serial.print("\t");
          Serial.print(az_cal); Serial.print("\t");

          // Serial.print(sq(ax_cal)+sq(ay_cal)+sq(az_cal));

          Serial.print("\n");
      }
      }
    }

  }
