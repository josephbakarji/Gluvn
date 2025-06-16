#include "I2Cdev.h"
#include  <SoftwareSerial.h>
#include "freeram.h"
#include "mpu.h"
#include "inv_mpu.h"
#include "math.h"
#include "calibration.h"
#include <EEPROM.h>

#define NUM_SENSORS 5

// EEPROM Memory Layout
#define EEPROM_MAGIC_ADDR 0      // Magic number to check if EEPROM is initialized
#define EEPROM_MAGIC_VALUE 0xCAFE // Magic value indicating valid calibration data

// EEPROM addresses for calibration data
#define EEPROM_L_MIN_FLEX_ADDR 2   // 5 ints = 10 bytes
#define EEPROM_L_MAX_FLEX_ADDR 12  // 5 ints = 10 bytes  
#define EEPROM_L_MIN_PRESS_ADDR 22 // 5 ints = 10 bytes
#define EEPROM_L_MAX_PRESS_ADDR 32 // 5 ints = 10 bytes

#define EEPROM_R_MIN_FLEX_ADDR 42  // 5 ints = 10 bytes
#define EEPROM_R_MAX_FLEX_ADDR 52  // 5 ints = 10 bytes
#define EEPROM_R_MIN_PRESS_ADDR 62 // 5 ints = 10 bytes
#define EEPROM_R_MAX_PRESS_ADDR 72 // 5 ints = 10 bytes

int ret;

char hand = 'r'; // Choose either 'r' or 'l'
int use_gyro = false;
bool calibration_mode = false; // Can be controlled via serial commands
bool write_serial = true; 

// print_mode = 0 -> doesn't print
// print_mode = 1 -> prints raw values
// print_mode = 2 -> prints calibrated values
int print_mode = 0;

const long interval = 12;  // Interval at which to read sensors and send data (milliseconds)

int gyro_max = 2000; // Range is +/- 2000 deg/s by default 
int accel_max = 32767.0; // Range is +/- 2g by default

int prespin[] = {0, 1, 2, 3, 4};
int flpin[] = {5, 8, 9, 10, 11};

uint16_t flexValues[5];
uint16_t pressValues[5];
int gx, gy, gz, roll, pitch, yaw, ax, ay, az;
uint16_t gx_cal, gy_cal, gz_cal, roll_cal, pitch_cal, yaw_cal, ax_cal, ay_cal, az_cal;

unsigned long previousMillis = 0;

// Serial command buffer
String serialCommand = "";
bool commandComplete = false;

// Dynamic calibration arrays stored in RAM
int L_MIN_FLEX_RAM[5], L_MAX_FLEX_RAM[5], L_MIN_PRESS_RAM[5], L_MAX_PRESS_RAM[5];
int R_MIN_FLEX_RAM[5], R_MAX_FLEX_RAM[5], R_MIN_PRESS_RAM[5], R_MAX_PRESS_RAM[5];

// Pointers to current hand's calibration data
int* MIN_FLEX;
int* MAX_FLEX;
int* MIN_PRESS;
int* MAX_PRESS;

// EEPROM Helper Functions
void writeIntArrayToEEPROM(int address, int* array, int length) {
  for (int i = 0; i < length; i++) {
    EEPROM.write(address + i*2, array[i] & 0xFF);        // Low byte
    EEPROM.write(address + i*2 + 1, (array[i] >> 8) & 0xFF); // High byte
  }
}

void readIntArrayFromEEPROM(int address, int* array, int length) {
  for (int i = 0; i < length; i++) {
    int lowByte = EEPROM.read(address + i*2);
    int highByte = EEPROM.read(address + i*2 + 1);
    array[i] = (highByte << 8) | lowByte;
  }
}

void loadCalibrationFromEEPROM() {
  // Check if EEPROM has valid calibration data
  int magic = (EEPROM.read(EEPROM_MAGIC_ADDR + 1) << 8) | EEPROM.read(EEPROM_MAGIC_ADDR);
  
  if (magic == EEPROM_MAGIC_VALUE) {
    // Load calibration data from EEPROM
    readIntArrayFromEEPROM(EEPROM_L_MIN_FLEX_ADDR, L_MIN_FLEX_RAM, 5);
    readIntArrayFromEEPROM(EEPROM_L_MAX_FLEX_ADDR, L_MAX_FLEX_RAM, 5);
    readIntArrayFromEEPROM(EEPROM_L_MIN_PRESS_ADDR, L_MIN_PRESS_RAM, 5);
    readIntArrayFromEEPROM(EEPROM_L_MAX_PRESS_ADDR, L_MAX_PRESS_RAM, 5);
    
    readIntArrayFromEEPROM(EEPROM_R_MIN_FLEX_ADDR, R_MIN_FLEX_RAM, 5);
    readIntArrayFromEEPROM(EEPROM_R_MAX_FLEX_ADDR, R_MAX_FLEX_RAM, 5);
    readIntArrayFromEEPROM(EEPROM_R_MIN_PRESS_ADDR, R_MIN_PRESS_RAM, 5);
    readIntArrayFromEEPROM(EEPROM_R_MAX_PRESS_ADDR, R_MAX_PRESS_RAM, 5);
    
    Serial.println("EEPROM_CALIBRATION_LOADED");
  } else {
    // Use default values if no calibration in EEPROM
    setDefaultCalibration();
    Serial.println("EEPROM_NO_CALIBRATION_USING_DEFAULTS");
  }
}

void setDefaultCalibration() {
  // Load default calibration values from calibration.h
  // This provides fallback values if EEPROM is empty
  
  // Copy LEFT hand values from calibration.h
  for (int i = 0; i < 5; i++) {
    L_MIN_FLEX_RAM[i] = L_MIN_FLEX[i];
    L_MAX_FLEX_RAM[i] = L_MAX_FLEX[i];
    L_MIN_PRESS_RAM[i] = L_MIN_PRESS[i];
    L_MAX_PRESS_RAM[i] = L_MAX_PRESS[i];
  }
  
  // Copy RIGHT hand values from calibration.h
  for (int i = 0; i < 5; i++) {
    R_MIN_FLEX_RAM[i] = R_MIN_FLEX[i];
    R_MAX_FLEX_RAM[i] = R_MAX_FLEX[i];
    R_MIN_PRESS_RAM[i] = R_MIN_PRESS[i];
    R_MAX_PRESS_RAM[i] = R_MAX_PRESS[i];
  }
}

void saveCalibrationToEEPROM() {
  // Write magic number
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE & 0xFF);
  EEPROM.write(EEPROM_MAGIC_ADDR + 1, (EEPROM_MAGIC_VALUE >> 8) & 0xFF);
  
  // Write calibration data
  writeIntArrayToEEPROM(EEPROM_L_MIN_FLEX_ADDR, L_MIN_FLEX_RAM, 5);
  writeIntArrayToEEPROM(EEPROM_L_MAX_FLEX_ADDR, L_MAX_FLEX_RAM, 5);
  writeIntArrayToEEPROM(EEPROM_L_MIN_PRESS_ADDR, L_MIN_PRESS_RAM, 5);
  writeIntArrayToEEPROM(EEPROM_L_MAX_PRESS_ADDR, L_MAX_PRESS_RAM, 5);
  
  writeIntArrayToEEPROM(EEPROM_R_MIN_FLEX_ADDR, R_MIN_FLEX_RAM, 5);
  writeIntArrayToEEPROM(EEPROM_R_MAX_FLEX_ADDR, R_MAX_FLEX_RAM, 5);
  writeIntArrayToEEPROM(EEPROM_R_MIN_PRESS_ADDR, R_MIN_PRESS_RAM, 5);
  writeIntArrayToEEPROM(EEPROM_R_MAX_PRESS_ADDR, R_MAX_PRESS_RAM, 5);
  
  Serial.println("EEPROM_CALIBRATION_SAVED");
}

void setHandPointers() {
  if(hand == 'r'){
    MIN_FLEX = R_MIN_FLEX_RAM;
    MAX_FLEX = R_MAX_FLEX_RAM;
    MIN_PRESS = R_MIN_PRESS_RAM;
    MAX_PRESS = R_MAX_PRESS_RAM;
  } else {
    MIN_FLEX = L_MIN_FLEX_RAM;
    MAX_FLEX = L_MAX_FLEX_RAM;
    MIN_PRESS = L_MIN_PRESS_RAM;
    MAX_PRESS = L_MAX_PRESS_RAM;
  }
}

// Function to process serial commands
void processSerialCommand(String command) {
  command.trim();
  command.toUpperCase();
  
  if (command == "CAL_START") {
    calibration_mode = true;
    Serial.println("CALIBRATION_MODE_ON");
  }
  else if (command == "CAL_STOP") {
    calibration_mode = false;
    Serial.println("CALIBRATION_MODE_OFF");
  }
  else if (command == "RAW_MODE") {
    calibration_mode = true;
    print_mode = 1;
    write_serial = false;
    Serial.println("RAW_PRINT_MODE_ON");
  }
  else if (command == "NORMAL_MODE") {
    calibration_mode = false;
    print_mode = 0;
    write_serial = true;
    Serial.println("NORMAL_MODE_ON");
  }
  else if (command == "STATUS") {
    Serial.print("HAND:");
    Serial.print(hand);
    Serial.print(",CAL:");
    Serial.print(calibration_mode ? "ON" : "OFF");
    Serial.print(",PRINT:");
    Serial.print(print_mode);
    Serial.print(",SERIAL:");
    Serial.println(write_serial ? "ON" : "OFF");
  }
  else if (command.startsWith("HAND_")) {
    if (command == "HAND_R") {
      hand = 'r';
      setHandPointers();
      Serial.println("HAND_SET_RIGHT");
    }
    else if (command == "HAND_L") {
      hand = 'l';
      setHandPointers();
      Serial.println("HAND_SET_LEFT");
    }
  }
  else if (command == "EEPROM_SAVE") {
    saveCalibrationToEEPROM();
  }
  else if (command == "EEPROM_LOAD") {
    loadCalibrationFromEEPROM();
    setHandPointers();
  }
  else if (command == "EEPROM_RESET") {
    setDefaultCalibration();
    saveCalibrationToEEPROM();
    setHandPointers();
    Serial.println("EEPROM_RESET_TO_DEFAULTS");
  }
  else if (command.startsWith("SET_CAL:")) {
    // Format: SET_CAL:HAND:TYPE:VALUES
    // Example: SET_CAL:L:MIN_FLEX:200,210,220,230,240
    processCalibrationData(command);
  }
}

void processCalibrationData(String command) {
  // Parse calibration data from serial
  // Format: SET_CAL:HAND:TYPE:VALUES
  int firstColon = command.indexOf(':', 8);  // Skip "SET_CAL:"
  int secondColon = command.indexOf(':', firstColon + 1);
  int thirdColon = command.indexOf(':', secondColon + 1);
  
  if (firstColon == -1 || secondColon == -1 || thirdColon == -1) {
    Serial.println("EEPROM_INVALID_FORMAT");
    return;
  }
  
  String handStr = command.substring(8, firstColon);
  String typeStr = command.substring(firstColon + 1, secondColon);
  String valuesStr = command.substring(secondColon + 1);
  
  // Parse values
  int values[5];
  int valueIndex = 0;
  int startPos = 0;
  
  for (int i = 0; i < 5 && valueIndex < 5; i++) {
    int commaPos = valuesStr.indexOf(',', startPos);
    if (commaPos == -1 && i == 4) {
      // Last value
      values[valueIndex] = valuesStr.substring(startPos).toInt();
    } else if (commaPos != -1) {
      values[valueIndex] = valuesStr.substring(startPos, commaPos).toInt();
      startPos = commaPos + 1;
    } else {
      Serial.println("EEPROM_PARSE_ERROR");
      return;
    }
    valueIndex++;
  }
  
  // Store values in appropriate array
  int* targetArray = nullptr;
  
  if (handStr == "L") {
    if (typeStr == "MIN_FLEX") targetArray = L_MIN_FLEX_RAM;
    else if (typeStr == "MAX_FLEX") targetArray = L_MAX_FLEX_RAM;
    else if (typeStr == "MIN_PRESS") targetArray = L_MIN_PRESS_RAM;
    else if (typeStr == "MAX_PRESS") targetArray = L_MAX_PRESS_RAM;
  } else if (handStr == "R") {
    if (typeStr == "MIN_FLEX") targetArray = R_MIN_FLEX_RAM;
    else if (typeStr == "MAX_FLEX") targetArray = R_MAX_FLEX_RAM;
    else if (typeStr == "MIN_PRESS") targetArray = R_MIN_PRESS_RAM;
    else if (typeStr == "MAX_PRESS") targetArray = R_MAX_PRESS_RAM;
  }
  
  if (targetArray != nullptr) {
    for (int i = 0; i < 5; i++) {
      targetArray[i] = values[i];
    }
    Serial.print("EEPROM_SET_");
    Serial.print(handStr);
    Serial.print("_");
    Serial.println(typeStr);
  } else {
    Serial.println("EEPROM_INVALID_TYPE");
  }
}

// #define LED_PIN 13
void setup() {

        Fastwire::setup(400, true);
        ret = mympu_open(200);
        Serial.begin(115200);
        // pinMode(LED_PIN, OUTPUT);

        // Load calibration from EEPROM
        loadCalibrationFromEEPROM();
        setHandPointers();

        unsigned short gyro_fsr;
        unsigned char accel_fsr;
        mpu_get_gyro_fsr(&gyro_fsr);
        mpu_get_accel_fsr(&accel_fsr);

}


//////////////////////////////

void loop() {

    // Process incoming serial commands
    while (Serial.available()) {
      char inChar = (char)Serial.read();
      if (inChar == '\n') {
        commandComplete = true;
      } else {
        serialCommand += inChar;
      }
    }
    
    // Process complete command
    if (commandComplete) {
      processSerialCommand(serialCommand);
      serialCommand = "";
      commandComplete = false;
    }

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
            Serial.write((uint8_t)(flexValues[i] & 255));
          }
          for(int i = 0; i<NUM_SENSORS; i++){
            Serial.write((uint8_t)(pressValues[i] >> 8)); 
            Serial.write((uint8_t)(pressValues[i] & 255));
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
