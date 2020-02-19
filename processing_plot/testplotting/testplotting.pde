

// Graphing sketch


// This program takes ASCII-encoded strings
// from the serial port at 9600 baud and graphs them. It expects values in the
// range 0 to 1023, followed by a newline, or newline and carriage return
// Created 19 Jan 2016
// Updated 20 Jan 2016
// by Sarath Babu
// This example code is in the public domain.

import processing.serial.*;

Serial myPort;        // The serial port
float inByte0 = 0;
float inByte1=0;


int arrL = 140;
int[] winArr = {5, 2}; //Array of windows (10 sensors)

float a0y_arr[] = new float[arrL];
float a1y_arr[] = new float[arrL];

int a0x_arr[] = new int[arrL];
int a1x_arr[] = new int[arrL];
int idxPos = 0;


int wL_x = 1000;
int wL_y = 400;
int swL_x = wL_x/winArr[0];
int swL_y = wL_y/winArr[1];

void setup () {
  // set the window size:

  size(1000, 400);

  for (int i = 0; i < arrL; i++) { 
    a0y_arr[i] = 0;
    a1y_arr[i] = 0;

    a0x_arr[i] = i;
    a1x_arr[i] = swL_x+i;
  }

  // List all the available serial ports
  // if using Processing 2.1 or later, use Serial.printArray()
  println(Serial.list());

  //From the available ports, select the port to which your arduino is connected
  //If the connected port is the first port  then the code is Serial.list()[0]
  //Since I am using the second port the code is written as Serial.list()[1]
  // Open whatever port is the one you're using.
  myPort = new Serial(this, Serial.list()[1], 9600);

  // don't generate a serialEvent() unless you get a newline character:
  myPort.bufferUntil('\n');

  // set inital background:
  background(0);
  //Create the first rectangle


  for (int i = 0; i<winArr[0]; i++) {
    for (int j = 0; j<winArr[1]; j++) {
      fill(150);
      rect(i * swL_x, j * swL_y, swL_x, swL_y);
    }
  }
}


void draw () {
  // draw the line of the first sensor:


  addSens2Arr(a0y_arr, inByte0, idxPos);
  addSens2Arr(a1y_arr, inByte1, idxPos);

  idxPos = (idxPos + 1) % arrL;

  UpdatePlot();

  DrawLine(a0y_arr, a0x_arr, arrL, idxPos);
  DrawLine(a1y_arr, a1x_arr, arrL, idxPos);
}

void UpdatePlot() {
  for (int i = 0; i<winArr[0]; i++) {
    for (int j = 0; j<winArr[1]; j++) {
      fill(150);
      rect(i * swL_x, j * swL_y, swL_x, swL_y);
    }
  }
}

void addSens2Arr(float y_arr[], float inByte, int idxpos) {
  y_arr[idxpos] = inByte;
}

void DrawLine(float y_arr[], int x_arr[], int arrL, int idxpos) {
  stroke(0, 0, 200);
  for (int i = 0; i<arrL-1; i++) {
    int pos1 = (idxpos + i) % arrL;
    int pos2 = (idxpos + i+1) % arrL;
    //println(x_arr[i], y_arr[pos1], x_arr[i+1], y_arr[pos2]);
    line(x_arr[i], 400 - y_arr[pos1], x_arr[i+1], 400 - y_arr[pos2]);
  }
}

void serialEvent (Serial myPort) {
  // get the ASCII string:
  String inString = myPort.readStringUntil('\n');
  if (inString != null) {
    // trim off any whitespace:
    inString = trim(inString);
    //Split the string value dot as delimiter
    float[] nums=float(split(inString, "."));
    //nums[0]=first sensor value
    //println("First sensor value="+nums[0]);
    //nums[1]=second sensor value
    //println("Second Sensor value="+nums[1]);
    // convert to an int and map to the screen height:
    inByte0 = float(inString);

    //map the first sensor value
    inByte0 = map(nums[0], 400, 700, 0, 400);
    //map the second sensor value
    inByte1=map(nums[1], 300, 700, 0, 400);
    //println(inByte1);
    //println(inByte0);
  }
}
