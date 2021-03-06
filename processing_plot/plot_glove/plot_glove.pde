

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
Serial myPort2;

int ax = 0;
int ay = 0;
int az = 0;
int gx = 0;
int gy = 0;
int gz = 0;

int p1 = 0;
int p2 = 0; 
int p3 = 0;
int p4 = 0;
int p5 = 0;
int f1 = 0;
int f2 = 0;
int f3 = 0;
int f4 = 0;
int f5 = 0;

int p1_2 = 0;
int p2_2 = 0;
int p3_2 = 0;
int p4_2 = 0;
int p5_2 = 0;
int f1_2 = 0;
int f2_2 = 0;
int f3_2 = 0;
int f4_2 = 0;
int f5_2 = 0;

int arrL = 140;
int[] winArr = {10, 2}; //Array of windows (10 sensors)

int p1y_arr[] = new int[arrL];
int p2y_arr[] = new int[arrL];
int p3y_arr[] = new int[arrL];
int p4y_arr[] = new int[arrL];
int p5y_arr[] = new int[arrL];
int p1x_arr[] = new int[arrL];
int p2x_arr[] = new int[arrL];
int p3x_arr[] = new int[arrL];
int p4x_arr[] = new int[arrL];
int p5x_arr[] = new int[arrL];

int p1y_2arr[] = new int[arrL];
int p2y_2arr[] = new int[arrL];
int p3y_2arr[] = new int[arrL];
int p4y_2arr[] = new int[arrL];
int p5y_2arr[] = new int[arrL];
int p1x_2arr[] = new int[arrL];
int p2x_2arr[] = new int[arrL];
int p3x_2arr[] = new int[arrL];
int p4x_2arr[] = new int[arrL];
int p5x_2arr[] = new int[arrL];

int f1y_arr[] = new int[arrL];
int f2y_arr[] = new int[arrL];
int f3y_arr[] = new int[arrL];
int f4y_arr[] = new int[arrL];
int f5y_arr[] = new int[arrL];
int f1x_arr[] = new int[arrL];
int f2x_arr[] = new int[arrL];
int f3x_arr[] = new int[arrL];
int f4x_arr[] = new int[arrL];
int f5x_arr[] = new int[arrL];

int f1y_2arr[] = new int[arrL];
int f2y_2arr[] = new int[arrL];
int f3y_2arr[] = new int[arrL];
int f4y_2arr[] = new int[arrL];
int f5y_2arr[] = new int[arrL];
int f1x_2arr[] = new int[arrL];
int f2x_2arr[] = new int[arrL];
int f3x_2arr[] = new int[arrL];
int f4x_2arr[] = new int[arrL];
int f5x_2arr[] = new int[arrL];

int idxPos = 0;


int wL_x =1800;
int wL_y = 510;
int swL_x = wL_x/winArr[0];
int swL_y = wL_y/winArr[1];

void setup () {
  // set the window size:

  size(1800, 510);

  for (int i = 0; i < arrL; i++) { 
    p1y_arr[i] = 0;
    p2y_arr[i] = 0;
    p3y_arr[i] = 0;
    p4y_arr[i] = 0;
    p5y_arr[i] = 0;
    
    p1y_2arr[i] = 0;
    p2y_2arr[i] = 0;
    p3y_2arr[i] = 0;
    p4y_2arr[i] = 0;
    p5y_2arr[i] = 0;
    
    f1y_arr[i] = 0;
    f2y_arr[i] = 0;
    f3y_arr[i] = 0;
    f4y_arr[i] = 0;
    f5y_arr[i] = 0;
    
    f1y_2arr[i] = 0;
    f2y_2arr[i] = 0;
    f3y_2arr[i] = 0;
    f4y_2arr[i] = 0;
    f5y_2arr[i] = 0;
    
    // Makes more sense to put them on top of each other
    p1x_arr[i] = i;
    p2x_arr[i] = swL_x + i;
    p3x_arr[i] = 2*swL_x + i;
    p4x_arr[i] = 3*swL_x + i;
    p5x_arr[i] = 4*swL_x + i;
    
    f1x_arr[i] = 5*swL_x + i;
    f2x_arr[i] = 6*swL_x + i;
    f3x_arr[i] = 7*swL_x + i;
    f4x_arr[i] = 8*swL_x + i;
    f5x_arr[i] = 9*swL_x + i;
   
    p1x_2arr[i] = i;
    p2x_2arr[i] = swL_x + i;
    p3x_2arr[i] = 2*swL_x + i;
    p4x_2arr[i] = 3*swL_x + i;
    p5x_2arr[i] = 4*swL_x + i;
    
    f1x_2arr[i] = 5*swL_x + i;
    f2x_2arr[i] = 6*swL_x + i;
    f3x_2arr[i] = 7*swL_x + i;
    f4x_2arr[i] = 8*swL_x + i;
    f5x_2arr[i] = 9*swL_x + i;
    
  }

  // List all the available serial ports
  // if using Processing 2.1 or later, use Serial.printArray()
  println(Serial.list());

  //From the available ports, select the port to which your arduino is connected
  //If the connected port is the first port  then the code is Serial.list()[0]
  //Since I am using the second port the code is written as Serial.list()[1]
  // Open whatever port is the one you're using.
  myPort = new Serial(this, Serial.list()[1], 115200);
  myPort2 = new Serial(this, Serial.list()[2], 115200);
  // don't generate a serialEvent() unless you get a newline character:
  myPort.bufferUntil(10); 
  myPort2.bufferUntil(10); 
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


  addSens2Arr(p1y_arr, p1, idxPos);
  addSens2Arr(p2y_arr, p2, idxPos);
  addSens2Arr(p3y_arr, p3, idxPos);
  addSens2Arr(p4y_arr, p4, idxPos);
  addSens2Arr(p5y_arr, p5, idxPos);

  addSens2Arr(p1y_2arr, p1_2, idxPos);
  addSens2Arr(p2y_2arr, p2_2, idxPos);
  addSens2Arr(p3y_2arr, p3_2, idxPos);
  addSens2Arr(p4y_2arr, p4_2, idxPos);
  addSens2Arr(p5y_2arr, p5_2, idxPos);

  addSens2Arr(f1y_arr, f1, idxPos);
  addSens2Arr(f2y_arr, f2, idxPos);
  addSens2Arr(f3y_arr, f3, idxPos);
  addSens2Arr(f4y_arr, f4, idxPos);
  addSens2Arr(f5y_arr, f5, idxPos);

  addSens2Arr(f1y_2arr, f1_2, idxPos);
  addSens2Arr(f2y_2arr, f2_2, idxPos);
  addSens2Arr(f3y_2arr, f3_2, idxPos);
  addSens2Arr(f4y_2arr, f4_2, idxPos);
  addSens2Arr(f5y_2arr, f5_2, idxPos);


  idxPos = (idxPos + 1) % arrL;
  UpdatePlot();

  DrawLine(p1y_arr, p1x_arr, arrL, idxPos, 0);
  DrawLine(p2y_arr, p2x_arr, arrL, idxPos, 0);
  DrawLine(p3y_arr, p3x_arr, arrL, idxPos, 0);
  DrawLine(p4y_arr, p4x_arr, arrL, idxPos, 0);
  DrawLine(p5y_arr, p5x_arr, arrL, idxPos, 0);
  
  DrawLine(p1y_2arr, p1x_2arr, arrL, idxPos, 1);
  DrawLine(p2y_2arr, p2x_2arr, arrL, idxPos, 1);
  DrawLine(p3y_2arr, p3x_2arr, arrL, idxPos, 1);
  DrawLine(p4y_2arr, p4x_2arr, arrL, idxPos, 1);
  DrawLine(p5y_2arr, p5x_2arr, arrL, idxPos, 1);
  
  DrawLine(f1y_arr, f1x_arr, arrL, idxPos, 0);
  DrawLine(f2y_arr, f2x_arr, arrL, idxPos, 0);
  DrawLine(f3y_arr, f3x_arr, arrL, idxPos, 0);
  DrawLine(f4y_arr, f4x_arr, arrL, idxPos, 0);
  DrawLine(f5y_arr, f5x_arr, arrL, idxPos, 0);
  
  DrawLine(f1y_2arr, f1x_2arr, arrL, idxPos, 1);
  DrawLine(f2y_2arr, f2x_2arr, arrL, idxPos, 1);
  DrawLine(f3y_2arr, f3x_2arr, arrL, idxPos, 1);
  DrawLine(f4y_2arr, f4x_2arr, arrL, idxPos, 1);
  DrawLine(f5y_2arr, f5x_2arr, arrL, idxPos, 1);
}

void UpdatePlot() {
  for (int i = 0; i<winArr[0]; i++) {
    for (int j = 0; j<winArr[1]; j++) {
      fill(150);
      rect(i * swL_x, j * swL_y, swL_x, swL_y);
    }
  }
}

void addSens2Arr(int y_arr[], int inByte, int idxpos) {
  y_arr[idxpos] = inByte;
}

void DrawLine(int y_arr[], int x_arr[], int arrL, int idxpos, int row) {
  stroke(0, 250, 250);
  for (int i = 0; i<arrL-1; i++) {
    int pos1 = (idxpos + i) % arrL;
    int pos2 = (idxpos + i+1) % arrL;
    //println(x_arr[i], y_arr[pos1], x_arr[i+1], y_arr[pos2]);
    if(row ==0){
      line(x_arr[i],  y_arr[pos1], x_arr[i+1],  y_arr[pos2]);
    }else if(row==1){
      line(x_arr[i],  y_arr[pos1] + swL_y, x_arr[i+1],  y_arr[pos2] + swL_y);
    }else if(row==2){
      line(x_arr[i],  y_arr[pos1] + 2*swL_y, x_arr[i+1],  y_arr[pos2] + 2*swL_y);
    }
    
  }
}

void serialEvent (Serial ThisPort) {
  // get the ASCII string:
  
  byte[] inBuffer = new byte[24];
  byte[] inBuffer2 = new byte[24];
  
  if(ThisPort == myPort){
    int ss = ThisPort.readBytes(inBuffer);
    
    //println("buffer1: ", inBuffer);
    if (inBuffer != null) { 
      //print("Imin");
      String myString = new String(inBuffer);
      
      //println(inBuffer.length);
      //String cs = str(inBuffer[0]);
      
      byte cs = inBuffer[0];
      if(cs == 119){
      
      //ax = (int)inBuffer[1];
      //ay = (int)inBuffer[3];
      //az = (int)inBuffer[5];
      //gx = (int)inBuffer[7];
      //gy = (int)inBuffer[9];
      //gz = (int)inBuffer[11];
      
      //byte[] temp = {inBuffer[1], inBuffer[2]};
      //int pitch = int(inBuffer[2]);
      //int roll = int(inBuffer[3]);
      f1 = int(inBuffer[13]);
      f2 = int(inBuffer[14]);
      f3 = int(inBuffer[15]);
      f4 = int(inBuffer[16]);
      f5 = int(inBuffer[17]);
      p1 = int(inBuffer[18]);
      p2 = int(inBuffer[19]);
      p3 = int(inBuffer[20]);
      p4 = int(inBuffer[21]);
      p5 = int(inBuffer[22]);
      println(cs, p1, ", ", p2, ", ", p3, ", ", p4, ", ", p5, " || ", f1, ", ", f2, ", ", f3, ", ", f4, ", ", f5," || ", ax, ", ", ay, ", ", az, ", ", gx, ", ", gy, ", ", gz);  
    }
     

      
    }
  }
    
    if(ThisPort == myPort2){
      int ss2 = ThisPort.readBytes(inBuffer2);
     
    //println("buffer2: ", inBuffer2);
    if (inBuffer2 != null) { 
      //print("Imin");
      String myString2 = new String(inBuffer2);
      
      //println(inBuffer.length);
      String cs2 = str(inBuffer2[0]);
      println(inBuffer.length())
      if(inBuffer2[0] == 119){
      //byte[] temp = {inBuffer[1], inBuffer[2]};
      //int pitch = int(inBuffer[2]);
      //int roll = int(inBuffer[3]);
      f1_2 = int(inBuffer2[13]);
      f2_2 = int(inBuffer2[14]);
      f3_2 = int(inBuffer2[15]);
      f4_2 = int(inBuffer2[16]);
      f5_2 = int(inBuffer2[17]);
      p1_2 = int(inBuffer2[18]);
      p2_2 = int(inBuffer2[19]);
      p3_2 = int(inBuffer2[20]);
      p4_2 = int(inBuffer2[21]);
      p5_2 = int(inBuffer2[22]);
      }
      //println(cs2, p1_2, ", ", p2_2, ", ", p3_2, ", ", p4_2, ", ", p5_2, " || ", f1_2, ", ", f2_2, ", ", f3_2, ", ", f4_2, ", ", f5_2)
    }
    }
  }
