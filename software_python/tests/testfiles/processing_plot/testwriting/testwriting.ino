
void setup() {
  // initialize the serial communication:
  Serial.begin(9600);
}

void loop() {
  // store the value of analog input 0 in x:
  int x=analogRead(A1);
  // store the value of analog input 1 in y:
  int y=analogRead(A2);
  Serial.write(x);
  //seperate both the values by dot:
  Serial.write(".");
  Serial.print(y);
  //new line:
  Serial.write("\n");
  // wait a bit for the analog-to-digital converter
  // to stabilize after the last reading:
  delay(1);
}

