#include <Wire.h>


byte address = 0x21;

byte A[2] = {0, 0};
byte x = 0x10;

void setup() {
  Wire.begin();

  Serial.begin(9600);
  while (!Serial); // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");


  A[0] = 0x03;
  A[1] = 0x01;  

  delay(1000);

  Wire.beginTransmission(address);
  Wire.write(A,2);
  byte error = Wire.endTransmission(1);

  if (error == 0) {
    Serial.println("Config Success");;
  } else {
    Serial.println("Fail");
  }
}

void loop() {
    

    A[0] = 0x01;
    A[1] = ~x;

    x = x << 1;
    
    Wire.beginTransmission(address);
    Wire.write(A,2);
    byte error = Wire.endTransmission(1);

    if (error == 0) {
      Serial.println("Write Success");
    } else {
      Serial.println("Fail");
    }

    // Read
    Wire.beginTransmission(address);
    Wire.write(0);
    error = Wire.endTransmission(0);

    if (error == 0) {
      Wire.requestFrom(address, 1);
      char c = Wire.read();
      error = Wire.endTransmission(1);

      if (error == 0) {
        Serial.print("Initiate Read ");
        Serial.println(c, HEX);
      } else {
        Serial.println("Fail");
      }
      
    } else {
      Serial.println("Fail");
    }


  delay(5000); // Wait 5 seconds for next scan
}