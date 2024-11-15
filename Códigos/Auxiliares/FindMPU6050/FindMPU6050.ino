#include <Wire.h>

byte Error, Address;
int QuantityDevices;

void setup(){
  Wire.begin();

  Serial.begin(9600);
  Serial.println("\nI2C Scanner");
}

void loop(){
  Serial.println("Scanning...");

  QuantityDevices = 0;
  for(Address = 1; Address < 127; Address++ ) {
    // The i2c_scanner uses the return value of the Write.endTransmisstion to see if a device did acknowledge to the Address.
    Wire.beginTransmission(Address);
    Error = Wire.endTransmission();

    if (Error == 0) {
      Serial.print("I2C device found at Address 0x");
      Serial.printf("%02X", Address);
      Serial.print("!");

      QuantityDevices++;
    } else if (Error==4)  {
      Serial.print("Unknow Error at Address 0x");
      Serial.printf("%02X", Address);
    }
    Serial.println();
  }

  QuantityDevices == 0 ? Serial.println("No I2C devices found\n") : Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}