#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include <Wire.h>
#include "rgb_lcd.h"

rgb_lcd lcd;

//Create a instance of class LSM6DS3
LSM6DS3 myIMU( I2C_MODE, 0x6A );  //I2C device address 0x6A

void setup() {
  lcd.begin(16, 2); // Démarrage du LCD
  Serial.begin(9600);  
  
  if( myIMU.begin() != 0 )
  {
	  Serial.println("Device error");
  }
  else  
  {
      Serial.println("Device OK!");
  }
}

void loop()
{
  //Accelerometer
  Serial.print("\nAccelerometer:\n");
  Serial.print(" X1 = ");
  Serial.println(myIMU.readFloatAccelX(), 4);
  Serial.print(" Y1 = ");
  Serial.println(myIMU.readFloatAccelY(), 4);
  Serial.print(" Z1 = ");
  Serial.println(myIMU.readFloatAccelZ(), 4);
  lcd.clear();
}


/*
  lcd.print(myIMU.readFloatGyroX());
  lcd.print(myIMU.readFloatGyroY());
  lcd.print(myIMU.readFloatGyroZ());
  */
