/*
  Set the frequency of the SiT5358 DCTCXO.

  By: Paul Clark
  SparkFun Electronics
  Date: 2024/8/1
  SparkFun code, firmware, and software is released under the MIT License.
  Please see LICENSE.md for further details.

*/

// You will need the SparkFun Toolkit. Click here to get it: http://librarymanager/All#SparkFun_Toolkit

#include <SparkFun_SiT5358.h> // Click here to get the library: http://librarymanager/All#SparkFun_SiT5358

SfeSiT5358ArdI2C myTCXO;

void setup()
{
  delay(1000); // Allow time for the microcontroller to start up

  Serial.begin(115200); // Begin the Serial console
  while (!Serial)
  {
    delay(100); // Wait for the user to open the Serial Monitor
  }
  Serial.println("SparkFun SiT5358 Example");

  Wire.begin(); // Begin the I2C bus

  if (!myTCXO.begin())
  {
    Serial.println("SiT5358 not detected! Please check the address and try again...");
    while (1); // Do nothing more
  }

  myTCXO.setBaseFrequencyHz(10000000.0); // Pass the oscillator base frequency into the driver

  Serial.print("Base frequency set to ");
  Serial.println(myTCXO.getBaseFrequencyHz());
  Serial.println(" Hz");

  myTCXO.setPullRangeControl(SiT5358_PULL_RANGE_200ppm); // Set the pull range control to 200ppm

  Serial.print("Pull range control set to ");
  Serial.println(myTCXO.getPullRangeControlText(myTCXO.getPullRangeControl()));

  myTCXO.setFrequencyHz(10001000.0); // Set the frequency to 10.001MHz (+100ppm)

  Serial.print("Frequency set to ");
  Serial.print(myTCXO.getFrequencyHz());
  Serial.println(" Hz");

  Serial.print("Frequency control word should be 16777216. It is ");
  Serial.println(myTCXO.getFrequencyControlWord());
}

void loop()
{
  // Nothing to do here
}
