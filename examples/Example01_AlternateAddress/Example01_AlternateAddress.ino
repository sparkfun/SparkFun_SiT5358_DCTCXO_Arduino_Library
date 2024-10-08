/*
  Using alternate I2C addresses for the SiT5358 DCTCXO.

  This example shows how to use an alternate address and TwoWire port for the DCTCXO.

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

  bool begun;
  begun = myTCXO.begin(Wire, 0x60); // Initialize the SiT5358 - using a custom bus and address
  begun = myTCXO.begin(0x60); // This is also possible. It defaults to Wire
  begun = myTCXO.begin(); // This is also possible. It defaults to Wire and address 0x60

  if (!begun)
  {
    Serial.println("SiT5358 not detected! Please check the address and try again...");
    while (1); // Do nothing more
  }

  // Read the frequency control word - should be zero initially
  int32_t fcw = myTCXO.getFrequencyControlWord();
  Serial.print("The frequency control word is: ");
  Serial.println(fcw);

  // Read the pull range control
  uint8_t prc = myTCXO.getPullRangeControl();
  Serial.print("Pull range control is: ");
  Serial.println(myTCXO.getPullRangeControlText(prc));
}

void loop()
{
  // Nothing to do here
}
