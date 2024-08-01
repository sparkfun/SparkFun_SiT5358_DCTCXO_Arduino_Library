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

#include <SparkFun_ADS1219.h> // Click here to get the library: http://librarymanager/All#SparkFun_ADS1219

SfeADS1219ArdI2C myADC;

void setup()
{
  delay(1000); // Allow time for the microcontroller to start up

  Serial.begin(115200); // Begin the Serial console
  while (!Serial)
  {
    delay(100); // Wait for the user to open the Serial Monitor
  }
  Serial.println("SparkFun ADS1219 Example");

  Wire.begin(); // Begin the I2C bus

  bool begun;
  begun = myADC.begin(Wire, 0x4F); // Initialize the ADC - using a custom bus and address - see notes above
  begun = myADC.begin(0x4F); // This is also possible. It defaults to Wire

  if (!begun)
  {
    Serial.println("ADC not detected! Please check the address and try again...");
    while (1); // Do nothing more
  }

  Serial.println("Reading the differential voltage between AIN0 (+) and AIN1 (-)");
}

void loop()
{
  myADC.startSync(); // Start a single-shot conversion.

  while (myADC.dataReady() == false) // Check if the conversion is complete. This will return true if data is ready.
  {
    delay(10); // The conversion is not complete. Wait a little to avoid pounding the I2C bus.
  }

  myADC.readConversion(); // Read the conversion result from the ADC. Store it internally.
  int32_t sample = myADC.getConversionRaw(); // Get the raw ADC value. Note: this is NOT adjusted for gain.
  Serial.print("Raw ADC value: ");
  Serial.println(sample); // Print the raw ADC value
}
