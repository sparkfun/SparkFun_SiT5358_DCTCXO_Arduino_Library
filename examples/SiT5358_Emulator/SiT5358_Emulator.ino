/*
  Emulate the SiT5358 DCTCXO.

  By: Paul Clark
  SparkFun Electronics
  Date: 2024/8/1
  SparkFun code, firmware, and software is released under the MIT License.
  Please see LICENSE.md for further details.

*/

#include <Wire.h>

#define I2C_DEV_ADDR 0x60

uint8_t registerBytes[6] = { 0, 0, 0, 0, 0, 0 };
uint8_t registerAddress = 0;

// On Request:
// Write bytes from registerBytes, starting at registerAddress * 2
void onRequest()
{
  int i = registerAddress * 2;
  for (; i < 6; i++)
    Wire.write(registerBytes[i]);
}

// On Receive:
// Copy the first incoming byte into registerAddress
// Copy the remaining bytes into registerBytes, starting at registerAddress * 2
void onReceive(int len)
{
  int count = -1;
  while (Wire.available())
  {
    uint8_t b = Wire.read();
    switch (count)
    {
      case -1:
        registerAddress = b;
        break;
      default:
        if (((registerAddress * 2) + count) < 6)
          registerBytes[((registerAddress * 2) + count)] = b;
        break;
    }
    count++;
  }
}

void setup()
{
  delay(1000); // Allow time for the microcontroller to start up

  Serial.begin(115200); // Begin the Serial console
  while (!Serial)
  {
    delay(100); // Wait for the user to open the Serial Monitor
  }
  Serial.println("SparkFun SiT5358 Emulator");

  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
}

void loop()
{
  static unsigned long lastPrint = 0;

  if (millis() > (lastPrint + 1000))
  {
    lastPrint = millis();

    // Extract the 26-bit Frequency Control Word
    uint32_t freqControl = ((uint32_t)registerBytes[0]) << 8;
    freqControl |= (uint32_t)registerBytes[1];
    freqControl |= (((uint32_t)registerBytes[2]) & 0x03) << 24;
    freqControl |= ((uint32_t)registerBytes[3]) << 16;
    if (freqControl & 0x02000000) // Correct two's complement
      freqControl |= 0xFC000000;
    
    union // Avoid any ambiguity when converting uint32_t to int32_t
    {
        uint32_t unsigned32;
        int32_t signed32;
    } unsignedSigned32;
    unsignedSigned32.unsigned32 = freqControl;

    Serial.print("Frequency control is ");
    Serial.print(unsignedSigned32.signed32);

    // Extract the 4-bit Pull Range Control
    Serial.print(". Pull range control is ");
    Serial.println(registerBytes[5]);
  }
}