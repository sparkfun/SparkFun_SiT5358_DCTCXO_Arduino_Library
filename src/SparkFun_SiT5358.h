/*
    SparkFun SiT5358 DCTCXO Arduino Library

    Repository
    https://github.com/sparkfun/SparkFun_SiT5358_DCTCXO_Arduino_Library

    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics

    Name: SparkFun_SiT5358.h

    Description:
    An Arduino Library for the SiT5358 Digitally-Controlled
    Temperature-Compensated Crystal Oscillator from SiTime.
    Requires the SparkFun Toolkit:
    https://github.com/sparkfun/SparkFun_Toolkit

    Notes:
    The SiT5358 has three 16-bit registers: two define the 26-bit frequency control word;
    one specifies the frequency pull range.
    The frequency is changed via the frequency control word.
    The frequency control word does not set the frequency directly. Instead it defines how
    far the base frequency is to be pulled in Parts Per Million.
    To set the oscillator to a specific frequency, it is necessary to know the base frequency
    and pull range limit (in ppm). The frequency control word defines the pull as fractional ppm.

*/

#pragma once

#include <stdint.h>

#include <Arduino.h>

#include <Arduino.h>
// .. some header order issue right now...
// clang-format off
#include <SparkFun_Toolkit.h>
#include "sfTk/sfDevSiT5358.h"
// clang-format on

class SfeSiT5358ArdI2C : public sfDevSiT5358
{
  public:
    SfeSiT5358ArdI2C()
    {
    }

    /// @brief  Sets up Arduino I2C driver using the default I2C address then calls the super class begin.
    /// @return True if successful, false otherwise.
    // bool begin(void)
    // {
    //     if (_theI2CBus.init(kDefaultSiT5358Addr) != kSTkErrOk)
    //         return false;

    //     setCommunicationBus(&_theI2CBus);

    //     _theI2CBus.setStop(false); // Use restarts not stops for I2C reads

    //     return SfeSiT5358Driver::begin();
    // }

    /// @brief  Sets up Arduino I2C driver using the specified I2C address then calls the super class begin.
    /// @return True if successful, false otherwise.
    bool begin(const uint8_t address = kDefaultSiT5358Addr)
    {
        if (_theI2CBus.init(address) != ksfTkErrOk)
            return false;

        return beginDevice();
    }

    /// @brief  Sets up Arduino I2C driver using the specified I2C address then calls the super class begin.
    /// @return True if successful, false otherwise.
    bool begin(TwoWire &wirePort, const uint8_t &address)
    {
        if (_theI2CBus.init(wirePort, address) != ksfTkErrOk)
            return false;

        return beginDevice();
    }

  private:
    bool beginDevice(void)
    {

        // the intent is that the bus is setup and we can see if the device is connected
        if (_theI2CBus.ping() == ksfTkErrOk)
            return false;

        _theI2CBus.setStop(false); // Use restarts not stops for I2C reads

        return sfDevSiT5358::begin(&_theI2CBus) == ksfTkErrOk;
    }

    sfTkArdI2C _theI2CBus;
};
