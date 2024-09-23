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
#include <SparkFun_Toolkit.h>

///////////////////////////////////////////////////////////////////////////////
// I2C Addressing
///////////////////////////////////////////////////////////////////////////////
// The SiT5358 can be ordered with a pre-programmed I2C address in the range
// 0x60 to 0x6F (unshifted). It can also be ordered with a selectable address
// of 0x62 / 0x6A via the A0 Pin 5. Here we assume a default address of 0x60.
// The actual address can be defined via the begin method.
const uint8_t kDefaultSiT5358Addr = 0x60; // 

///////////////////////////////////////////////////////////////////////////////
// Enum Definitions
///////////////////////////////////////////////////////////////////////////////

typedef enum
{
    SiT5358_PULL_RANGE_6ppm25 = 0,
    SiT5358_PULL_RANGE_10ppm,
    SiT5358_PULL_RANGE_12ppm5,
    SiT5358_PULL_RANGE_25ppm,
    SiT5358_PULL_RANGE_50ppm,
    SiT5358_PULL_RANGE_80ppm,
    SiT5358_PULL_RANGE_100ppm,
    SiT5358_PULL_RANGE_125ppm,
    SiT5358_PULL_RANGE_150ppm,
    SiT5358_PULL_RANGE_200ppm,
    SiT5358_PULL_RANGE_400ppm,
    SiT5358_PULL_RANGE_600ppm,
    SiT5358_PULL_RANGE_800ppm,
    SiT5358_PULL_RANGE_1200ppm,
    SiT5358_PULL_RANGE_1600ppm,
    SiT5358_PULL_RANGE_3200ppm,
} sfe_SiT5358_pull_range_control_t;

///////////////////////////////////////////////////////////////////////////////
// 16-bit Register Addresses
///////////////////////////////////////////////////////////////////////////////

const uint8_t kSfeSiT5358RegControlLSW = 0x00; // Digital Frequency Control Least Significant Word (LSW)
const uint8_t kSfeSiT5358RegControlMSW = 0x01; // Digital Frequency Control Most Significant Word (MSW)
const uint8_t kSfeSiT5358RegPullRange = 0x02; // Digital Pull Range Control

///////////////////////////////////////////////////////////////////////////////
// Digital Frequency Control Most Significant Word (MSW) Register Description
///////////////////////////////////////////////////////////////////////////////

// A union is used here so that individual values from the register can be
// accessed or the whole register can be accessed.
typedef union
{
    struct
    {
        uint16_t freqControl : 10; // DIGITAL FREQUENCY CONTROL MOST SIGNIFICANT WORD (MSW) : RW
        uint16_t oe : 1;   // Output Enable Software Control : RW : DCTCXO option "J"
        uint16_t notUsed : 5; // Not used. Read as 0's
    };
    uint16_t word;
} sfe_SiT5358_reg_control_msw_t;

///////////////////////////////////////////////////////////////////////////////
// Digital Pull Range Control Register Description
///////////////////////////////////////////////////////////////////////////////

// A union is used here so that individual values from the register can be
// accessed or the whole register can be accessed.
typedef union
{
    struct
    {
        uint16_t pullRange : 4; // DIGITAL PULL RANGE CONTROL : RW
        uint16_t notUsed : 12; // Not used. Read as 0's
    };
    uint16_t word;
} sfe_SiT5358_reg_pull_range_t;

///////////////////////////////////////////////////////////////////////////////

class SfeSiT5358Driver
{
public:
    // @brief Constructor. Instantiate the driver object using the specified address (if desired).
    SfeSiT5358Driver()
        : _baseFrequencyHz{10000000.0}, _maxFrequencyChangePPB{3200000.0}
    {
    }

    /// @brief Begin communication with the SiT5358. Read the registers.
    /// @return true if readRegisters is successful.
    bool begin(void);

    /// @brief Read all three SiT5358 registers and update the driver's internal copies
    /// @return true if the read is successful and the Frequency Control MSW and Pull Range Control have 0's where expected
    bool readRegisters(void);


    /// @brief Get the 26-bit frequency control word - from the driver's internal copy
    /// @return The 26-bit frequency control word as int32_t (signed, two's complement)
    int32_t getFrequencyControlWord(void);

    /// @brief Set the 26-bit frequency control word - and update the driver's internal copy
    /// @param freq the frequency control word as int32_t (signed, two's complement)
    /// @return true if the write is successful
    bool setFrequencyControlWord(int32_t freq);


    /// @brief Get the OE control bit - from the driver's internal copy
    /// @return The OE control bit as bool
    bool getOEControl(void);

    /// @brief Set the OE control - and update the driver's internal copy
    /// @param oe the OE control bit : false = disabled; true = enabled
    /// @return true if the write is successful
    /// Note: only valid on option "J" parts
    bool setOEControl(bool oe);


    /// @brief Get the 4-bit pull range control - from the driver's internal copy
    /// @return The 4-bit pull range as uint8_t (in the four LS bits)
    uint8_t getPullRangeControl(void);

    /// @brief Set the 4-bit pull range control - and update the driver's internal copy
    /// @param pullRange the 4-bit pull range (in the four LS bits)
    /// @return true if the write is successful
    bool setPullRangeControl(uint8_t pullRange);


    /// @brief Get the base oscillator frequency - from the driver's internal copy
    /// @return The oscillator base frequency as double
    double getBaseFrequencyHz(void);

    /// @brief Set the base oscillator frequency in Hz - set the driver's internal _baseFrequencyHz
    /// @param freq the base frequency in Hz
    /// @return true if the write is successful
    void setBaseFrequencyHz(double freq);


    /// @brief Get the oscillator frequency based on the base frequency, pull range and control word
    /// @return The oscillator frequency as double
    double getFrequencyHz(void);

    /// @brief Set the oscillator frequency based on the base frequency and pull range
    /// @param freq the oscillator frequency in Hz
    /// @return true if the write is successful
    /// Note: The frequency change will be limited by the pull range capabilities of the device.
    ///       Call getFrequencyHz to read the frequency set.
    /// Note: setFrequencyHz ignores _maxFrequencyChangePPB.
    bool setFrequencyHz(double freq);


    /// @brief Get the maximum frequency change in PPB
    /// @return The maximum frequency change in PPB - from the driver's internal store
    double getMaxFrequencyChangePPB(void);

    /// @brief Set the maximum frequency change in PPB - set the driver's internal _maxFrequencyChangePPB
    /// @param ppb the maximum frequency change in PPB
    void setMaxFrequencyChangePPB(double ppb);


    /// @brief Set the frequency according to the GNSS receiver clock bias in milliseconds
    /// @param bias the GNSS RX clock bias in milliseconds
    /// @param Pk the Proportional term
    /// @param Ik the Integral term
    /// @return true if the write is successful
    /// Note: the frequency change will be limited by: the pull range capabilities of the device;
    ///       and the setMaxFrequencyChangePPB. Call getFrequencyHz to read the frequency set.
    /// The default values for Pk and Ik come from very approximate Ziegler-Nichols tuning:
    /// oscillation starts when Pk is ~1.4; with a period of ~5 seconds.
    bool setFrequencyByBiasMillis(double bias, double Pk = 0.63, double Ik = 0.151);


    /// @brief Convert the 4-bit pull range into text
    /// @return the pull range as text
    const char * getPullRangeControlText(uint8_t pullRange);

    /// @brief Convert the 4-bit pull range into double
    /// @return the pull range as double
    double getPullRangeControlDouble(uint8_t pullRange);

protected:
    /// @brief Sets the communication bus to the specified bus.
    /// @param theBus Bus to set as the communication devie.
    void setCommunicationBus(sfeTkArdI2C *theBus);

private:
    sfeTkArdI2C *_theBus; // Pointer to bus device.

    int32_t _frequencyControl; // Local store for the frequency control word. 26-Bit, 2's complement
    uint8_t _pullRange; // Local store for the pull range control nibble. 4-bit
    bool _oe; // Local store for the OE control bit
    double _baseFrequencyHz; // The base frequency used by getFrequencyHz and setFrequencyHz
    double _maxFrequencyChangePPB; // The maximum frequency change in PPB for setFrequencyByBiasMillis
};

class SfeSiT5358ArdI2C : public SfeSiT5358Driver
{
public:
    SfeSiT5358ArdI2C()
    {
    }

    /// @brief  Sets up Arduino I2C driver using the default I2C address then calls the super class begin.
    /// @return True if successful, false otherwise.
    bool begin(void)
    {
        if (_theI2CBus.init(kDefaultSiT5358Addr) != kSTkErrOk)
            return false;

        setCommunicationBus(&_theI2CBus);

        _theI2CBus.setStop(false); // Use restarts not stops for I2C reads

        return SfeSiT5358Driver::begin();
    }

    /// @brief  Sets up Arduino I2C driver using the specified I2C address then calls the super class begin.
    /// @return True if successful, false otherwise.
    bool begin(const uint8_t &address)
    {
        if (_theI2CBus.init(address) != kSTkErrOk)
            return false;

        setCommunicationBus(&_theI2CBus);

        _theI2CBus.setStop(false); // Use restarts not stops for I2C reads

        return SfeSiT5358Driver::begin();
    }

    /// @brief  Sets up Arduino I2C driver using the specified I2C address then calls the super class begin.
    /// @return True if successful, false otherwise.
    bool begin(TwoWire &wirePort, const uint8_t &address)
    {
        if (_theI2CBus.init(wirePort, address) != kSTkErrOk)
            return false;

        setCommunicationBus(&_theI2CBus);

        _theI2CBus.setStop(false); // Use restarts not stops for I2C reads

        return SfeSiT5358Driver::begin();
    }

private:
    sfeTkArdI2C _theI2CBus;
};
