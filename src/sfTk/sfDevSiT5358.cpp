/*
    SparkFun SiT5358 DCTCXO Arduino Library

    Repository
    https://github.com/sparkfun/SparkFun_SiT5358_DCTCXO_Arduino_Library

    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics

    Name: SparkFun_SiT5358.cpp

    Description:
    An Arduino Library for the SiT5358 Digitally-Controlled
    Temperature-Compensated Crystal Oscillator from SiTime.
    Requires the SparkFun Toolkit:
    https://github.com/sparkfun/SparkFun_Toolkit

*/

#include "sfDevSiT5358.h"

/// @brief Begin communication with the SiT5358. Read the registers.
/// @return true if readRegisters is successful.
sfTkError_t sfDevSiT5358::begin(sfTkII2C *commBus)
{
    if (commBus == nullptr)
        return ksfTkErrFail;

    _theBus = commBus;

    if (_theBus->ping() != ksfTkErrOk)
        return ksfTkErrFail;

    return readRegisters() ? ksfTkErrOk : ksfTkErrFail;
}

/// @brief Read all three SiT5358 registers and update the driver's internal copies
/// @return true if the read is successful and the Frequency Control MSW and Pull Range Control have 0's where expected
bool sfDevSiT5358::readRegisters(void)
{
    // Read 6 bytes, starting at address kSfeSiT5358RegControlLSW (0x00)
    uint8_t theBytes[6];
    size_t readBytes;
    if (_theBus->readRegister(kSfeSiT5358RegControlLSW, (uint8_t *)&theBytes[0], 6, readBytes) != ksfTkErrOk)
        return false;
    if (readBytes != 6)
        return false;

    // Extract the three 16-bit registers - MSB first
    uint16_t register00 = (((uint16_t)theBytes[0]) << 8) | ((uint16_t)theBytes[1]); // Frequency Control LSW
    uint16_t register01 = (((uint16_t)theBytes[2]) << 8) | ((uint16_t)theBytes[3]); // Frequency Control MSW
    uint16_t register02 = (((uint16_t)theBytes[4]) << 8) | ((uint16_t)theBytes[5]); // Pull Range Control

    if ((register01 & 0xF800) != 0)
        return false; // Return false if Frequency Control MSW bits 11-15 are non-zero
    if ((register02 & 0xFFF0) != 0)
        return false; // Return false if Pull Range Control bits 4-15 are non-zero

    // Extract the frequency control and OE bits from register01
    sfe_SiT5358_reg_control_msw_t controlMSW;
    controlMSW.word = register01;
    union // Avoid any ambiguity when converting uint32_t to int32_t
    {
        uint32_t unsigned32;
        int32_t signed32;
    } unsignedSigned32;
    unsignedSigned32.unsigned32 = (((uint32_t)controlMSW.freqControl) << 16) | ((uint32_t)register00);
    if ((unsignedSigned32.unsigned32 & 0x02000000) != 0) // Two's complement
        unsignedSigned32.unsigned32 |= 0xFC000000;
    _frequencyControl = unsignedSigned32.signed32; // Store the two's complement frequency control word

    _oe = (bool)controlMSW.oe; // Store the OE bit (option "J" only)

    _pullRange = theBytes[5] & 0x0F; // Store the pull range control nibble

    return true;
}

/// @brief Get the 26-bit frequency control word - from the driver's internal copy
/// @return The 26-bit frequency control word as int32_t (signed, two's complement)
int32_t sfDevSiT5358::getFrequencyControlWord(void)
{
    return _frequencyControl;
}

/// @brief Set the 26-bit frequency control word - and update the driver's internal copy
/// @param freq the frequency control word as int32_t (signed, two's complement)
/// @return true if the write is successful
bool sfDevSiT5358::setFrequencyControlWord(int32_t freq)
{
    uint8_t theBytes[4];

    union // Avoid any ambiguity when converting uint32_t to int32_t
    {
        uint32_t unsigned32;
        int32_t signed32;
    } unsignedSigned32;
    unsignedSigned32.signed32 = freq;
    theBytes[0] = (uint8_t)((unsignedSigned32.unsigned32 & 0x0000FF00) >> 8);
    theBytes[1] = (uint8_t)(unsignedSigned32.unsigned32 & 0x000000FF);
    theBytes[2] = (uint8_t)((unsignedSigned32.unsigned32 & 0x03000000) >> 24) | (((uint8_t)_oe) << 2);
    theBytes[3] = (uint8_t)((unsignedSigned32.unsigned32 & 0x00FF0000) >> 16);

    if (_theBus->writeRegister((uint8_t)0x00, (const uint8_t *)&theBytes[0], 4) != ksfTkErrOk)
        return false; // Return false if the write failed

    _frequencyControl = freq; // Only update the driver's copy if the write was successful
    return true;
}

/// @brief Get the OE control bit - from the driver's internal copy
/// @return The OE control bit as bool
bool sfDevSiT5358::getOEControl(void)
{
    return _oe;
}

/// @brief Set the OE control - and update the driver's internal copy
/// @param oe the OE control bit : false = disabled; true = enabled
/// @return true if the write is successful
/// Note: only valid on option "J" parts
bool sfDevSiT5358::setOEControl(bool oe)
{
    // It may be possible to write only the 16-bit MSW, but, for safety, write both LSW and MSW
    uint8_t theBytes[4];

    union // Avoid any ambiguity when converting uint32_t to int32_t
    {
        uint32_t unsigned32;
        int32_t signed32;
    } unsignedSigned32;
    unsignedSigned32.signed32 = _frequencyControl;
    theBytes[0] = (uint8_t)((unsignedSigned32.unsigned32 & 0x0000FF00) >> 8);
    theBytes[1] = (uint8_t)(unsignedSigned32.unsigned32 & 0x000000FF);
    theBytes[2] = (uint8_t)((unsignedSigned32.unsigned32 & 0x03000000) >> 24) | (((uint8_t)oe) << 2);
    theBytes[3] = (uint8_t)((unsignedSigned32.unsigned32 & 0x00FF0000) >> 16);

    if (_theBus->writeRegister(kSfeSiT5358RegControlLSW, (const uint8_t *)&theBytes[0], 4) != ksfTkErrOk)
        return false; // Return false if the write failed

    _oe = oe; // Only update the driver's copy if the write was successful
    return true;
}

/// @brief Get the 4-bit pull range control - from the driver's internal copy
/// @return The 4-bit pull range as uint8_t (in the four LS bits)
uint8_t sfDevSiT5358::getPullRangeControl(void)
{
    return _pullRange;
}

/// @brief Set the 4-bit pull range control - and update the driver's internal copy
/// @param pullRange the 4-bit pull range (in the four LS bits)
/// @return true if the write is successful
bool sfDevSiT5358::setPullRangeControl(uint8_t pullRange)
{
    uint8_t theBytes[2];

    theBytes[0] = 0;
    theBytes[1] = pullRange & 0x0F;

    if (_theBus->writeRegister(kSfeSiT5358RegPullRange, (const uint8_t *)&theBytes[0], 2) != ksfTkErrOk)
        return false; // Return false if the write failed

    _pullRange = pullRange; // Only update the driver's copy if the write was successful
    return true;
}

/// @brief Get the base oscillator frequency - from the driver's internal copy
/// @return The oscillator base frequency as double
double sfDevSiT5358::getBaseFrequencyHz(void)
{
    return _baseFrequencyHz;
}

/// @brief Set the base oscillator frequency in Hz - set the driver's internal _baseFrequencyHz
/// @param freq the base frequency in Hz
void sfDevSiT5358::setBaseFrequencyHz(double freq)
{
    _baseFrequencyHz = freq;
}

/// @brief Get the oscillator frequency based on the base frequency, pull range and control word
/// @return The oscillator frequency as double
double sfDevSiT5358::getFrequencyHz(void)
{
    double pullRangeDbl = getPullRangeControlDouble(_pullRange);

    double freqControl = (double)_frequencyControl;

    if (freqControl >= 0.0)
        freqControl /= 33554431.0; // Scale 0.0 to 1.0
    else
        freqControl /= 33554432.0; // Scale 0.0 to -1.0

    double freqOffsetHz = _baseFrequencyHz * freqControl * pullRangeDbl;
    double freqHz = _baseFrequencyHz + freqOffsetHz;

    return freqHz;
}

/// @brief Set the oscillator frequency based on the base frequency and pull range
/// @param freq the oscillator frequency in Hz
/// @return true if the write is successful
/// Note: The frequency change will be limited by the pull range capabilities of the device.
///       Call getFrequencyHz to read the frequency set.
/// Note: setFrequencyHz ignores _maxFrequencyChangePPB.
bool sfDevSiT5358::setFrequencyHz(double freq)
{
    // Calculate the frequency offset from the base frequency
    double freqOffsetHz = freq - _baseFrequencyHz;

    // Calculate the frequency offset as a fraction of the pull range
    double pullRangeDbl = getPullRangeControlDouble(_pullRange);

    double maxFreqOffsetHz = _baseFrequencyHz * pullRangeDbl;

    double freqControl = freqOffsetHz / maxFreqOffsetHz;

    if (freqControl >= 0.0)
    {
        if (freqControl > 1.0)
            freqControl = 1.0;

        freqControl *= 33554431.0;
    }
    else
    {
        if (freqControl < -1.0)
            freqControl = -1.0;

        freqControl *= 33554432.0;
    }

    int32_t freqControlInt = (int32_t)freqControl;

    // Just in case, ensure freqControlInt is limited to 2^25 (26-bits signed)
    if (freqControlInt > 33554431)
        freqControlInt = 33554431;

    if (freqControlInt < -33554432)
        freqControlInt = -33554432;

    return setFrequencyControlWord(freqControlInt);
}

/// @brief Get the maximum frequency change in PPB
/// @return The maximum frequency change in PPB - from the driver's internal store
double sfDevSiT5358::getMaxFrequencyChangePPB(void)
{
    return _maxFrequencyChangePPB;
}

/// @brief Set the maximum frequency change in PPB - set the driver's internal _maxFrequencyChangePPB
/// @param ppb the maximum frequency change in PPB
void sfDevSiT5358::setMaxFrequencyChangePPB(double ppb)
{
    _maxFrequencyChangePPB = ppb;
}

/// @brief Set the frequency according to the GNSS receiver clock bias in milliseconds
/// @param bias the GNSS RX clock bias in milliseconds
/// @param Pk the Proportional term
/// @param Ik the Integral term
/// @return true if the write is successful
/// Note: the frequency change will be limited by: the pull range capabilities of the device;
///       and the setMaxFrequencyChangePPB. Call getFrequencyHz to read the frequency set.
/// The default values for Pk and Ik come from very approximate Ziegler-Nichols tuning:
/// oscillation starts when Pk is ~1.4; with a period of ~5 seconds.
bool sfDevSiT5358::setFrequencyByBiasMillis(double bias, double Pk, double Ik)
{
    double freq = getFrequencyHz();

    static double I;
    static bool initialized = false;
    if (!initialized)
    {
        I = freq; // Initialize I with the current frequency for a more reasonable startup
        initialized = true;
    }

    double clockInterval_s = 1.0 / freq; // Convert freq to interval in seconds

    // Our setpoint is zero. Bias is the process value. Convert it to error
    double error = 0.0 - bias;

    double errorInClocks = error / 1000.0; // Convert error from millis to seconds
    errorInClocks /= clockInterval_s;      // Convert error to clock cycles

    // Calculate the maximum frequency change in clock cycles
    double maxChangeInClocks = freq * _maxFrequencyChangePPB / 1.0e9;

    // Limit errorInClocks to +/-maxChangeInClocks
    if (errorInClocks >= 0.0)
    {
        if (errorInClocks > maxChangeInClocks)
            errorInClocks = maxChangeInClocks;
    }
    else
    {
        if (errorInClocks < (0.0 - maxChangeInClocks))
            errorInClocks = 0.0 - maxChangeInClocks;
    }

    double P = errorInClocks * Pk;
    double dI = errorInClocks * Ik;
    I += dI; // Add the delta to the integral

    return setFrequencyHz(P + I); // Set the frequency to proportional plus integral
}

/// @brief Convert the 4-bit pull range into text
/// @return the pull range as text
const char *sfDevSiT5358::getPullRangeControlText(uint8_t pullRange)
{
    switch (pullRange)
    {
    default: // Can only be true if pullRange is invalid (>= 0x10)
        return ("Invalid");
    case SiT5358_PULL_RANGE_6ppm25:
        return "6.25ppm";
    case SiT5358_PULL_RANGE_10ppm:
        return "10ppm";
    case SiT5358_PULL_RANGE_12ppm5:
        return "12.5ppm";
    case SiT5358_PULL_RANGE_25ppm:
        return "25ppm";
    case SiT5358_PULL_RANGE_50ppm:
        return "50ppm";
    case SiT5358_PULL_RANGE_80ppm:
        return "80ppm";
    case SiT5358_PULL_RANGE_100ppm:
        return "100ppm";
    case SiT5358_PULL_RANGE_125ppm:
        return "125ppm";
    case SiT5358_PULL_RANGE_150ppm:
        return "150ppm";
    case SiT5358_PULL_RANGE_200ppm:
        return "200ppm";
    case SiT5358_PULL_RANGE_400ppm:
        return "400ppm";
    case SiT5358_PULL_RANGE_600ppm:
        return "600ppm";
    case SiT5358_PULL_RANGE_800ppm:
        return "800ppm";
    case SiT5358_PULL_RANGE_1200ppm:
        return "1200ppm";
    case SiT5358_PULL_RANGE_1600ppm:
        return "1600ppm";
    case SiT5358_PULL_RANGE_3200ppm:
        return "3200ppm";
    }
}

/// @brief Convert the 4-bit pull range into double
/// @return the pull range as double
double sfDevSiT5358::getPullRangeControlDouble(uint8_t pullRange)
{
    switch (pullRange & 0x0F)
    {
    default:
    case SiT5358_PULL_RANGE_6ppm25:
        return 6.25e-6;
    case SiT5358_PULL_RANGE_10ppm:
        return 10e-6;
    case SiT5358_PULL_RANGE_12ppm5:
        return 12.5e-6;
    case SiT5358_PULL_RANGE_25ppm:
        return 25e-6;
    case SiT5358_PULL_RANGE_50ppm:
        return 50e-6;
    case SiT5358_PULL_RANGE_80ppm:
        return 80e-6;
    case SiT5358_PULL_RANGE_100ppm:
        return 100e-6;
    case SiT5358_PULL_RANGE_125ppm:
        return 125e-6;
    case SiT5358_PULL_RANGE_150ppm:
        return 150e-6;
    case SiT5358_PULL_RANGE_200ppm:
        return 200e-6;
    case SiT5358_PULL_RANGE_400ppm:
        return 400e-6;
    case SiT5358_PULL_RANGE_600ppm:
        return 600e-6;
    case SiT5358_PULL_RANGE_800ppm:
        return 800e-6;
    case SiT5358_PULL_RANGE_1200ppm:
        return 1200e-6;
    case SiT5358_PULL_RANGE_1600ppm:
        return 1600e-6;
    case SiT5358_PULL_RANGE_3200ppm:
        return 3200e-6;
    }
}

/// @brief  PROTECTED: update the local pointer to the I2C bus.
/// @param  theBus Pointer to the bus object.
void sfDevSiT5358::setCommunicationBus(sfTkII2C *theBus)
{
    _theBus = theBus;
}
