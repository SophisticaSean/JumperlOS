// SPDX-License-Identifier: MIT
//
// OG (RP2040 Jumperless) analog transfer functions - pure, no Arduino, so the
// host test (test/test_og_analog) can check the constants against the
// reference firmware (Jumperless 1.3.22, JumperlessNano/src/Peripherals.cpp)
// and the rev 2 schematic.
//
// Everything is expressed through the firmware's shared linear form
//   ADC: volts = raw * spread / 4095 - zero
//   DAC: code  = volts * 4095 / spread + zero
// (Peripherals.cpp readAdcVoltage() / setDac0voltage() / setDac1voltage()),
// so what is tested here is exactly what the board runs.
#pragma once

namespace ogAnalog {

struct Transfer {
    float spread;
    float zero;
};

// ADC0-2: LM324 unity buffer -> 1k/2k divider (R6/R21 ...) -> LM324 unity
// buffer -> RP2040 ADC. 5 V in = 3.33 V at the pin = full scale, so
// volts = raw * 5.0 / 4095 (the reference's showMeasurements).
// ADC3: LM324 buffer -> R17 68k into the R14 21k (+3V3) / R19 47k (GND) node
// -> buffer. The reference's measured map is raw * 16 / 4010 - 8.1, which is
// raw * 16.34 / 4095 - 8.1 in this form: -8.1 V at raw 0, +8.24 V at 4095.
// (The schematic's nominal values give ~18.8 V / 4096 codes with 0 V near
// raw 2330 - the reference's constants are used because they were measured;
// a bench point on GND / 3V3 / 5V shows which is right for a given board.)
constexpr Transfer kAdc[ 4 ] = {
    { 5.0f, 0.0f },
    { 5.0f, 0.0f },
    { 5.0f, 0.0f },
    { 16.34f, 8.1f },
};

inline float adcVolts( int channel, int raw ) {
    if ( channel < 0 || channel > 3 ) return 0.0f;
    if ( raw < 0 ) raw = 0;
    if ( raw > 4095 ) raw = 4095;
    return raw * ( kAdc[ channel ].spread / 4095.0f ) - kAdc[ channel ].zero;
}

// DAC backends, by detected part (Peripherals.cpp initDAC on the OG).
enum DacKind { DAC_NONE = 0, DAC_MCP4822_SPI = 1, DAC_MCP4725_I2C = 2 };

// rev 2: two MCP4725 on I2C0, VDD = +5 V reference.
//   DAC0: unity L272 follower, 0..5 V, code = V * 4095 / 5.
//   DAC1: L272 non-inverting stage, gain 1 + (47k+68k)/(47k+21k) = 2.691 with
//         the ground leg returned to +5 V: Vout = 5 * (2.691 * code/4095 - 1.691)
//         -> 13.45 V per 4096 codes, 0 V at code 2573 (USB-voltage independent).
// rev 3.x: MCP4822 on SPI0, 2x gain (4.096 V full scale).
//   DAC0: unity from the DAC, 0..4.096 V.
//   DAC1: 16 V per 4096 codes, 0 V at code 1772 (bench 2026-09-08).
inline Transfer dacTransfer( DacKind kind, int channel ) {
    if ( kind == DAC_MCP4725_I2C ) {
        return channel == 0 ? Transfer{ 5.0f, 0.0f } : Transfer{ 13.45f, 2573.0f };
    }
    if ( kind == DAC_MCP4822_SPI ) {
        return channel == 0 ? Transfer{ 4.096f, 0.0f } : Transfer{ 16.0f, 1772.0f };
    }
    return Transfer{ 1.0f, 0.0f };
}

inline int dacCode( DacKind kind, int channel, float volts ) {
    Transfer t = dacTransfer( kind, channel );
    // Same expression as setDacNvoltage(): float sum, then truncated.
    int code = (int)( volts * 4095.0f / t.spread + t.zero );
    if ( code < 0 ) code = 0;
    if ( code > 4095 ) code = 4095;
    return code;
}

inline float dacVoltsForCode( DacKind kind, int channel, int code ) {
    Transfer t = dacTransfer( kind, channel );
    return ( code - t.zero ) * t.spread / 4095.0f;
}

// The range a backend can be asked for (what dacCode() maps without
// clamping). The rev 2 DAC1 stage runs on +/-8 V, so the negative end is
// the amplifier's swing, not the code range.
inline float dacMinVolts( DacKind kind, int channel ) {
    if ( kind == DAC_MCP4725_I2C && channel == 1 ) return -6.5f;
    return dacVoltsForCode( kind, channel, 0 );
}
inline float dacMaxVolts( DacKind kind, int channel ) {
    return dacVoltsForCode( kind, channel, 4095 );
}

}  // namespace ogAnalog
