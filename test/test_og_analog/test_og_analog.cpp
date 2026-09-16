// SPDX-License-Identifier: MIT
//
// Host check for the OG analog transfer constants (src/boards/og/og_analog.h)
// - the pure math under Peripherals.cpp's readAdcVoltage() / setDacNvoltage()
// on an OG board. Pinned against:
//   * the reference firmware Jumperless 1.3.22 (JumperlessNano/src/Peripherals.cpp
//     showMeasurements: ADC0-2 raw*5.0/4095, ADC3 raw*16.0/4010 - 8.1; rev 3
//     setDac0_5Vvoltage V*4095/5),
//   * the rev 2 schematic (Hardware/KiCAD/Jumperless Rev 2): MCP4725 x2 at
//     0x60/0x61, DAC1 L272 stage gain 2.691 with the ground leg to +5 V,
//   * the board descriptor (board_og.cpp kOgAdc) - the two tables must agree.
//
// Build & run (no hardware, no PlatformIO):   test/test_og_analog/run.sh
#include "boards/og/og_analog.h"
#include "boards/board.h"
#include "JumperlessDefines.h"

#include <cmath>
#include <cstdio>

static int failures = 0;
#define CHECK(cond, msg)                                                     \
  do {                                                                       \
    if (!(cond)) {                                                           \
      std::printf("FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__);           \
      failures++;                                                            \
    }                                                                        \
  } while (0)
#define NEAR(a, b, tol, msg) CHECK(std::fabs((a) - (b)) <= (tol), msg)

using namespace ogAnalog;

int main() {
  // --- ADC0-2: the reference's raw * 5.0 / 4095 ----------------------------
  for (int ch = 0; ch < 3; ch++) {
    NEAR(adcVolts(ch, 0), 0.0f, 1e-6f, "ADC0-2 raw 0 = 0 V");
    NEAR(adcVolts(ch, 4095), 5.0f, 1e-6f, "ADC0-2 raw 4095 = 5.0 V");
    NEAR(adcVolts(ch, 2703), 2703 * 5.0f / 4095, 1e-5f, "ADC0-2 3.3 V point");
    NEAR(adcVolts(ch, 688), 0.84f, 0.01f, "ADC0 GND reads 0.84 on the reference bench (raw ~688)");
  }
  // --- ADC3: the reference's raw * 16 / 4010 - 8.1 ---------------------------
  for (int raw = 0; raw <= 4095; raw += 5) {
    float ref = raw * (16.0f / 4010) - 8.1f;
    NEAR(adcVolts(3, raw), ref, 0.02f, "ADC3 matches the reference's 16/4010 - 8.1 map");
  }
  NEAR(adcVolts(3, 0), -8.1f, 1e-5f, "ADC3 raw 0 = -8.1 V");
  NEAR(adcVolts(3, 4095), 8.24f, 1e-4f, "ADC3 raw 4095 = +8.24 V");
  CHECK(adcVolts(4, 100) == 0.0f && adcVolts(-1, 100) == 0.0f, "ADC channel out of range -> 0");
  CHECK(adcVolts(0, 5000) == adcVolts(0, 4095), "raw clamps at 4095");

  // --- descriptor and og_analog agree (one table drives JSON + conversion) ----
  const board::BoardTopology &b = board::ogBoardTopology;
  CHECK(b.adcCount == 4, "OG has 4 ADC channels");
  for (int i = 0; i < b.adcCount; i++) {
    int ch = b.adc[i].node - ADC0;
    CHECK(ch >= 0 && ch < 4, "descriptor ADC node is ADC0..ADC3");
    if (ch < 0 || ch >= 4) continue;
    NEAR(b.adc[i].maxV - b.adc[i].minV, kAdc[ch].spread, 1e-4f, "descriptor spread == og_analog spread");
    NEAR(-b.adc[i].minV, kAdc[ch].zero, 1e-4f, "descriptor zero == og_analog zero");
  }

  // --- rev 2 DAC0: MCP4725 with VDD (+5 V) as reference, unity follower -----
  CHECK(dacCode(DAC_MCP4725_I2C, 0, 0.0f) == 0, "rev2 DAC0 0 V = code 0");
  CHECK(dacCode(DAC_MCP4725_I2C, 0, 5.0f) == 4095, "rev2 DAC0 5 V = code 4095");
  CHECK(dacCode(DAC_MCP4725_I2C, 0, 2.5f) == (int)(2.5f * 4095 / 5), "rev2 DAC0 2.5 V = V*4095/5 (reference formula)");
  CHECK(dacCode(DAC_MCP4725_I2C, 0, 9.0f) == 4095, "rev2 DAC0 clamps high");
  CHECK(dacCode(DAC_MCP4725_I2C, 0, -1.0f) == 0, "rev2 DAC0 clamps low");
  NEAR(dacVoltsForCode(DAC_MCP4725_I2C, 0, 2047), 2.4988f, 1e-3f, "rev2 DAC0 mid code = 2.5 V");

  // --- rev 2 DAC1: Vout = V5 * (2.691 * code/4095 - 1.691) -------------------
  // From the schematic: gain 1 + (R16 47k + R20 68k) / (R15 47k + R13 21k),
  // ground leg to +5 V. Check og_analog's linear form against that model.
  const float gain = 1.0f + (47.0f + 68.0f) / (47.0f + 21.0f);
  for (int code = 0; code <= 4095; code += 15) {
    float model = 5.0f * (gain * code / 4095.0f - (gain - 1.0f));
    NEAR(dacVoltsForCode(DAC_MCP4725_I2C, 1, code), model, 0.03f, "rev2 DAC1 matches the L272 stage model");
  }
  NEAR(dacVoltsForCode(DAC_MCP4725_I2C, 1, 2573), 0.0f, 0.01f, "rev2 DAC1 0 V at code 2573");
  CHECK(dacCode(DAC_MCP4725_I2C, 1, 0.0f) == 2573, "rev2 DAC1 asks 0 V -> code 2573");
  NEAR(dacVoltsForCode(DAC_MCP4725_I2C, 1, 4095), 5.0f, 0.01f, "rev2 DAC1 full code = +5 V (its top)");
  NEAR(dacVoltsForCode(DAC_MCP4725_I2C, 1, 0), -8.45f, 0.02f, "rev2 DAC1 code 0 = -8.45 V nominal (past the -8 V rail)");
  NEAR(dacMinVolts(DAC_MCP4725_I2C, 1), -6.5f, 1e-6f, "rev2 DAC1 usable minimum is the amplifier swing");
  NEAR(dacMaxVolts(DAC_MCP4725_I2C, 1), 5.0f, 0.01f, "rev2 DAC1 usable maximum");
  // Round trip through the firmware's truncating code expression.
  for (float v = -6.0f; v <= 5.0f; v += 0.25f) {
    int code = dacCode(DAC_MCP4725_I2C, 1, v);
    NEAR(dacVoltsForCode(DAC_MCP4725_I2C, 1, code), v, 13.45f / 4095 + 1e-3f, "rev2 DAC1 round trip within 1 LSB");
  }

  // --- rev 3 MCP4822 (unchanged from the 2026-09-08 bench) --------------------
  CHECK(dacCode(DAC_MCP4822_SPI, 0, 4.096f) == 4095, "rev3 DAC0 4.096 V = full scale");
  CHECK(dacCode(DAC_MCP4822_SPI, 1, 0.0f) == 1772, "rev3 DAC1 0 V = code 1772");
  NEAR(dacVoltsForCode(DAC_MCP4822_SPI, 1, 4095), 9.07f, 0.01f, "rev3 DAC1 code range top (clips ~7 V on hw)");

  CHECK(dacTransfer(DAC_NONE, 0).spread == 1.0f, "no DAC -> identity transfer, never a divide by 0");

  if (failures == 0) std::printf("test_og_analog: all checks passed\n");
  else std::printf("test_og_analog: %d FAILURES\n", failures);
  return failures == 0 ? 0 : 1;
}
