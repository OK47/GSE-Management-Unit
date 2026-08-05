#ifndef KJO_ANALOG_H
#define KJO_ANALOG_H

//
//    Ken Overton
//    Analog input subsystem  --  external ADS1015 A/D daughter board
//
//    The external ADS1015 (I2C, 12-bit) is used for:
//      Channel 3  --  2S LiPo battery voltage (via resistor divider)
//      Channel 0  --  AUX analog input (thresholded to a digital HIGH/LOW)
//
//    Battery voltage calculation:
//      V_bat = raw_count * AD_BASE_SCALE * GSEMU_LIPO_SCALE
//
//    AUX digital state:
//      Raw count >= AD_AUX_THRESHOLD  →  HIGH  →  LCO signal asserted
//      Raw count <  AD_AUX_THRESHOLD  →  LOW   →  LCO signal idle
//      AUX input is a real-world LCO signal (~0 V idle, ~12 V triggered --
//      3S LiPo or lead-acid launch control system) brought down to a safe
//      ADS1015 input range through a resistor divider on the main board.
//      Threshold is set at 0.5 V at the ADS1015 pin (post-divider) --
//      calibrated against the actual divider hardware, gives good
//      separation from ground across all real input signal cases.
//
//    LCO watch (CAN-based; replaces the retired wired Remote-Start output):
//      When armed via CAN_ARM_LCO_WATCH(param=1), Check_LCO_Watch()
//      (main.cpp) samples this channel every loop() iteration; on crossing
//      AD_AUX_THRESHOLD it sends CAN_LCO_TRIGGERED to EMU (fire-and-forget)
//      and disarms itself (single-shot). See KJO_Shared_Libraries's
//      docs/superpowers/specs/2026-07-23-multi-source-ignition-design.md.
//

#include <Arduino.h>
#include "KJO_GPIO.h"

// ─── ADS1015 base scale ───────────────────────────────────────────────────────
// Hardware constant — identical for all units using GAIN_ONE (±4.096 V full scale).
// Value: 4.096 V / 2048 counts ≈ 0.00199144777 V/count.
constexpr float   AD_BASE_SCALE     = 0.00199144777f;   // V/count at GAIN_ONE

// ─── LiPo voltage divider scale — GSEMU-specific ─────────────────────────────
// Accounts for the actual resistor values in the GSEMU voltage divider network.
// ⚠ PLACEHOLDER: seeded from the EMU divider as a first approximation.
//   Measure the GSEMU divider resistors and recalculate before calibrated use:
//     GSEMU_LIPO_SCALE = (R_top + R_bottom) / R_bottom
constexpr float   GSEMU_LIPO_SCALE  = 2.79111f;         // ⚠ placeholder — calibrate before use

// ─── ADS1015 channel assignments ──────────────────────────────────────────────
constexpr uint8_t AD_LIPO_CHANNEL   = 3;    // Channel 3: 2S LiPo voltage divider
constexpr uint8_t AD_AUX_CHANNEL    = 0;    // Channel 0: AUX analog input

// ─── AUX input digital threshold ──────────────────────────────────────────────
// 0.5 V (post-divider, at the ADS1015 pin) expressed in counts at GAIN_ONE:
//   0.5 V / 0.00199144777 V/count ≈ 251 counts.
constexpr int16_t AD_AUX_THRESHOLD  = 251;  // counts — readings >= this are HIGH

// ─── Battery display interval ─────────────────────────────────────────────────
constexpr uint32_t BATTERY_DISPLAY_INTERVAL_MS = 10000; // ms — display battery voltage every 10 s

#endif // KJO_ANALOG_H
