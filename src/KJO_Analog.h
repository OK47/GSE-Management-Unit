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
//      Raw count >= AD_AUX_THRESHOLD  →  HIGH  →  Remote Start asserted (if Launch Enabled)
//      Raw count <  AD_AUX_THRESHOLD  →  LOW   →  Remote Start de-asserted
//      Threshold is set at half of 3.3 V full-scale ≈ 1.65 V.
//
//    Remote Start output:
//      The REMOTE_START_EIO pin (I/O 6) is driven LOW only when BOTH:
//        - AUX input is HIGH (above threshold)
//        - LAUNCH_ENABLE_EIO (I/O 5) reads LOW (EMU has enabled remote start)
//      Both pins are defined in KJO_GPIO.h.
//

#include <Arduino.h>
#include "KJO_GPIO.h"   // REMOTE_START_EIO, LAUNCH_ENABLE_EIO

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
// Half of 3.3 V expressed in ADS1015 counts at GAIN_ONE:
//   1.65 V / 0.00199144777 V/count ≈ 829 counts.
constexpr int16_t AD_AUX_THRESHOLD  = 829;  // counts — readings >= this are HIGH

// ─── Battery display interval ─────────────────────────────────────────────────
constexpr uint32_t BATTERY_DISPLAY_INTERVAL_MS = 10000; // ms — display battery voltage every 10 s

#endif // KJO_ANALOG_H
