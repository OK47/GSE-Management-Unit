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
//      raw count >= AD_AUX_THRESHOLD  →  HIGH  →  LCO signal asserted
//      raw count <  AD_AUX_THRESHOLD  →  LOW   →  LCO signal idle
//      AUX input is a real-world LCO signal (~0 V idle, ~12 V triggered --
//      3S LiPo or lead-acid launch control system) brought down to a safe
//      ADS1015 input range through a resistor divider on the main board.
//      Threshold is set at 0.5 V at the ADS1015 pin (post-divider) --
//      calibrated against the actual divider hardware, gives good
//      separation from ground across all real input signal cases.
//
//      POLARITY (2026-08): a bare unsigned comparison is correct here, not
//      abs(raw count). An abs()-based check sat here previously on the
//      assumption that a reversed firing-lead connection would still
//      register a usable negative-going reading. Bench characterization
//      (2026-08-08/09) showed that assumption was wrong: the ADS1015's own
//      input protection diodes clamp a reversed-polarity input to within
//      roughly a diode drop of GND (a few hundred mV, chip-to-chip
//      variable) well before the divider can pull it further negative --
//      abs() was never a reliable safety net, since whether the clamped
//      voltage happened to clear AD_AUX_THRESHOLD was essentially a coin
//      flip. Firing-lead polarity is now verified once at installation (a
//      simple bench continuity/polarity check, no live fire required) and
//      wired correctly, so the signal is guaranteed positive-going here. A
//      future accidental reversal will fail silently (never crosses
//      threshold) rather than being caught -- an accepted tradeoff given
//      the one-time verification step.
//
//    LCO watch (CAN-based; replaces the retired wired Remote-Start output):
//      When armed via CAN_ARM_LCO_WATCH(param=1), Check_LCO_Watch()
//      (main.cpp) samples this channel every loop() iteration; on crossing
//      AD_AUX_THRESHOLD it sends CAN_LCO_TRIGGERED to EMU (fire-and-forget)
//      and disarms itself (single-shot). See KJO_Shared_Libraries'
//      docs/superpowers/specs/2026-07-23-multi-source-ignition-design.md.
//

#include <Arduino.h>
#include "KJO_GPIO.h"
#include "KJO_LCO_Sense.h"

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
// CENTRALIZED (2026-09): AD_AUX_CHANNEL/AD_AUX_THRESHOLD now come from the
// shared KJO_LCO_Sense.h (KJO_Shared_Libraries) instead of being an
// independently-maintained local copy that merely documented an expected
// match with EMU's AD_AIRSTART_CHANNEL/AIRSTART_THRESHOLD -- see that
// header for the full rationale. Local names kept as aliases so no other
// code in this file/main.cpp needed to change.
constexpr uint8_t AD_AUX_CHANNEL    = LCO_SENSE_ADS_CHANNEL;   // Channel 0: AUX analog input

// ─── AUX input digital threshold ──────────────────────────────────────────────
constexpr int16_t AD_AUX_THRESHOLD  = LCO_SENSE_THRESHOLD;  // counts — readings >= this are HIGH

// ─── Battery display interval ─────────────────────────────────────────────────
constexpr uint32_t BATTERY_DISPLAY_INTERVAL_MS = 10000; // ms — display battery voltage every 10 s

#endif // KJO_ANALOG_H
