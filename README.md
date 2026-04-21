# Wi-Fi CSI Presence Radar (v1.0 MVP)

Practical ESP32-S3 firmware for a Wi-Fi CSI radar focused on robust, location-agnostic presence sensing. 

This repository has been streamlined into a **3-state MVP** that prioritizes deployment stability over experimental detector bloat.

The firmware exposes three core states:
- `no_presence`
- `presence_static` (occupancy confirmed)
- `presence_motion` (active macro movement)

## Core Design Principles

1. **Location Agnostic**: Uses relative feature engineering (frame-mean normalization and self-calibrating baselines) to ensure consistent performance across different physical environments without manual code-level tuning.
2. **Simplified FSM**: A robust 3-state Finite State Machine (FSM) replaces the legacy 4-state and hybrid detectors. This reduces "state flickering" and improves classification reliability.
3. **Manual Guided Setup**: The 4-stage guided setup (Empty, Walk, Still, Clear) is now **manual-only**. This gives the user total control over sampling quality for site-specific threshold tuning.
4. **Lean Payload**: The WebSocket and JSON APIs have been purged of legacy telemetry, resulting in smaller payloads and better performance on the ESP32.

## Current Firmware Behavior

### Phase 1: Raw CSI Capture
The device emits host-loggable lines in this format:
```text
CSI_DATA_HEADER,<csv_column_names...>
CSI_DATA,<timestamp_us>,<rssi>,<num_subcarriers>,...,<iq_vector>
```

Recommended high-rate host capture:
```bash
node tools/capture_ws.mjs ws://<ESP32-IP>/ws/log session.csv
```

### Phase 2: Presence Sensing
The radar computes:
- Per-frame amplitude extraction and detrending.
- **Occupancy score**: Based on relative deviation from an empty-room baseline.
- **Motion score**: Macro-motion energy calculated over a weighted temporal window.
- **Stability score**: Identifies when presence is sustained but static.

### Phase 3: Guided Setup
To calibrate a new location, open the dashboard and use the **Guided Setup** section:
1. **Empty Room**: Learns the RF floor of the environment.
2. **Walk Test**: Samples peak macro-motion to set the motion threshold.
3. **Still Test**: Samples occupancy deviation while a person is stationary.
4. **Clear Test**: Validates the system returns to the baseline.
5. **Apply**: One-click application of suggested thresholds to the active profile.

*Note: All stages now require the user to manually click "Start Sampling" to ensure data is only collected during active testing.*

## Dedicated AP Mode
For the cleanest CSI data, use two ESP32s: one as a `dedicated_ap` (traffic source) and one as a `sensor_sta` (receiver/processor).
- **AP Board**: Provides a stable RF environment and periodic broadcast traffic.
- **Sensor Board**: Performs all DSP and presence classification.

## Dashboard
Open `http://<ESP32-IP>/` to access the live dashboard:
- **Live Waterfall**: Visualize CSI amplitude changes in realtime.
- **State Indicator**: Large status pill showing the current FSM state.
- **Diagnostic Grid**: Realtime scores for Occupancy, Motion, and Stability.
- **Tuning Controls**: Manual overrides for thresholds and notch filters (for fan noise).

## Build
Update the role and Wi-Fi settings in `main/main.c`, then build with ESP-IDF:
```bash
idf.py set-target esp32s3
idf.py build flash monitor
```

## Limitations
- **Single-Device Sensing**: Better at robust occupancy states than precise direction or counting.
- **Environment Sensitivity**: While relative features help, significant physical changes (moving furniture/routers) still require a quick recalibration.
