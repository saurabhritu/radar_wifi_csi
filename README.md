# Wi-Fi CSI Presence Radar

ESP32-S3 firmware for a practical Wi-Fi CSI radar that follows the development path you described:

1. prove raw CSI capture
2. ship presence detection first
3. separate micromotion from macro motion
4. package it as a home-friendly sensor

The firmware now exposes the end-goal states directly:

- `no_presence`
- `presence_static`
- `presence_micromotion`
- `presence_motion`

## What Was Taken From `Awesome-WiFi-CSI-Sensing`

The NTUMARS list is most useful here as a map of the space rather than a single implementation. For this repo, the most actionable sublinks were:

- [`ESP32-CSI-Tool`](https://github.com/StevenMHernandez/ESP32-CSI-Tool)
  It shows the fastest path to phase 1: emit `CSI_DATA` lines over serial and log on the host PC.
- [`espressif/esp-csi`](https://github.com/espressif/esp-csi)
  Its `get-started` examples validate the ESP32 capture path, and the `esp-radar` examples show how Espressif productizes CSI sensing.
- [`esp_wifi_sensing`](https://components.espressif.com/components/espressif/esp_wifi_sensing/versions/0.1.0/readme)
  The useful product ideas are dynamic baseline tracking, debounce/hysteresis, ping-assisted sampling, and diagnostics-friendly state output.
- The occupancy-detection section of the awesome list
  That section reinforces that occupancy/presence is the right first robust milestone before trying harder activity recognition tasks.

This implementation borrows those ideas directly:

- raw per-packet serial logging for capture validation
- queued CSI processing instead of heavy work in the Wi-Fi callback
- empty-room calibration plus slow drift compensation
- hysteretic FSM output rather than a single raw threshold

## Current Firmware Behavior

### Phase 1: Raw CSI Capture

The device emits host-loggable lines in this format:

```text
CSI_DATA_HEADER,<csv_column_names...>
CSI_DATA,<timestamp_us>,<rssi>,<num_subcarriers>,...,<iq_vector>
```

Each packet includes:

- timestamp
- RSSI
- packet rate and subcarrier counts
- raw energies and normalization references
- live thresholds and FSM counters
- CSI vector
- current radar state and scores

Serial debug capture example:

```bash
idf.py monitor | grep "CSI_DATA" > csi_session.csv
```

Recommended high-rate host capture:

```bash
node tools/capture_ws.mjs ws://<ESP32-IP>/ws/log session.csv
```

The firmware now exposes three capture paths:

- `ws://<ESP32-IP>/ws/log`
  Dedicated high-rate CSV log stream for serious experiments. Use this with `tools/capture_ws.mjs`.
- browser capture
  Open the web UI, click `Start Browser Capture (light)`, then `Stop Browser Capture & Download`. This is intentionally decimated and chunked so it does not compete with dashboard responsiveness.
- serial logging
  Useful for quick debugging or when you already have USB attached, but it is no longer the recommended path for high-rate sessions.

You can now stage a session in this order:

1. start `node tools/capture_ws.mjs ws://<ESP32-IP>/ws/log session.csv` on your computer if you want a full-rate capture
2. turn on raw logging if you also want serial debug output
3. click `Start Browser Capture (light)` only if you want a convenience browser-downloaded CSV
4. click `Start Detection` to begin a fresh calibration and live classification

Notes:

- `raw_logging_enabled` is off by default.
- If serial throughput becomes the bottleneck, set a high monitor baud rate such as `921600`.
- The firmware now supports two roles: `sensor_sta` and `dedicated_ap`.
- The `/ws/log` path is designed so if the host logger falls behind, rows for that sink are dropped instead of stalling CSI processing or the UI.

### Phase 2: Presence First

The radar computes:

- per-frame amplitude extraction
- frame mean normalization and detrending
- subcarrier filtering
- short/slow feature tracking
- occupancy score against an empty-room baseline
- motion energy over time
- thresholding plus hysteresis

This yields the first meaningful output:

- `no_presence`
- `presence_static`

### Phase 3: Micromotion

Micromotion is separated from macro motion using:

- short-window temporal energy
- macro-motion score with peak weighting
- stability score
- persistence counters

This extends the classifier to:

- `presence_micromotion`
- `presence_motion`

### Parallel Hybrid Detector

The firmware also runs a realtime three-state hybrid detector in parallel with
the original FSM. It uses a fast `motion_score` gate for `presence_motion`, then
uses frame-normalized selected-subcarrier `shift_score` for static presence and
clear-room recovery. Micromotion remains telemetry only in this path because
the recent captures show selected micro energy is still noisier than selected
shift.

The realtime hybrid detector uses these starting gates:

- motion enter: `motion_score >= 0.90` or `rhythm_motion_score >= 1.45` for 5 frames
- static enter: `selected_shift_score >= 0.62` for 8 frames
- clear: `occupancy_score <= 0.55`, `motion_score <= 0.85`, and `rhythm_motion_score <= 1.05` for 28 frames
- motion hold: 20 frames after motion to reduce flicker
- static hold: 45 frames after static evidence so still presence remains visible

It also exposes visual-inspired telemetry:

- `shape_shift_score`: full-envelope shape difference from empty-room baseline, telemetry only
- `rhythm_motion_score`: rolling normalized envelope movement for walking-like motion
- `patch_activity_score`: localized heatmap-like activity
- `micro_activity`: experimental flag when selected micro energy persists during static presence

Hybrid telemetry is exposed in the dashboard and CSV as:

- `hybrid_presence_state`
- `hybrid_motion_frames`
- `hybrid_micro_frames`
- `hybrid_static_frames`
- `hybrid_clear_frames`
- `hybrid_motion_hold_frames`
- `hybrid_static_hold_frames`

Use `hybrid_presence_state` as the practical realtime state while tuning. Keep
the primary `presence_state` beside it as a diagnostic reference for the older
four-state FSM.

### Phase 4: Productization

The firmware adds:

- manual empty-room recalibration
- a calibration guard that blocks baseline learning if the scene looks occupied
- a `Force calibration` override for intentional occupied-scene calibration
- slow drift compensation only when the room looks quiet
- disturbance counter for low-confidence events like fans/doors/router traffic
- dashboard controls for motion threshold, notch filtering, and raw logging
- a dedicated ESP32 SoftAP mode that emits steady UDP broadcast traffic for cleaner CSI sessions

## Dedicated AP Mode

If your home router is busy with TV streaming, phones, or other devices, a second ESP32 running as a dedicated AP will usually work better for CSI experiments.

Why it helps:

- fixed channel and simpler RF environment
- fewer unrelated packets and less traffic burstiness
- more repeatable packet cadence
- easier placement because only the sensor ESP32 joins that AP

The improvement is usually strongest for:

- more stable packet rate
- cleaner calibration
- easier threshold tuning

It will not solve everything by itself. Geometry still matters, and static presence is still harder than obvious motion.

To use it:

1. In [`main/main.c`](/Users/saurabhritu/Code/radar_wifi_csi/main/main.c), flash one board with `DEVICE_ROLE = CSI_WIFI_ROLE_DEDICATED_AP`.
2. Flash the sensing board with `DEVICE_ROLE = CSI_WIFI_ROLE_SENSOR_STA`.
3. Use the same `WIFI_SSID` and `WIFI_PASSWORD` on both boards.
4. Keep `WIFI_AP_CHANNEL` fixed, then place the two boards a few meters apart.
5. Open the sensor board dashboard, start capture, then start detection.

The AP board serves a small status page too, but it does not run detection. Its job is to provide a clean Wi-Fi source plus periodic broadcast packets for the sensor board.

## Dashboard

Open:

```text
http://<ESP32-IP>/
```

The dashboard shows:

- current 4-state radar output
- guided setup quality, deployment readiness, and the main limiter for the current location
- guided setup stages for empty-room, walk-through, still-presence, and clear-room validation
- automatic transition settling plus optional manual sampling for each guided setup stage
- suggested motion and presence thresholds from the guided setup run, with one-click profile application
- downgraded `motion_only` guided profiles when static presence is not separable from clear-room samples
- active motion, micromotion, and presence threshold controls for inspecting or overriding the profile
- parallel selected-subcarrier detector telemetry for shift, macro, and micro features
- parallel hybrid detector output using active motion thresholds plus selected-subcarrier shift
- occupancy, micromotion, motion, and stability scores
- calibration progress
- capture mode status for browser, serial, and local WebSocket logging
- live subcarrier amplitude envelope
- a rolling waterfall heatmap
- disturbance and presence event counts
- packet-rate jitter, RSSI range, usable subcarriers, and CSI queue drops

## Build

Update the role and Wi-Fi settings in [`main/main.c`](/Users/saurabhritu/Code/radar_wifi_csi/main/main.c), then build with ESP-IDF:

```bash
idf.py set-target esp32s3
idf.py build flash monitor
```

## Tuning Guidance

- Start with an empty-room recalibration.
- Leave `motion_threshold` around `1.35` for first tests.
- If ceiling fans or periodic motion create false alarms, use the notch slider.
- Validate the phase-1 capture path first by collecting `CSI_DATA` logs on the host.
- Only after presence is stable should you spend time tuning micromotion sensitivity.

## Limitations

- This is still single-device ESP32 CSI sensing, so it is better at robust occupancy states than precise direction, counting, or pose estimation.
- Host-PC logging is implemented; SD-card logging is not yet added in this repo.
- Thresholds still need room-by-room tuning on real hardware.
