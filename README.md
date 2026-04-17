# 📡 Wi-Fi CSI Human Radar

> **Privacy-first human motion detection using Wi-Fi CSI on ESP32-S3.**

Detects physical movement in an environment through signal disturbances—without cameras, microphones, or dedicated PIR sensors. This project leverages **Wi-Fi Channel State Information (CSI)** to provide high-precision, edge-based occupancy and motion sensing.

---

## 🚀 Overview

The **Wi-Fi CSI Human Radar** transforms a standard ESP32-S3 into a sophisticated environment sensor. By analyzing the subcarrier amplitudes of Wi-Fi packets (CSI), the system calculates signal variance triggers when human movement disrupts the multipath propagation.

### 🌟 Key Features
- **Privacy-Preserving**: No visual or audio data is captured. Detection is based entirely on radio frequency disturbances.
- **Edge Intelligence**: All signal processing and motion detection algorithms run locally on the ESP32-S3.
- **Real-Time Dashboard**: An embedded high-performance WebSocket dashboard featuring:
  - **Polar Radar Sweep**: Dynamic visual representation of motion events.
  - **CSI Waveform**: Live visualization of subcarrier amplitudes.
  - **HUD (Heads-Up Display)**: Real-time status, variance scores, and event logging.
- **Tunable Logic**: Adjust sensitivity thresholds and frame confirmation counts at runtime via the web UI.
- **Micro-Latency**: Direct WebSocket streaming ensures minimal delay between physical movement and visual feedback.

---

## 🛠 Hardware Required

- **ESP32-S3 Development Board** (Required for CSI support on internal Wi-Fi stack).
- A stable 2.4GHz Wi-Fi Access Point (AP).

---

## 🚦 Quick Start

### 1. Configure Wi-Fi
Open [main.c](main/main.c) and update your Wi-Fi credentials:

```c
#define WIFI_SSID     "YOUR_SSID"
#define WIFI_PASSWORD "YOUR_PASSWORD"
```

### 2. Build and Flash
Ensure you have the [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) environment set up.

```bash
# Set target to ESP32-S3
idf.py set-target esp32s3

# Build and flash the project
idf.py build flash monitor
```

### 3. Access Dashboard
Once flashed, find the device's IP address in the serial monitor and open it in your browser:
`http://<ESP32-IP>/`

---

## 🧠 Technical Details

### CSI Engine
The system captures CSI data from incoming Wi-Fi packets. It focuses on the **Amplitude** of the subcarriers. A ring buffer stores historical amplitudes to calculate a standard deviation (variance) score. When the variance exceeds the `ALERT_THRESHOLD`, a motion state is triggered.

### Software Stack
- **Framework**: ESP-IDF (C-based)
- **Networking**: ESP-NETIF, FreeRTOS
- **Web Interface**: Vanilla JS + HTML5 Canvas (embedded in `dashboard.h`)
- **Communication**: HTTP Server with WebSocket protocols

---

## ⚖️ License
This project is open-source. Feel free to fork, modify, and improve the radar logic!
