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
- **Self-Healing Wi-Fi**: Infinite reconnection logic with 30s smart backoff ensures the device recovers automatically from network outages.
- **Micro-Latency**: Direct WebSocket streaming ensures minimal delay between physical movement and visual feedback.

---

## 🚥 LED Status Indicators (XIAO ESP32-S3)

The onboard orange LED (**GPIO 21**) provides real-time biometric and system feedback:

| State | LED Pattern | Description |
| :--- | :--- | :--- |
| **Connecting / Searching** | Slow Pulse | 500ms ON / 500ms OFF |
| **Disconnected / Error** | Rapid Flicker | 100ms ON / 100ms OFF |
| **Online (Still)** | Single Blip | Short 50ms flash every 2s (System Alive) |
| **Breathing Detected** | Double Blip | "Heartbeat" pattern (Biometric Pulse) |
| **Motion Detected** | **Steady ON** | Solid glow during active movement |

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

### Self-Healing Wi-Fi
The system is designed for 24/7 standalone operation. If the link is dropped:
- **Phase 1 (Fast)**: 5 attempts are made immediately.
- **Phase 2 (Patient)**: If persistent, the device waits **30 seconds** between retries indefinitely.
- **Visual Status**: The onboard LED remains in "Searching" mode (Slow Pulse) throughout this process to signal it is attempting recovery.

### Software Stack
- **Framework**: ESP-IDF (C-based)
- **Networking**: ESP-NETIF, FreeRTOS
- **Web Interface**: Vanilla JS + HTML5 Canvas (embedded in `dashboard.h`)
- **Communication**: HTTP Server with WebSocket protocols

---

## ⚖️ License
This project is open-source. Feel free to fork, modify, and improve the radar logic!
