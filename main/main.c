/**
 * @file main.c
 * @brief Wi-Fi CSI Presence Radar — application entry point.
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  CONFIGURE YOUR WI-FI CREDENTIALS BELOW BEFORE FLASHING        │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * After flashing, open a browser at http://<ESP32-IP>/ to view the
 * radar dashboard. The device IP is printed on the serial monitor.
 */

#include "esp_idf_version.h"
#include "esp_log.h"
#include "http_server.h"
#include "nvs_flash.h"
#include "wifi_csi.h"
#include <stdbool.h>
#include <stdio.h>

static const char *TAG = "main";

/* ╔══════════════════════════════════════════════════════╗
   ║  ▶  PICK A ROLE AND SET ITS WIFI CREDENTIALS HERE   ║
   ╚══════════════════════════════════════════════════════╝

   Flash one ESP32 as `CSI_WIFI_ROLE_SENSOR_STA`.
   Flash the other ESP32 as `CSI_WIFI_ROLE_DEDICATED_AP`.
*/
#define DEVICE_ROLE CSI_WIFI_ROLE_SENSOR_STA
#define WIFI_SSID "CSI-LAB"
#define WIFI_PASSWORD "csi-lab-123"
#define WIFI_AP_CHANNEL 6
#define AP_BROADCAST_INTERVAL_MS 20

/* ── Presence radar tuning ───────────────────────────── */
#define MOTION_THRESHOLD 1.05f    /* macro-motion score threshold         */
#define MOTION_CONFIRM_FRAMES 8   /* consecutive hits before motion state */
#define CLEAR_CONFIRM_FRAMES 96   /* consecutive quiet frames before clear */
#define RAW_LOGGING_ENABLED false /* phase-1 CSI_DATA serial stream       */

void app_main(void) {
  ESP_LOGI(TAG, "=== Wi-Fi CSI Presence Radar ===");
  ESP_LOGI(TAG, "ESP-IDF version: %s", esp_get_idf_version());
  ESP_LOGI(TAG, "Configured role: %s", csi_wifi_role_to_string(DEVICE_ROLE));

  /* ── 1. Init NVS (required by Wi-Fi driver) ── */
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "NVS needs erase — erasing…");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  }

  /* ── 2. Start CSI engine (Wi-Fi STA + CSI capture) ── */
  const csi_config_t csi_cfg = {
      .ssid = WIFI_SSID,
      .password = WIFI_PASSWORD,
      .motion_threshold = MOTION_THRESHOLD,
      .motion_confirm_frames = MOTION_CONFIRM_FRAMES,
      .clear_confirm_frames = CLEAR_CONFIRM_FRAMES,
      .raw_logging_enabled = RAW_LOGGING_ENABLED,
      .wifi_role = DEVICE_ROLE,
      .ap_channel = WIFI_AP_CHANNEL,
      .ap_broadcast_interval_ms = AP_BROADCAST_INTERVAL_MS,
  };
  ESP_ERROR_CHECK(csi_engine_init(&csi_cfg));

  /* ── 3. Start HTTP + WebSocket server ── */
  ESP_ERROR_CHECK(http_server_start());

  ESP_LOGI(TAG, "System ready. Open http://<device-ip>/ in your browser.");
}
