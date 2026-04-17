/**
 * @file main.c
 * @brief Wi-Fi CSI Human Radar — application entry point.
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  CONFIGURE YOUR WI-FI CREDENTIALS BELOW BEFORE FLASHING        │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * After flashing, open a browser at http://<ESP32-IP>/ to view the
 * radar dashboard.  The device IP is printed on the serial monitor.
 */

#include <stdio.h>
#include "esp_log.h"
#include "esp_idf_version.h"
#include "nvs_flash.h"
#include "wifi_csi.h"
#include "http_server.h"

static const char *TAG = "main";

/* ╔═══════════════════════════════════════╗
   ║  ▶  SET YOUR WI-FI CREDENTIALS HERE  ║
   ╚═══════════════════════════════════════╝ */
#define WIFI_SSID     "Stacksr"
#define WIFI_PASSWORD "Stacksr@7276"

/* ── Motion detection tuning ─────────────────────────── */
#define ALERT_THRESHOLD        20.0f  /* variance score → motion            */
#define MOTION_CONFIRM_FRAMES  3      /* consecutive hits before MOTION flag */
#define CLEAR_CONFIRM_FRAMES   10     /* consecutive clears before CLEAR flag */

void app_main(void)
{
    ESP_LOGI(TAG, "=== Wi-Fi CSI Human Radar ===");
    ESP_LOGI(TAG, "ESP-IDF version: %s", esp_get_idf_version());

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
        .ssid                = WIFI_SSID,
        .password            = WIFI_PASSWORD,
        .alert_threshold     = ALERT_THRESHOLD,
        .motion_confirm_frames = MOTION_CONFIRM_FRAMES,
        .clear_confirm_frames  = CLEAR_CONFIRM_FRAMES,
    };
    ESP_ERROR_CHECK(csi_engine_init(&csi_cfg));

    /* ── 3. Start HTTP + WebSocket server ── */
    ESP_ERROR_CHECK(http_server_start());

    ESP_LOGI(TAG, "System ready. Open http://<device-ip>/ in your browser.");
}
