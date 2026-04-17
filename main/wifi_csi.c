/**
 * @file wifi_csi.c
 * @brief Wi-Fi CSI capture engine with motion detection state machine.
 *
 * Flow:
 *   1. Connect to AP in STA mode.
 *   2. Register esp_wifi_set_csi_rx_cb() callback.
 *   3. Inject null data frames at ~50 Hz via a FreeRTOS timer to keep CSI
 *      flowing even when no real traffic is present.
 *   4. Each callback: extract per-subcarrier amplitudes, push into ring buffer,
 *      compute variance score, run state machine.
 *   5. csi_engine_get_status() returns a thread-safe snapshot for the WS layer.
 */

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "wifi_csi.h"

/* ───────── tunables ───────── */
#define CSI_RING_DEPTH      20      /* frames kept in history               */
#define CSI_MAX_SUBCARRIERS 128     /* max subcarriers from ESP32-S3        */
#define INJECT_PERIOD_MS    20      /* ~50 Hz null-frame injection           */
#define WIFI_CONNECT_RETRIES 5

static const char *TAG = "csi_engine";

/* ───────── ring buffer ───────── */
static float s_ring[CSI_RING_DEPTH][CSI_MAX_SUBCARRIERS];
static int   s_ring_head = 0;          /* next write position               */
static int   s_ring_fill = 0;          /* how many slots contain valid data */

/* ───────── state machine ───────── */
static float              s_threshold      = 20.0f;
static uint8_t            s_confirm_motion = 3;
static uint8_t            s_confirm_clear  = 10;
static csi_motion_state_t s_motion_state   = CSI_STATE_CLEAR;
static uint8_t            s_consec_above   = 0;
static uint8_t            s_consec_below   = 0;
static uint32_t           s_event_count    = 0;
static float              s_last_variance  = 0.0f;
static uint8_t            s_num_sc         = 0;
static float              s_latest_amps[CSI_MAX_SUBCARRIERS];

/* ───────── thread safety ───────── */
static SemaphoreHandle_t  s_mutex = NULL;

/* ───────── Wi-Fi events ───────── */
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_cnt = 0;
static char s_remote_addr[20] = "0.0.0.0";
static uint8_t s_null_frame[24] = {
    0x48, 0x01, 0x00, 0x00,                         /* Frame Control, Duration */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,             /* Receiver (Broadcast) */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,             /* Source (Our MAC) */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,             /* BSSID (Broadcast) */
    0x00, 0x00                                      /* Seq Num */
};

/* ──────────────────────────────────────────────────────
 * Internal helpers
 * ────────────────────────────────────────────────────── */

/**
 * Compute per-subcarrier variance across ring history, then average those
 * variances to get a single "motion score".
 */
static float compute_variance_score(uint8_t num_sc)
{
    if (s_ring_fill < 2 || num_sc == 0) return 0.0f;

    float total_var = 0.0f;

    for (int sc = 0; sc < (int)num_sc; sc++) {
        /* compute mean for this subcarrier */
        float sum = 0.0f;
        for (int i = 0; i < s_ring_fill; i++) {
            int idx = (s_ring_head - 1 - i + CSI_RING_DEPTH) % CSI_RING_DEPTH;
            sum += s_ring[idx][sc];
        }
        float mean = sum / (float)s_ring_fill;

        /* compute variance */
        float var_sum = 0.0f;
        for (int i = 0; i < s_ring_fill; i++) {
            int idx = (s_ring_head - 1 - i + CSI_RING_DEPTH) % CSI_RING_DEPTH;
            float diff = s_ring[idx][sc] - mean;
            var_sum += diff * diff;
        }
        total_var += var_sum / (float)s_ring_fill;
    }
    return total_var / (float)num_sc;
}

/**
 * CSI receive callback — called from the Wi-Fi driver task at ~50 Hz.
 */
static void csi_rx_callback(void *ctx, wifi_csi_info_t *csi_info)
{
    if (!csi_info || !csi_info->buf) return;

    int8_t  *raw  = csi_info->buf;
    uint32_t len  = csi_info->len;    /* bytes: 2 × num_subcarriers (I,Q pairs) */
    uint32_t num_sc = len / 2;
    if (num_sc > CSI_MAX_SUBCARRIERS) num_sc = CSI_MAX_SUBCARRIERS;

    /* ── compute amplitudes ── */
    float amps[CSI_MAX_SUBCARRIERS];
    for (uint32_t i = 0; i < num_sc; i++) {
        float I = (float)raw[2 * i];
        float Q = (float)raw[2 * i + 1];
        amps[i] = sqrtf(I * I + Q * Q);
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* push into ring */
    memcpy(s_ring[s_ring_head], amps, num_sc * sizeof(float));
    s_ring_head = (s_ring_head + 1) % CSI_RING_DEPTH;
    if (s_ring_fill < CSI_RING_DEPTH) s_ring_fill++;

    /* store latest snapshot */
    memcpy(s_latest_amps, amps, num_sc * sizeof(float));
    s_num_sc = (uint8_t)num_sc;

    /* ── compute score ── */
    float score = compute_variance_score((uint8_t)num_sc);
    s_last_variance = score;

    /* ── state machine ── */
    if (score > s_threshold) {
        s_consec_above++;
        s_consec_below = 0;
        if (s_motion_state == CSI_STATE_CLEAR &&
            s_consec_above >= s_confirm_motion) {
            s_motion_state = CSI_STATE_MOTION;
            s_event_count++;
            ESP_LOGI(TAG, "MOTION DETECTED (score=%.2f, event=%lu)",
                     score, (unsigned long)s_event_count);
        }
    } else {
        s_consec_below++;
        s_consec_above = 0;
        if (s_motion_state == CSI_STATE_MOTION &&
            s_consec_below >= s_confirm_clear) {
            s_motion_state = CSI_STATE_CLEAR;
            ESP_LOGI(TAG, "CLEAR (score=%.2f)", score);
        }
    }

    xSemaphoreGive(s_mutex);
}

/* ───────── null frame injection timer ───────── */
static TimerHandle_t s_inject_timer = NULL;

static void inject_timer_cb(TimerHandle_t xTimer)
{
    /* Sending a null data frame to the AP triggers CSI feedback */
    esp_wifi_80211_tx(WIFI_IF_STA, s_null_frame, sizeof(s_null_frame), false);
}

/* ───────── Wi-Fi event handler ───────── */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_cnt < WIFI_CONNECT_RETRIES) {
            esp_wifi_connect();
            s_retry_cnt++;
            ESP_LOGW(TAG, "Retrying Wi-Fi (%d/%d)…",
                     s_retry_cnt, WIFI_CONNECT_RETRIES);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Wi-Fi connection failed.");
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        snprintf(s_remote_addr, sizeof(s_remote_addr),
                 IPSTR, IP2STR(&event->ip_info.ip));
        
        /* Disable power save for high-frequency CSI and WebSocket stability */
        esp_wifi_set_ps(WIFI_PS_NONE);
        
        ESP_LOGW(TAG, "====================================");
        ESP_LOGW(TAG, "  DEVICE IS ONLINE: http://%s/", s_remote_addr);
        ESP_LOGW(TAG, "====================================");
        
        s_retry_cnt = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ──────────────────────────────────────────────────────
 * Public API
 * ────────────────────────────────────────────────────── */

esp_err_t csi_engine_init(const csi_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;

    s_threshold       = cfg->alert_threshold;
    s_confirm_motion  = cfg->motion_confirm_frames;
    s_confirm_clear   = cfg->clear_confirm_frames;

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    s_wifi_event_group = xEventGroupCreate();

    /* ── init netif + event loop ── */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = {};
    strlcpy((char *)wifi_cfg.sta.ssid,     cfg->ssid,     sizeof(wifi_cfg.sta.ssid));
    strlcpy((char *)wifi_cfg.sta.password,  cfg->password, sizeof(wifi_cfg.sta.password));
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_cfg.sta.pmf_cfg.capable    = true;
    wifi_cfg.sta.pmf_cfg.required   = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Prepare the injection frame with our MAC address */
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    memcpy(s_null_frame + 10, mac, 6);

    /* wait for connect or failure */
    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(15000));

    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGE(TAG, "Failed to connect to AP '%s'.", cfg->ssid);
        /* continue anyway so the WebSocket still comes up */
    }

    /* ── configure CSI ── */
    wifi_csi_config_t csi_cfg = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = true,
        .ltf_merge_en      = true,
        .channel_filter_en = true,
        .manu_scale        = false,
        .shift             = 0,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_rx_callback, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));

    /* ── start null-frame injection timer ── */
    s_inject_timer = xTimerCreate("csi_inject",
                                  pdMS_TO_TICKS(INJECT_PERIOD_MS),
                                  pdTRUE, NULL, inject_timer_cb);
    if (s_inject_timer) xTimerStart(s_inject_timer, 0);

    ESP_LOGI(TAG, "CSI engine started. Threshold=%.1f", s_threshold);
    return ESP_OK;
}

void csi_engine_get_status(csi_status_t *out)
{
    if (!out) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    out->motion_state    = s_motion_state;
    out->variance_score  = s_last_variance;
    out->num_subcarriers = s_num_sc;
    out->motion_event_count = s_event_count;
    memcpy(out->amplitudes, s_latest_amps, s_num_sc * sizeof(float));
    strlcpy(out->remote_addr, s_remote_addr, sizeof(out->remote_addr));
    xSemaphoreGive(s_mutex);
}

void csi_engine_recalibrate(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_ring_head    = 0;
    s_ring_fill    = 0;
    s_consec_above = 0;
    s_consec_below = 0;
    s_motion_state = CSI_STATE_CLEAR;
    s_last_variance= 0.0f;
    ESP_LOGI(TAG, "Recalibrated — baseline cleared.");
    xSemaphoreGive(s_mutex);
}

void csi_engine_set_threshold(float threshold)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_threshold = threshold;
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "Threshold updated to %.1f", threshold);
}
