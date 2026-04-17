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
#include "driver/gpio.h"
#include "wifi_csi.h"

/* ───────── tunables ───────── */
#define CSI_RING_DEPTH      100     /* ~2.0s history at 50Hz                */
#define CSI_MAX_SUBCARRIERS 128     /* max subcarriers from ESP32-S3        */
#define INJECT_PERIOD_MS    20      /* ~50 Hz null-frame injection           */
#define WIFI_CONNECT_RETRIES 5
#define BOARD_LED_GPIO      21      /* XIAO ESP32-S3 User LED pin            */

static const char *TAG = "csi_engine";

/* ───────── ring buffer ───────── */
static float s_ring_amp[CSI_RING_DEPTH][CSI_MAX_SUBCARRIERS];
static float s_ring_phase[CSI_RING_DEPTH][CSI_MAX_SUBCARRIERS];
static int   s_ring_head = 0;          /* next write position               */
static int   s_ring_fill = 0;          /* how many slots contain valid data */

/* ───────── state machine ───────── */
static float              s_threshold      = 20.0f;
static uint8_t            s_confirm_motion = 3;
static uint8_t            s_confirm_clear  = 10;
static csi_motion_state_t s_motion_state   = CSI_STATE_CLEAR;
static csi_activity_t     s_activity       = CSI_ACT_STILL;
static csi_direction_t    s_direction      = CSI_DIR_NONE;

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
static TimerHandle_t s_reconnect_timer = NULL;
static char s_remote_addr[20] = "0.0.0.0";
static bool s_wifi_is_error = false;
static bool s_wifi_is_connected = false;

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
 * Compute per-subcarrier variance across ring history.
 * Robust enhancement: only uses subcarriers 6-26 and 38-58 (stable parts of BW20).
 */
static float compute_variance_score(uint8_t num_sc)
{
    if (s_ring_fill < 10 || num_sc == 0) return 0.0f;

    float total_var = 0.0f;
    int   valid_sc  = 0;

    for (int sc = 0; sc < (int)num_sc; sc++) {
        /* Filter out DC, edges, and pilot subcarriers for robustness */
        if (sc < 6 || sc == 32 || sc > 58) continue;

        float sum = 0.0f;
        for (int i = 0; i < s_ring_fill; i++) {
            sum += s_ring_amp[i][sc];
        }
        float mean = sum / (float)s_ring_fill;

        float var_sum = 0.0f;
        for (int i = 0; i < s_ring_fill; i++) {
            float diff = s_ring_amp[i][sc] - mean;
            var_sum += diff * diff;
        }
        total_var += var_sum / (float)s_ring_fill;
        valid_sc++;
    }
    return (valid_sc > 0) ? (total_var / (float)valid_sc) : 0.0f;
}

/**
 * Detect direction by analyzing phase shifts across the last few frames.
 */
static csi_direction_t detect_direction(uint8_t num_sc)
{
    if (s_ring_fill < 5) return CSI_DIR_NONE;

    float phase_delta_sum = 0.0f;
    int   samples = 0;

    int cur = (s_ring_head - 1 + CSI_RING_DEPTH) % CSI_RING_DEPTH;
    int pre = (s_ring_head - 2 + CSI_RING_DEPTH) % CSI_RING_DEPTH;

    for (int sc = 6; sc < (int)num_sc && sc <= 58; sc++) {
        if (sc == 32) continue;
        float diff = s_ring_phase[cur][sc] - s_ring_phase[pre][sc];
        /* Normalize to -PI to PI */
        while (diff >  M_PI) diff -= 2.0f * M_PI;
        while (diff < -M_PI) diff += 2.0f * M_PI;
        phase_delta_sum += diff;
        samples++;
    }

    float avg_delta = (samples > 0) ? (phase_delta_sum / (float)samples) : 0.0f;
    
    if (avg_delta > 0.05f) return CSI_DIR_TOWARD;
    if (avg_delta < -0.05f) return CSI_DIR_AWAY;
    return CSI_DIR_NONE;
}

/**
 * CSI receive callback — called from the Wi-Fi driver task at ~50 Hz.
 */
static void csi_rx_callback(void *ctx, wifi_csi_info_t *csi_info)
{
    if (!csi_info || !csi_info->buf) return;

    int8_t  *raw  = csi_info->buf;
    uint32_t len  = csi_info->len;
    uint32_t num_sc = len / 2;
    if (num_sc > CSI_MAX_SUBCARRIERS) num_sc = CSI_MAX_SUBCARRIERS;

    float amps[CSI_MAX_SUBCARRIERS] = {0};
    float phases[CSI_MAX_SUBCARRIERS] = {0};

    for (uint32_t i = 0; i < num_sc; i++) {
        float I = (float)raw[2 * i];
        float Q = (float)raw[2 * i + 1];
        amps[i] = sqrtf(I * I + Q * Q);
        phases[i] = atan2f(Q, I);
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);

    /* push into ring */
    memcpy(s_ring_amp[s_ring_head], amps, num_sc * sizeof(float));
    memcpy(s_ring_phase[s_ring_head], phases, num_sc * sizeof(float));
    s_ring_head = (s_ring_head + 1) % CSI_RING_DEPTH;
    if (s_ring_fill < CSI_RING_DEPTH) s_ring_fill++;

    /* store latest snapshot for UI */
    memcpy(s_latest_amps, amps, num_sc * sizeof(float));
    s_num_sc = (uint8_t)num_sc;

    /* ── compute movement score ── */
    float score = compute_variance_score((uint8_t)num_sc);
    s_last_variance = score;

    /* ── detect direction ── */
    s_direction = detect_direction((uint8_t)num_sc);

    /* ── activity classification ── */
    if (score > s_threshold) {
        s_activity = CSI_ACT_MOVING;
    } else if (score > 1.2f) { /* Threshold for breathing-level disturbance */
        s_activity = CSI_ACT_BREATHING;
    } else {
        s_activity = CSI_ACT_STILL;
    }

    /* ── motion state machine (hysteresis) ── */
    if (score > s_threshold) {
        s_consec_above++;
        s_consec_below = 0;
        if (s_motion_state == CSI_STATE_CLEAR && s_consec_above >= s_confirm_motion) {
            s_motion_state = CSI_STATE_MOTION;
            s_event_count++;
        }
    } else {
        s_consec_below++;
        s_consec_above = 0;
        if (s_motion_state == CSI_STATE_MOTION && s_consec_below >= s_confirm_clear) {
            s_motion_state = CSI_STATE_CLEAR;
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
        s_wifi_is_connected = false;
        s_wifi_is_error = false; /* Still trying, so not a permanent error */
        
        if (s_retry_cnt < 5) {
            esp_wifi_connect();
            s_retry_cnt++;
            ESP_LOGW(TAG, "Reconnecting (%d/5)...", s_retry_cnt);
        } else {
            /* Enter backoff mode to avoid spamming the router */
            ESP_LOGW(TAG, "Connection failed 5 times. Waiting 30s before next attempt...");
            if (s_reconnect_timer) xTimerStart(s_reconnect_timer, 0);
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
        s_wifi_is_connected = true;
        s_wifi_is_error = false;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ──────────────────────────────────────────────────────
 * Public API
 * ────────────────────────────────────────────────────── */

/* ───────── status led task ───────── */
static void reconnect_timer_cb(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "Backoff over. Retrying connection...");
    esp_wifi_connect();
}

static void led_status_task(void *arg)
{
    gpio_reset_pin(BOARD_LED_GPIO);
    gpio_set_direction(BOARD_LED_GPIO, GPIO_MODE_OUTPUT);
    
    while (1) {
        if (!s_wifi_is_connected) {
            if (s_wifi_is_error) {
                /* Fast flicker - Error / Disconnected permanently */
                gpio_set_level(BOARD_LED_GPIO, 0); vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(BOARD_LED_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(100));
            } else {
                /* Slow pulse - Connecting / Searching */
                gpio_set_level(BOARD_LED_GPIO, 0); vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(BOARD_LED_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(500));
            }
        } else {
            /* ONLINE - Check activity */
            if (s_motion_state == CSI_STATE_MOTION) {
                /* ALERT - Steady ON */
                gpio_set_level(BOARD_LED_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(200));
            } else if (s_activity == CSI_ACT_BREATHING) {
                /* BREATHING detected - Heartbeat Pattern */
                gpio_set_level(BOARD_LED_GPIO, 0); vTaskDelay(pdMS_TO_TICKS(50));
                gpio_set_level(BOARD_LED_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(150));
                gpio_set_level(BOARD_LED_GPIO, 0); vTaskDelay(pdMS_TO_TICKS(50));
                gpio_set_level(BOARD_LED_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(1750));
            } else {
                /* STILL / Processing - Single blip "Alive" signal */
                gpio_set_level(BOARD_LED_GPIO, 0); vTaskDelay(pdMS_TO_TICKS(50));
                gpio_set_level(BOARD_LED_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(1950));
            }
        }
    }
}

esp_err_t csi_engine_init(const csi_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;

    s_threshold       = cfg->alert_threshold;
    s_confirm_motion  = cfg->motion_confirm_frames;
    s_confirm_clear   = cfg->clear_confirm_frames;

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    s_wifi_event_group = xEventGroupCreate();
    s_reconnect_timer = xTimerCreate("wifi_reconnect", pdMS_TO_TICKS(30000),
                                     pdFALSE, NULL, reconnect_timer_cb);

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

    /* ── start status led task ── */
    xTaskCreate(led_status_task, "led_status", 3072, NULL, 5, NULL);

    ESP_LOGI(TAG, "CSI engine started. Threshold=%.1f", s_threshold);
    return ESP_OK;
}

void csi_engine_get_status(csi_status_t *out)
{
    if (!out) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    out->motion_state    = s_motion_state;
    out->activity        = s_activity;
    out->direction       = s_direction;
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
