/**
 * @file wifi_csi.c
 * @brief Wi-Fi CSI capture engine with four-state presence radar logic.
 *
 * This implementation follows the practical path suggested by the ESP32 CSI
 * Toolkit and Espressif's esp-csi / esp-radar material:
 *
 * 1. Keep CSI capture lightweight in the Wi-Fi callback and push packets into a
 *    worker queue.
 * 2. Preserve a phase-1 deliverable by streaming raw `CSI_DATA,...` lines over
 *    serial so a host PC can log timestamp + RSSI + CSI vectors per packet.
 * 3. Learn an empty-room baseline, compensate drift only when the scene looks
 *    quiet, and expose a product-friendly four-state FSM:
 *       no_presence
 *       presence_static
 *       presence_micromotion
 *       presence_motion
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "http_server.h"
#include "wifi_csi.h"

/* ───────── tunables ───────── */
#define INJECT_PERIOD_MS                 20
#define WIFI_CONNECT_RETRIES             5
#define BOARD_LED_GPIO                   21
#define CSI_QUEUE_LEN                    24
#define CSI_MIN_EFFECTIVE_SUBCARRIERS    12
#define CSI_FRAME_DROP_RSSI             -88
#define CSI_FAST_FEATURE_ALPHA         0.42f
#define CSI_SLOW_FEATURE_ALPHA         0.08f
#define CSI_DISPLAY_ALPHA              0.18f
#define CSI_EMPTY_BASELINE_FAST_ALPHA  0.18f
#define CSI_EMPTY_BASELINE_DRIFT_ALPHA 0.015f
#define CSI_EMPTY_BASELINE_HOLD_ALPHA  0.0015f
#define CSI_EMPTY_FLOOR_ALPHA          0.08f
#define CSI_EMPTY_JITTER_ALPHA         0.10f
#define CSI_MOTION_FLOOR_ALPHA         0.08f
#define CSI_MOTION_JITTER_ALPHA        0.10f
#define CSI_SCORE_ALPHA                0.24f
#define CSI_STABILITY_ALPHA            0.25f
#define CSI_OCCUPANCY_JITTER_GAIN      2.00f
#define CSI_MOTION_JITTER_GAIN         1.80f
#define CSI_MIN_OCCUPANCY_REF          0.30f
#define CSI_MIN_MOTION_REF             0.20f
#define CSI_BASELINE_CALIBRATION_FRAMES 80U
#define CSI_STATIC_CONFIRM_FRAMES       16U
#define CSI_MICRO_CONFIRM_FRAMES        10U
#define CSI_PRESENCE_HOLD_FRAMES       220U
#define CSI_DISTURBANCE_COOLDOWN_FRAMES 30U
#define CSI_CALIBRATION_GUARD_COOLDOWN_FRAMES 20U
#define CSI_DISPLAY_SUBCARRIERS         CSI_MAX_SUBCARRIERS
#define CSI_CSV_ROW_MAX               3584U
#define CSI_SERIAL_LOG_MIN_INTERVAL_US 50000U
#define CSI_BROWSER_CAPTURE_MIN_INTERVAL_US 50000U
#define CSI_AP_DEFAULT_CHANNEL            6U
#define CSI_AP_DEFAULT_BROADCAST_MS      20U
#define CSI_AP_MAX_CLIENTS                4U
#define CSI_AP_TRAFFIC_PORT            3333
#define CSI_SELECTED_SUBCARRIER_COUNT    10U
#define CSI_SELECTED_MACRO_ENTER       2.20f
#define CSI_SELECTED_MACRO_CLEAR       1.35f
#define CSI_SELECTED_SHIFT_ENTER       1.45f
#define CSI_SELECTED_SHIFT_CLEAR       1.05f
#define CSI_SELECTED_MICRO_ENTER       1.35f
#define CSI_SELECTED_MICRO_CLEAR       1.05f
#define CSI_SELECTED_MIN_SHIFT_REF     0.055f
#define CSI_SELECTED_MIN_DELTA_REF     0.030f
#define CSI_HYBRID_SHIFT_ENTER        0.62f
#define CSI_HYBRID_SHIFT_CLEAR        0.60f
#define CSI_HYBRID_MOTION_ENTER       0.90f
#define CSI_HYBRID_MOTION_CLEAR       0.85f
#define CSI_HYBRID_OCCUPANCY_CLEAR    0.55f
#define CSI_HYBRID_MOTION_CONFIRM      5U
#define CSI_HYBRID_STATIC_CONFIRM      8U
#define CSI_HYBRID_CLEAR_CONFIRM      28U
#define CSI_HYBRID_MOTION_HOLD        20U
#define CSI_HYBRID_STATIC_HOLD        45U
#define CSI_HYBRID_MICRO_ENTER        1.20f
#define CSI_HYBRID_MICRO_CLEAR        0.90f
#define CSI_VISUAL_RHYTHM_ENTER       1.45f
#define CSI_VISUAL_RHYTHM_CLEAR       1.05f
#define CSI_MICRO_ACTIVITY_CONFIRM    16U

static const char *TAG = "csi_engine";

static const uint8_t s_selected_subcarriers[CSI_SELECTED_SUBCARRIER_COUNT] = {
    66, 79, 80, 81, 82, 83, 84, 85, 126, 127
};

typedef struct {
    uint64_t timestamp_us;
    uint16_t len;
    int8_t rssi;
    bool first_word_invalid;
    uint8_t src_mac[6];
    int8_t raw[CSI_MAX_SUBCARRIERS * 2];
} csi_frame_t;

typedef struct {
    uint16_t samples;
    float occupancy_sum;
    float motion_sum;
    float micromotion_sum;
    float stability_sum;
    float occupancy_peak;
    float motion_peak;
} csi_setup_metrics_t;

/* ───────── processing state ───────── */
static float s_motion_threshold = 1.8f;
static csi_detection_profile_t s_detection_profile;
static bool s_detection_profile_custom = false;
static uint8_t s_confirm_motion = 3;
static uint8_t s_confirm_clear = 8;
static bool s_raw_logging_enabled = true;
static bool s_detection_enabled = false;
static bool s_force_calibration_enabled = false;
static csi_wifi_role_t s_wifi_role = CSI_WIFI_ROLE_SENSOR_STA;
static uint16_t s_ap_broadcast_interval_ms = CSI_AP_DEFAULT_BROADCAST_MS;
static uint8_t s_ap_channel = CSI_AP_DEFAULT_CHANNEL;

static csi_presence_state_t s_presence_state = CSI_PRESENCE_NO_PRESENCE;
static csi_presence_state_t s_selected_presence_state = CSI_PRESENCE_NO_PRESENCE;
static csi_presence_state_t s_hybrid_presence_state = CSI_PRESENCE_NO_PRESENCE;
static uint8_t s_macro_frames = 0;
static uint8_t s_micro_frames = 0;
static uint8_t s_static_frames = 0;
static uint8_t s_clear_frames = 0;
static uint8_t s_presence_hold_frames = 0;
static uint8_t s_disturbance_cooldown = 0;
static uint32_t s_state_event_count = 0;
static uint32_t s_disturbance_event_count = 0;
static uint32_t s_processed_frames = 0;
static uint8_t s_selected_macro_frames = 0;
static uint8_t s_selected_micro_frames = 0;
static uint8_t s_selected_static_frames = 0;
static uint8_t s_selected_clear_frames = 0;
static uint8_t s_hybrid_motion_frames = 0;
static uint8_t s_hybrid_micro_frames = 0;
static uint8_t s_hybrid_static_frames = 0;
static uint8_t s_hybrid_clear_frames = 0;
static uint8_t s_hybrid_motion_hold_frames = 0;
static uint8_t s_hybrid_static_hold_frames = 0;
static uint8_t s_micro_activity_frames = 0;

static float s_occupancy_score = 0.0f;
static float s_micromotion_score = 0.0f;
static float s_motion_score = 0.0f;
static float s_stability_score = 0.0f;
static float s_packet_rate_hz = 0.0f;
static float s_packet_rate_jitter_hz = 0.0f;
static float s_rssi_avg = -127.0f;
static float s_last_occupancy_energy = 0.0f;
static float s_last_motion_avg_energy = 0.0f;
static float s_last_motion_energy = 0.0f;
static float s_last_occupancy_ref = 0.0f;
static float s_last_motion_ref = 0.0f;
static float s_last_motion_enter = 0.0f;
static float s_last_motion_clear = 0.0f;
static float s_last_micro_enter = 0.0f;
static float s_last_micro_clear = 0.0f;
static float s_last_presence_enter = 0.0f;
static float s_last_presence_clear = 0.0f;
static float s_selected_baseline[CSI_SELECTED_SUBCARRIER_COUNT];
static float s_selected_prev_amp[CSI_SELECTED_SUBCARRIER_COUNT];
static float s_selected_micro_ema[CSI_SELECTED_SUBCARRIER_COUNT];
static bool s_selected_prev_ready[CSI_SELECTED_SUBCARRIER_COUNT];
static float s_selected_shift_floor = 0.0f;
static float s_selected_shift_jitter = 0.10f;
static float s_selected_macro_floor = 0.0f;
static float s_selected_macro_jitter = 0.10f;
static float s_selected_micro_floor = 0.0f;
static float s_selected_micro_jitter = 0.10f;
static float s_selected_shift_score = 0.0f;
static float s_selected_macro_score = 0.0f;
static float s_selected_micro_score = 0.0f;
static float s_selected_shift_raw = 0.0f;
static float s_selected_macro_raw = 0.0f;
static float s_selected_micro_raw = 0.0f;
static float s_selected_shift_ref = 0.0f;
static float s_selected_macro_ref = 0.0f;
static float s_selected_micro_ref = 0.0f;
static uint8_t s_selected_present_count = 0;
static float s_shape_shift_score = 0.0f;
static float s_rhythm_motion_score = 0.0f;
static float s_patch_activity_score = 0.0f;
static float s_shape_shift_raw = 0.0f;
static float s_rhythm_motion_raw = 0.0f;
static float s_patch_activity_raw = 0.0f;
static float s_shape_shift_ref = 0.0f;
static float s_rhythm_motion_ref = 0.0f;
static float s_patch_activity_ref = 0.0f;
static bool s_micro_activity = false;

static uint8_t s_num_sc = 0;
static uint8_t s_valid_sc = 0;
static bool s_raw_log_header_emitted = false;
static uint64_t s_last_serial_log_us = 0;
static uint64_t s_last_capture_enqueue_us = 0;
static char s_csv_row_buf[CSI_CSV_ROW_MAX];
static char s_manual_annotation[CSI_MANUAL_ANNOTATION_MAX] = "none";
static float s_latest_amps[CSI_MAX_SUBCARRIERS];
static float s_fast_features[CSI_MAX_SUBCARRIERS];
static float s_slow_features[CSI_MAX_SUBCARRIERS];
static float s_display_features[CSI_MAX_SUBCARRIERS];
static float s_empty_baseline[CSI_MAX_SUBCARRIERS];
static float s_visual_baseline[CSI_MAX_SUBCARRIERS];
static float s_visual_prev_norm[CSI_MAX_SUBCARRIERS];
static float s_visual_short_ema[CSI_MAX_SUBCARRIERS];
static float s_visual_patch_jitter[CSI_MAX_SUBCARRIERS];
static bool s_visual_prev_ready[CSI_MAX_SUBCARRIERS];
static bool s_feature_filters_ready = false;
static bool s_baseline_ready = false;
static bool s_calibration_guard_blocked = false;
static uint16_t s_calibration_frames_remaining = CSI_BASELINE_CALIBRATION_FRAMES;
static uint8_t s_calibration_guard_cooldown = 0;
static uint32_t s_calibration_guard_hits = 0;
static float s_empty_floor = 0.0f;
static float s_empty_jitter = 0.10f;
static float s_motion_floor = 0.0f;
static float s_motion_jitter = 0.10f;
static float s_shape_shift_floor = 0.0f;
static float s_shape_shift_jitter = 0.01f;
static float s_rhythm_motion_floor = 0.0f;
static float s_rhythm_motion_jitter = 0.01f;
static float s_patch_activity_floor = 0.0f;
static float s_patch_activity_jitter = 0.10f;
static int8_t s_last_rssi = -127;
static int8_t s_min_rssi = 127;
static int8_t s_max_rssi = -127;
static uint64_t s_last_timestamp_us = 0;
static volatile uint32_t s_csi_queue_drops = 0;
static csi_setup_stage_t s_guided_stage = CSI_SETUP_IDLE;
static bool s_guided_active = false;
static bool s_guided_manual_sampling = false;
static bool s_guided_sampling = false;
static uint16_t s_guided_settle_frames_remaining = 0;
static csi_setup_metrics_t s_guided_metrics[CSI_SETUP_COMPLETE + 1];
static float s_guided_suggested_motion_threshold = 0.0f;
static float s_guided_suggested_presence_enter = 0.0f;
static float s_guided_motion_contrast = 0.0f;
static float s_guided_static_contrast = 0.0f;
static bool s_guided_profile_applied = false;
static char s_guided_profile_mode[16] = "incomplete";
static char s_guided_result[16] = "incomplete";

/* ───────── thread safety / tasks ───────── */
static SemaphoreHandle_t s_mutex = NULL;
static QueueHandle_t s_csi_queue = NULL;
static TaskHandle_t s_csi_worker_task = NULL;
static TaskHandle_t s_ap_traffic_task = NULL;

/* ───────── notch filter ───────── */
static bool s_notch_enabled = false;
static float s_notch_freq_hz = 0.0f;
static float s_notch_a1 = 0.0f;
static float s_notch_a2 = 0.0f;
static float s_notch_b0 = 0.0f;
static float s_notch_b1 = 0.0f;
static float s_notch_b2 = 0.0f;
static float s_notch_x1[CSI_MAX_SUBCARRIERS];
static float s_notch_x2[CSI_MAX_SUBCARRIERS];
static float s_notch_y1[CSI_MAX_SUBCARRIERS];
static float s_notch_y2[CSI_MAX_SUBCARRIERS];

/* ───────── Wi-Fi events ───────── */
static EventGroupHandle_t s_wifi_event_group = NULL;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_cnt = 0;
static TimerHandle_t s_reconnect_timer = NULL;
static TimerHandle_t s_inject_timer = NULL;
static esp_netif_t *s_wifi_netif = NULL;
static char s_remote_addr[20] = "0.0.0.0";
static volatile bool s_wifi_is_error = false;
static volatile bool s_wifi_is_connected = false;
static volatile bool s_null_frame_ready = false;
static uint8_t s_ap_bssid[6];
static bool s_ap_bssid_valid = false;
static volatile uint8_t s_ap_client_count = 0;

static uint8_t s_null_frame[24] = {
    0x48, 0x01, 0x00, 0x00,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x00
};

static const char s_ap_broadcast_payload[] = "CSI_KEEPALIVE";

static bool is_stable_subcarrier(uint8_t sc)
{
    return sc >= 6 && sc <= 58 && !(sc > 26 && sc < 38);
}

static float ema_blend(float current, float sample, float alpha)
{
    return current + ((sample - current) * alpha);
}

static float clampf_local(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static uint8_t clamp_confirm_frames(uint8_t frames, uint8_t fallback)
{
    if (frames == 0) {
        return fallback;
    }
    if (frames > CSI_MAX_CONFIRM_FRAMES) {
        return CSI_MAX_CONFIRM_FRAMES;
    }
    return frames;
}

static float clamp_motion_threshold(float threshold)
{
    if (!isfinite(threshold)) {
        return s_motion_threshold;
    }
    return clampf_local(threshold, CSI_MIN_MOTION_THRESHOLD,
                        CSI_MAX_MOTION_THRESHOLD);
}

static csi_detection_profile_t profile_from_motion_threshold(float threshold)
{
    csi_detection_profile_t profile;
    float motion_enter = clamp_motion_threshold(threshold);
    float micro_enter;
    float presence_enter;

    profile.motion_enter = motion_enter;
    profile.motion_clear = fmaxf(0.80f, motion_enter * 0.78f);
    micro_enter = fmaxf(0.60f, motion_enter * 0.57f);
    profile.micro_enter = micro_enter;
    profile.micro_clear = fmaxf(0.46f, micro_enter * 0.80f);
    presence_enter = fmaxf(0.98f, 0.98f + ((motion_enter - 1.0f) * 0.16f));
    profile.presence_enter = presence_enter;
    profile.presence_clear = fmaxf(0.82f, presence_enter * 0.86f);

    return profile;
}

static csi_detection_profile_t sanitize_detection_profile(
    csi_detection_profile_t profile)
{
    profile.motion_enter = clamp_motion_threshold(profile.motion_enter);
    profile.motion_clear = clampf_local(profile.motion_clear, 0.40f,
                                        profile.motion_enter);
    profile.micro_enter = clampf_local(profile.micro_enter, 0.30f,
                                       profile.motion_enter);
    profile.micro_clear = clampf_local(profile.micro_clear, 0.20f,
                                       profile.micro_enter);
    profile.presence_enter = clampf_local(profile.presence_enter, 0.60f, 6.00f);
    profile.presence_clear = clampf_local(profile.presence_clear, 0.40f,
                                         profile.presence_enter);
    return profile;
}

static void set_profile_from_motion_threshold_locked(float threshold)
{
    s_motion_threshold = clamp_motion_threshold(threshold);
    s_detection_profile =
        sanitize_detection_profile(profile_from_motion_threshold(s_motion_threshold));
    s_detection_profile_custom = false;
}

static void set_custom_detection_profile_locked(csi_detection_profile_t profile)
{
    s_detection_profile = sanitize_detection_profile(profile);
    s_motion_threshold = s_detection_profile.motion_enter;
    s_detection_profile_custom = true;
}

static float clamp_notch_frequency(float freq_hz)
{
    if (!isfinite(freq_hz) || freq_hz <= 0.0f) {
        return 0.0f;
    }
    return clampf_local(freq_hz, 0.0f, CSI_MAX_NOTCH_FREQ_HZ);
}

static void clear_notch_history_locked(void)
{
    memset(s_notch_x1, 0, sizeof(s_notch_x1));
    memset(s_notch_x2, 0, sizeof(s_notch_x2));
    memset(s_notch_y1, 0, sizeof(s_notch_y1));
    memset(s_notch_y2, 0, sizeof(s_notch_y2));
}

static void configure_notch_locked(float freq_hz)
{
    freq_hz = clamp_notch_frequency(freq_hz);
    if (freq_hz == 0.0f) {
        s_notch_enabled = false;
        s_notch_freq_hz = 0.0f;
        clear_notch_history_locked();
        return;
    }

    {
        float w0 = 2.0f * (float)M_PI * (freq_hz / CSI_SAMPLE_RATE_HZ);
        float alpha = sinf(w0) / 4.0f;
        float a0 = 1.0f + alpha;

        s_notch_b0 = 1.0f / a0;
        s_notch_b1 = (-2.0f * cosf(w0)) / a0;
        s_notch_b2 = 1.0f / a0;
        s_notch_a1 = (-2.0f * cosf(w0)) / a0;
        s_notch_a2 = (1.0f - alpha) / a0;
    }

    s_notch_enabled = true;
    s_notch_freq_hz = freq_hz;
    clear_notch_history_locked();
}

static float apply_notch_locked(uint8_t sc, float value)
{
    float y0;

    if (!s_notch_enabled || sc >= CSI_MAX_SUBCARRIERS) {
        return value;
    }

    y0 = s_notch_b0 * value +
         s_notch_b1 * s_notch_x1[sc] +
         s_notch_b2 * s_notch_x2[sc] -
         s_notch_a1 * s_notch_y1[sc] -
         s_notch_a2 * s_notch_y2[sc];

    s_notch_x2[sc] = s_notch_x1[sc];
    s_notch_x1[sc] = value;
    s_notch_y2[sc] = s_notch_y1[sc];
    s_notch_y1[sc] = y0;

    return y0;
}

const char *csi_presence_state_to_string(csi_presence_state_t state)
{
    switch (state) {
    case CSI_PRESENCE_STATIC:
        return "presence_static";
    case CSI_PRESENCE_MICROMOTION:
        return "presence_micromotion";
    case CSI_PRESENCE_MOTION:
        return "presence_motion";
    case CSI_PRESENCE_NO_PRESENCE:
    default:
        return "no_presence";
    }
}

const char *csi_wifi_role_to_string(csi_wifi_role_t role)
{
    switch (role) {
    case CSI_WIFI_ROLE_DEDICATED_AP:
        return "dedicated_ap";
    case CSI_WIFI_ROLE_SENSOR_STA:
    default:
        return "sensor_sta";
    }
}

static uint16_t clamp_ap_broadcast_interval(uint16_t interval_ms)
{
    if (interval_ms == 0U) {
        return CSI_AP_DEFAULT_BROADCAST_MS;
    }
    return (uint16_t)clampf_local((float)interval_ms, 10.0f, 1000.0f);
}

static uint8_t clamp_ap_channel(uint8_t channel)
{
    if (channel == 0U) {
        return CSI_AP_DEFAULT_CHANNEL;
    }
    return (uint8_t)clampf_local((float)channel, 1.0f, 13.0f);
}

static bool detection_supported(void)
{
    return s_wifi_role == CSI_WIFI_ROLE_SENSOR_STA;
}

static void sanitize_annotation(const char *src, char *dst, size_t dst_size)
{
    size_t out = 0;

    if (!dst || dst_size == 0U) {
        return;
    }

    if (!src || src[0] == '\0') {
        strlcpy(dst, "none", dst_size);
        return;
    }

    for (size_t i = 0; src[i] != '\0' && out + 1U < dst_size; i++) {
        char ch = src[i];

        if (((ch >= 'a') && (ch <= 'z')) ||
            ((ch >= 'A') && (ch <= 'Z')) ||
            ((ch >= '0') && (ch <= '9'))) {
            dst[out++] = (char)((ch >= 'A' && ch <= 'Z') ? (ch + 32) : ch);
        } else if ((ch == ' ') || (ch == '-') || (ch == '_')) {
            dst[out++] = '_';
        }
    }

    dst[out] = '\0';
    if (out == 0U) {
        strlcpy(dst, "none", dst_size);
    }
}

static void refresh_netif_ip_locked(void)
{
    esp_netif_ip_info_t ip_info;

    if (!s_wifi_netif || esp_netif_get_ip_info(s_wifi_netif, &ip_info) != ESP_OK) {
        strlcpy(s_remote_addr, "0.0.0.0", sizeof(s_remote_addr));
        return;
    }

    snprintf(s_remote_addr, sizeof(s_remote_addr), IPSTR, IP2STR(&ip_info.ip));
}

static void reset_processing_state_locked(void)
{
    bool force_calibration_enabled = s_force_calibration_enabled;

    memset(s_latest_amps, 0, sizeof(s_latest_amps));
    memset(s_fast_features, 0, sizeof(s_fast_features));
    memset(s_slow_features, 0, sizeof(s_slow_features));
    memset(s_display_features, 0, sizeof(s_display_features));
    memset(s_empty_baseline, 0, sizeof(s_empty_baseline));
    memset(s_visual_baseline, 0, sizeof(s_visual_baseline));
    memset(s_visual_prev_norm, 0, sizeof(s_visual_prev_norm));
    memset(s_visual_short_ema, 0, sizeof(s_visual_short_ema));
    memset(s_visual_patch_jitter, 0, sizeof(s_visual_patch_jitter));
    memset(s_visual_prev_ready, 0, sizeof(s_visual_prev_ready));
    memset(s_selected_baseline, 0, sizeof(s_selected_baseline));
    memset(s_selected_prev_amp, 0, sizeof(s_selected_prev_amp));
    memset(s_selected_micro_ema, 0, sizeof(s_selected_micro_ema));
    memset(s_selected_prev_ready, 0, sizeof(s_selected_prev_ready));

    s_presence_state = CSI_PRESENCE_NO_PRESENCE;
    s_selected_presence_state = CSI_PRESENCE_NO_PRESENCE;
    s_hybrid_presence_state = CSI_PRESENCE_NO_PRESENCE;
    s_macro_frames = 0;
    s_micro_frames = 0;
    s_static_frames = 0;
    s_clear_frames = 0;
    s_presence_hold_frames = 0;
    s_disturbance_cooldown = 0;
    s_state_event_count = 0;
    s_disturbance_event_count = 0;
    s_processed_frames = 0;
    s_selected_macro_frames = 0;
    s_selected_micro_frames = 0;
    s_selected_static_frames = 0;
    s_selected_clear_frames = 0;
    s_hybrid_motion_frames = 0;
    s_hybrid_micro_frames = 0;
    s_hybrid_static_frames = 0;
    s_hybrid_clear_frames = 0;
    s_hybrid_motion_hold_frames = 0;
    s_hybrid_static_hold_frames = 0;
    s_micro_activity_frames = 0;

    s_occupancy_score = 0.0f;
    s_micromotion_score = 0.0f;
    s_motion_score = 0.0f;
    s_stability_score = 0.0f;
    s_packet_rate_hz = 0.0f;
    s_packet_rate_jitter_hz = 0.0f;
    s_rssi_avg = -127.0f;
    s_last_occupancy_energy = 0.0f;
    s_last_motion_avg_energy = 0.0f;
    s_last_motion_energy = 0.0f;
    s_last_occupancy_ref = 0.0f;
    s_last_motion_ref = 0.0f;
    s_last_motion_enter = 0.0f;
    s_last_motion_clear = 0.0f;
    s_last_micro_enter = 0.0f;
    s_last_micro_clear = 0.0f;
    s_last_presence_enter = 0.0f;
    s_last_presence_clear = 0.0f;
    s_selected_shift_floor = 0.0f;
    s_selected_shift_jitter = 0.10f;
    s_selected_macro_floor = 0.0f;
    s_selected_macro_jitter = 0.10f;
    s_selected_micro_floor = 0.0f;
    s_selected_micro_jitter = 0.10f;
    s_selected_shift_score = 0.0f;
    s_selected_macro_score = 0.0f;
    s_selected_micro_score = 0.0f;
    s_selected_shift_raw = 0.0f;
    s_selected_macro_raw = 0.0f;
    s_selected_micro_raw = 0.0f;
    s_selected_shift_ref = 0.0f;
    s_selected_macro_ref = 0.0f;
    s_selected_micro_ref = 0.0f;
    s_selected_present_count = 0;
    s_shape_shift_score = 0.0f;
    s_rhythm_motion_score = 0.0f;
    s_patch_activity_score = 0.0f;
    s_shape_shift_raw = 0.0f;
    s_rhythm_motion_raw = 0.0f;
    s_patch_activity_raw = 0.0f;
    s_shape_shift_ref = 0.0f;
    s_rhythm_motion_ref = 0.0f;
    s_patch_activity_ref = 0.0f;
    s_micro_activity = false;

    s_num_sc = 0;
    s_valid_sc = 0;
    s_raw_log_header_emitted = false;
    s_last_serial_log_us = 0;
    s_last_capture_enqueue_us = 0;
    s_feature_filters_ready = false;
    s_baseline_ready = false;
    s_calibration_guard_blocked = false;
    s_calibration_frames_remaining = CSI_BASELINE_CALIBRATION_FRAMES;
    s_calibration_guard_cooldown = 0;
    s_calibration_guard_hits = 0;
    s_empty_floor = 0.0f;
    s_empty_jitter = 0.10f;
    s_motion_floor = 0.0f;
    s_motion_jitter = 0.10f;
    s_shape_shift_floor = 0.0f;
    s_shape_shift_jitter = 0.01f;
    s_rhythm_motion_floor = 0.0f;
    s_rhythm_motion_jitter = 0.01f;
    s_patch_activity_floor = 0.0f;
    s_patch_activity_jitter = 0.10f;
    s_last_rssi = -127;
    s_min_rssi = 127;
    s_max_rssi = -127;
    s_last_timestamp_us = 0;
    s_csi_queue_drops = 0;

    clear_notch_history_locked();
    s_force_calibration_enabled = force_calibration_enabled;
    if (!s_detection_profile_custom) {
        s_detection_profile =
            sanitize_detection_profile(profile_from_motion_threshold(s_motion_threshold));
    }
}

static void restart_calibration_locked(bool blocked)
{
    bool raw_log_header_emitted = s_raw_log_header_emitted;
    uint64_t last_capture_enqueue_us = s_last_capture_enqueue_us;
    float packet_rate_hz = s_packet_rate_hz;
    int8_t last_rssi = s_last_rssi;
    uint64_t last_timestamp_us = s_last_timestamp_us;
    float packet_rate_jitter_hz = s_packet_rate_jitter_hz;
    float rssi_avg = s_rssi_avg;
    int8_t min_rssi = s_min_rssi;
    int8_t max_rssi = s_max_rssi;
    uint32_t guard_hits = s_calibration_guard_hits;
    bool force_calibration_enabled = s_force_calibration_enabled;
    uint32_t queue_drops = s_csi_queue_drops;

    reset_processing_state_locked();
    s_raw_log_header_emitted = raw_log_header_emitted;
    s_last_capture_enqueue_us = last_capture_enqueue_us;
    s_packet_rate_hz = packet_rate_hz;
    s_packet_rate_jitter_hz = packet_rate_jitter_hz;
    s_rssi_avg = rssi_avg;
    s_last_rssi = last_rssi;
    s_min_rssi = min_rssi;
    s_max_rssi = max_rssi;
    s_last_timestamp_us = last_timestamp_us;
    s_csi_queue_drops = queue_drops;
    s_force_calibration_enabled = force_calibration_enabled;
    s_calibration_guard_blocked = blocked;
    s_calibration_guard_hits = blocked ? (guard_hits + 1U) : guard_hits;
    s_calibration_guard_cooldown = blocked ?
        CSI_CALIBRATION_GUARD_COOLDOWN_FRAMES : 0U;
}

static void push_display_amplitudes_locked(const float *amps, uint8_t num_sc)
{
    memset(s_latest_amps, 0, sizeof(s_latest_amps));
    memcpy(s_latest_amps, amps, num_sc * sizeof(float));
    s_num_sc = num_sc;
}

static void update_live_packet_stats_locked(uint64_t timestamp_us, int8_t rssi,
                                            uint8_t valid_sc)
{
    uint64_t prev_timestamp_us = s_last_timestamp_us;
    float prev_rate_hz = s_packet_rate_hz;

    s_last_rssi = rssi;
    s_min_rssi = (s_min_rssi == 127 || rssi < s_min_rssi) ? rssi : s_min_rssi;
    s_max_rssi = (s_max_rssi == -127 || rssi > s_max_rssi) ? rssi : s_max_rssi;
    s_rssi_avg = (s_rssi_avg <= -126.0f) ? (float)rssi :
        ema_blend(s_rssi_avg, (float)rssi, 0.08f);
    s_last_timestamp_us = timestamp_us;
    s_valid_sc = valid_sc;
    if (prev_timestamp_us > 0 && timestamp_us > prev_timestamp_us) {
        float inst_rate_hz = 1000000.0f / (float)(timestamp_us - prev_timestamp_us);
        s_packet_rate_hz = (s_packet_rate_hz <= 0.0f) ? inst_rate_hz :
            ema_blend(s_packet_rate_hz, inst_rate_hz, 0.18f);
        if (prev_rate_hz > 0.0f) {
            s_packet_rate_jitter_hz = ema_blend(
                s_packet_rate_jitter_hz,
                fabsf(inst_rate_hz - prev_rate_hz),
                0.16f);
        }
    }
}

static void adapt_empty_baseline_locked(const float *slow_snapshot, uint8_t num_sc,
                                        float alpha)
{
    uint8_t sc;

    for (sc = 0; sc < num_sc; sc++) {
        if (!is_stable_subcarrier(sc)) {
            continue;
        }
        s_empty_baseline[sc] = ema_blend(s_empty_baseline[sc], slow_snapshot[sc], alpha);
    }
}

static void smooth_scores_locked(float raw_occupancy_score,
                                 float raw_micromotion_score,
                                 float raw_motion_score,
                                 float raw_stability_score)
{
    if (s_processed_frames <= CSI_WARMUP_FRAMES) {
        s_occupancy_score = raw_occupancy_score;
        s_micromotion_score = raw_micromotion_score;
        s_motion_score = raw_motion_score;
        s_stability_score = raw_stability_score;
        return;
    }

    s_occupancy_score = ema_blend(s_occupancy_score, raw_occupancy_score,
                                  CSI_SCORE_ALPHA);
    s_micromotion_score = ema_blend(s_micromotion_score, raw_micromotion_score,
                                    CSI_SCORE_ALPHA);
    s_motion_score = ema_blend(s_motion_score, raw_motion_score,
                               CSI_SCORE_ALPHA);
    s_stability_score = ema_blend(s_stability_score, raw_stability_score,
                                  CSI_STABILITY_ALPHA);
}

static void update_selected_detector_locked(const float *raw_amps, uint8_t num_sc,
                                            float frame_mean,
                                            bool calibration_phase)
{
    float shift_sum = 0.0f;
    float macro_sum = 0.0f;
    float micro_sum = 0.0f;
    uint8_t present = 0;
    uint8_t macro_count = 0;
    bool macro_candidate;
    bool micro_candidate;
    bool static_candidate;
    bool clear_candidate;
    csi_presence_state_t new_state;

    if (!raw_amps || !isfinite(frame_mean) || frame_mean <= 0.0f) {
        return;
    }

    for (uint8_t i = 0; i < CSI_SELECTED_SUBCARRIER_COUNT; i++) {
        uint8_t sc = s_selected_subcarriers[i];
        float amp;
        float delta = 0.0f;

        if (sc >= num_sc) {
            continue;
        }

        amp = raw_amps[sc];
        if (!isfinite(amp) || amp <= 0.0f) {
            continue;
        }
        amp /= fmaxf(frame_mean, 1.0f);

        if (calibration_phase || s_selected_baseline[i] <= 0.0f) {
            s_selected_baseline[i] = (s_selected_baseline[i] <= 0.0f) ? amp :
                ema_blend(s_selected_baseline[i], amp, 0.08f);
        }

        if (s_selected_prev_ready[i]) {
            delta = fabsf(amp - s_selected_prev_amp[i]);
            macro_sum += delta;
            macro_count++;
            s_selected_micro_ema[i] = ema_blend(s_selected_micro_ema[i],
                                                delta, 0.18f);
            micro_sum += s_selected_micro_ema[i];
        }

        shift_sum += fabsf(amp - s_selected_baseline[i]);
        s_selected_prev_amp[i] = amp;
        s_selected_prev_ready[i] = true;
        present++;
    }

    s_selected_present_count = present;
    if (present == 0U) {
        return;
    }

    s_selected_shift_raw = shift_sum / (float)present;
    s_selected_macro_raw = (macro_count == 0U) ? 0.0f :
        (macro_sum / (float)macro_count);
    s_selected_micro_raw = (macro_count == 0U) ? 0.0f :
        (micro_sum / (float)macro_count);

    if (calibration_phase) {
        s_selected_shift_floor = (s_processed_frames <= 1U) ?
            s_selected_shift_raw :
            ema_blend(s_selected_shift_floor, s_selected_shift_raw, 0.08f);
        s_selected_shift_jitter = ema_blend(s_selected_shift_jitter,
            fabsf(s_selected_shift_raw - s_selected_shift_floor), 0.10f);
        s_selected_macro_floor = (s_processed_frames <= 1U) ?
            s_selected_macro_raw :
            ema_blend(s_selected_macro_floor, s_selected_macro_raw, 0.08f);
        s_selected_macro_jitter = ema_blend(s_selected_macro_jitter,
            fabsf(s_selected_macro_raw - s_selected_macro_floor), 0.10f);
        s_selected_micro_floor = (s_processed_frames <= 1U) ?
            s_selected_micro_raw :
            ema_blend(s_selected_micro_floor, s_selected_micro_raw, 0.08f);
        s_selected_micro_jitter = ema_blend(s_selected_micro_jitter,
            fabsf(s_selected_micro_raw - s_selected_micro_floor), 0.10f);
        s_selected_shift_score = 0.0f;
        s_selected_macro_score = 0.0f;
        s_selected_micro_score = 0.0f;
        s_selected_presence_state = CSI_PRESENCE_NO_PRESENCE;
        s_hybrid_presence_state = CSI_PRESENCE_NO_PRESENCE;
        return;
    }

    s_selected_shift_ref = fmaxf(CSI_SELECTED_MIN_SHIFT_REF,
        s_selected_shift_floor + (s_selected_shift_jitter * 2.4f));
    s_selected_macro_ref = fmaxf(CSI_SELECTED_MIN_DELTA_REF,
        s_selected_macro_floor + (s_selected_macro_jitter * 2.2f));
    s_selected_micro_ref = fmaxf(CSI_SELECTED_MIN_DELTA_REF,
        s_selected_micro_floor + (s_selected_micro_jitter * 2.0f));
    s_selected_shift_score = ema_blend(s_selected_shift_score,
        s_selected_shift_raw / s_selected_shift_ref, 0.22f);
    s_selected_macro_score = ema_blend(s_selected_macro_score,
        s_selected_macro_raw / s_selected_macro_ref, 0.30f);
    s_selected_micro_score = ema_blend(s_selected_micro_score,
        s_selected_micro_raw / s_selected_micro_ref, 0.24f);

    macro_candidate = s_selected_macro_score >= CSI_SELECTED_MACRO_ENTER;
    micro_candidate = (s_selected_shift_score >= CSI_SELECTED_SHIFT_CLEAR) &&
        (s_selected_macro_score < CSI_SELECTED_MACRO_ENTER) &&
        (s_selected_micro_score >= CSI_SELECTED_MICRO_ENTER);
    static_candidate = (s_selected_shift_score >= CSI_SELECTED_SHIFT_ENTER) &&
        (s_selected_macro_score <= CSI_SELECTED_MACRO_CLEAR) &&
        (s_selected_micro_score <= CSI_SELECTED_MICRO_ENTER);
    clear_candidate = (s_selected_shift_score <= CSI_SELECTED_SHIFT_CLEAR) &&
        (s_selected_macro_score <= CSI_SELECTED_MACRO_CLEAR) &&
        (s_selected_micro_score <= CSI_SELECTED_MICRO_CLEAR);

    s_selected_macro_frames = macro_candidate ?
        (uint8_t)fminf((float)UINT8_MAX, (float)(s_selected_macro_frames + 1)) : 0;
    s_selected_micro_frames = micro_candidate ?
        (uint8_t)fminf((float)UINT8_MAX, (float)(s_selected_micro_frames + 1)) : 0;
    s_selected_static_frames = static_candidate ?
        (uint8_t)fminf((float)UINT8_MAX, (float)(s_selected_static_frames + 1)) : 0;
    s_selected_clear_frames = clear_candidate ?
        (uint8_t)fminf((float)UINT8_MAX, (float)(s_selected_clear_frames + 1)) : 0;

    new_state = s_selected_presence_state;
    if (s_selected_macro_frames >= 4U) {
        new_state = CSI_PRESENCE_MOTION;
    } else if (s_selected_micro_frames >= 16U) {
        new_state = CSI_PRESENCE_MICROMOTION;
    } else if (s_selected_static_frames >= 12U) {
        new_state = CSI_PRESENCE_STATIC;
    } else if (s_selected_clear_frames >= 40U) {
        new_state = CSI_PRESENCE_NO_PRESENCE;
    }
    s_selected_presence_state = new_state;
}

static void update_visual_features_locked(const float *raw_amps, uint8_t num_sc,
                                          float frame_mean,
                                          bool calibration_phase)
{
    float shape_sum = 0.0f;
    float rhythm_sum = 0.0f;
    float patch_sum = 0.0f;
    uint8_t shape_count = 0;
    uint8_t rhythm_count = 0;
    uint8_t patch_count = 0;
    bool micro_candidate;

    if (!raw_amps || !isfinite(frame_mean) || frame_mean <= 0.0f) {
        return;
    }

    for (uint8_t sc = 0; sc < num_sc; sc++) {
        float norm;
        float delta = 0.0f;
        float patch_delta;
        float patch_threshold;

        if (!is_stable_subcarrier(sc) || raw_amps[sc] <= 0.0f ||
            !isfinite(raw_amps[sc])) {
            continue;
        }

        norm = raw_amps[sc] / fmaxf(frame_mean, 1.0f);
        if (calibration_phase || s_visual_baseline[sc] <= 0.0f) {
            s_visual_baseline[sc] = (s_visual_baseline[sc] <= 0.0f) ? norm :
                ema_blend(s_visual_baseline[sc], norm, 0.06f);
        }
        if (s_visual_short_ema[sc] <= 0.0f) {
            s_visual_short_ema[sc] = norm;
        }

        shape_sum += fabsf(norm - s_visual_baseline[sc]);
        shape_count++;

        if (s_visual_prev_ready[sc]) {
            delta = fabsf(norm - s_visual_prev_norm[sc]);
            rhythm_sum += delta;
            rhythm_count++;
        }

        patch_delta = fabsf(norm - s_visual_short_ema[sc]);
        patch_threshold = fmaxf(0.025f, s_visual_patch_jitter[sc] * 3.0f);
        if (patch_delta > patch_threshold) {
            patch_sum += 1.0f;
        }
        patch_count++;
        s_visual_patch_jitter[sc] = ema_blend(s_visual_patch_jitter[sc],
                                              patch_delta, 0.08f);
        s_visual_short_ema[sc] = ema_blend(s_visual_short_ema[sc], norm, 0.16f);
        s_visual_prev_norm[sc] = norm;
        s_visual_prev_ready[sc] = true;
    }

    if (shape_count == 0U) {
        return;
    }

    s_shape_shift_raw = shape_sum / (float)shape_count;
    s_rhythm_motion_raw = (rhythm_count == 0U) ? 0.0f :
        (rhythm_sum / (float)rhythm_count);
    s_patch_activity_raw = (patch_count == 0U) ? 0.0f :
        ((patch_sum / (float)patch_count) * 10.0f);

    if (calibration_phase) {
        s_shape_shift_floor = (s_processed_frames <= 1U) ? s_shape_shift_raw :
            ema_blend(s_shape_shift_floor, s_shape_shift_raw, 0.08f);
        s_shape_shift_jitter = ema_blend(s_shape_shift_jitter,
            fabsf(s_shape_shift_raw - s_shape_shift_floor), 0.10f);
        s_rhythm_motion_floor = (s_processed_frames <= 1U) ? s_rhythm_motion_raw :
            ema_blend(s_rhythm_motion_floor, s_rhythm_motion_raw, 0.08f);
        s_rhythm_motion_jitter = ema_blend(s_rhythm_motion_jitter,
            fabsf(s_rhythm_motion_raw - s_rhythm_motion_floor), 0.10f);
        s_patch_activity_floor = (s_processed_frames <= 1U) ?
            s_patch_activity_raw :
            ema_blend(s_patch_activity_floor, s_patch_activity_raw, 0.08f);
        s_patch_activity_jitter = ema_blend(s_patch_activity_jitter,
            fabsf(s_patch_activity_raw - s_patch_activity_floor), 0.10f);
        s_shape_shift_score = 0.0f;
        s_rhythm_motion_score = 0.0f;
        s_patch_activity_score = 0.0f;
        s_micro_activity = false;
        s_micro_activity_frames = 0;
        return;
    }

    s_shape_shift_ref = fmaxf(0.035f,
        s_shape_shift_floor + (s_shape_shift_jitter * 2.6f));
    s_rhythm_motion_ref = fmaxf(0.020f,
        s_rhythm_motion_floor + (s_rhythm_motion_jitter * 2.8f));
    s_patch_activity_ref = fmaxf(0.80f,
        s_patch_activity_floor + (s_patch_activity_jitter * 2.4f));
    s_shape_shift_score = ema_blend(s_shape_shift_score,
        s_shape_shift_raw / s_shape_shift_ref, 0.22f);
    s_rhythm_motion_score = ema_blend(s_rhythm_motion_score,
        s_rhythm_motion_raw / s_rhythm_motion_ref, 0.28f);
    s_patch_activity_score = ema_blend(s_patch_activity_score,
        s_patch_activity_raw / s_patch_activity_ref, 0.24f);

    micro_candidate = (s_hybrid_presence_state == CSI_PRESENCE_STATIC) &&
        (s_motion_score <= CSI_HYBRID_MOTION_CLEAR) &&
        (s_rhythm_motion_score <= CSI_VISUAL_RHYTHM_CLEAR) &&
        (s_selected_micro_score >= CSI_HYBRID_MICRO_ENTER);
    s_micro_activity_frames = micro_candidate ?
        (uint8_t)fminf((float)UINT8_MAX, (float)(s_micro_activity_frames + 1)) : 0;
    if (s_micro_activity_frames >= CSI_MICRO_ACTIVITY_CONFIRM) {
        s_micro_activity = true;
    } else if ((s_selected_micro_score <= CSI_HYBRID_MICRO_CLEAR) ||
               (s_hybrid_presence_state != CSI_PRESENCE_STATIC)) {
        s_micro_activity = false;
    }
}

static void update_hybrid_detector_locked(void)
{
    bool motion_candidate;
    bool micro_candidate;
    bool static_candidate;
    bool clear_candidate;
    csi_presence_state_t new_state;

    if (!s_baseline_ready || s_selected_present_count == 0U) {
        s_hybrid_presence_state = CSI_PRESENCE_NO_PRESENCE;
        s_hybrid_motion_frames = 0;
        s_hybrid_micro_frames = 0;
        s_hybrid_static_frames = 0;
        s_hybrid_clear_frames = 0;
        s_hybrid_motion_hold_frames = 0;
        s_hybrid_static_hold_frames = 0;
        return;
    }

    motion_candidate = (s_motion_score >= CSI_HYBRID_MOTION_ENTER) ||
        (s_rhythm_motion_score >= CSI_VISUAL_RHYTHM_ENTER);
    micro_candidate = (s_selected_shift_score >= CSI_HYBRID_SHIFT_ENTER) &&
        (s_motion_score <= CSI_HYBRID_MOTION_CLEAR) &&
        (s_rhythm_motion_score <= CSI_VISUAL_RHYTHM_CLEAR) &&
        (s_selected_micro_score >= CSI_HYBRID_MICRO_ENTER);
    static_candidate = (s_selected_shift_score >= CSI_HYBRID_SHIFT_ENTER) &&
        (s_motion_score < CSI_HYBRID_MOTION_ENTER) &&
        (s_rhythm_motion_score < CSI_VISUAL_RHYTHM_ENTER);
    clear_candidate = (s_occupancy_score <= CSI_HYBRID_OCCUPANCY_CLEAR) &&
        (s_motion_score <= CSI_HYBRID_MOTION_CLEAR) &&
        (s_rhythm_motion_score <= CSI_VISUAL_RHYTHM_CLEAR) &&
        (s_hybrid_motion_hold_frames == 0U) &&
        (s_hybrid_static_hold_frames == 0U);

    s_hybrid_motion_frames = motion_candidate ?
        (uint8_t)fminf((float)UINT8_MAX, (float)(s_hybrid_motion_frames + 1)) : 0;
    s_hybrid_micro_frames = micro_candidate ?
        (uint8_t)fminf((float)UINT8_MAX, (float)(s_hybrid_micro_frames + 1)) : 0;
    s_hybrid_static_frames = static_candidate ?
        (uint8_t)fminf((float)UINT8_MAX, (float)(s_hybrid_static_frames + 1)) : 0;
    s_hybrid_clear_frames = clear_candidate ?
        (uint8_t)fminf((float)UINT8_MAX, (float)(s_hybrid_clear_frames + 1)) : 0;
    if (motion_candidate) {
        s_hybrid_motion_hold_frames = CSI_HYBRID_MOTION_HOLD;
    } else if (s_hybrid_motion_hold_frames > 0U) {
        s_hybrid_motion_hold_frames--;
    }
    if (static_candidate) {
        s_hybrid_static_hold_frames = CSI_HYBRID_STATIC_HOLD;
    } else if (s_hybrid_static_hold_frames > 0U) {
        s_hybrid_static_hold_frames--;
    }

    new_state = s_hybrid_presence_state;
    if ((s_hybrid_motion_frames >= CSI_HYBRID_MOTION_CONFIRM) ||
        ((s_hybrid_presence_state == CSI_PRESENCE_MOTION) &&
         (s_hybrid_motion_hold_frames > 0U))) {
        new_state = CSI_PRESENCE_MOTION;
    } else if ((s_hybrid_static_frames >= CSI_HYBRID_STATIC_CONFIRM) ||
               ((s_hybrid_presence_state == CSI_PRESENCE_STATIC) &&
                (s_hybrid_static_hold_frames > 0U))) {
        new_state = CSI_PRESENCE_STATIC;
    } else if (s_hybrid_clear_frames >= CSI_HYBRID_CLEAR_CONFIRM) {
        new_state = CSI_PRESENCE_NO_PRESENCE;
    }

    s_hybrid_presence_state = new_state;
}

static const char *guided_stage_to_string(csi_setup_stage_t stage)
{
    switch (stage) {
    case CSI_SETUP_EMPTY_ROOM:
        return "empty_room";
    case CSI_SETUP_WALK_TEST:
        return "walk_test";
    case CSI_SETUP_STILL_TEST:
        return "still_test";
    case CSI_SETUP_CLEAR_TEST:
        return "clear_test";
    case CSI_SETUP_COMPLETE:
        return "complete";
    case CSI_SETUP_IDLE:
    default:
        return "idle";
    }
}

static void reset_guided_setup_locked(void)
{
    bool manual_sampling = s_guided_manual_sampling;

    memset(s_guided_metrics, 0, sizeof(s_guided_metrics));
    s_guided_stage = CSI_SETUP_IDLE;
    s_guided_active = false;
    s_guided_sampling = false;
    s_guided_settle_frames_remaining = 0;
    s_guided_suggested_motion_threshold = 0.0f;
    s_guided_suggested_presence_enter = 0.0f;
    s_guided_motion_contrast = 0.0f;
    s_guided_static_contrast = 0.0f;
    s_guided_profile_applied = false;
    s_guided_manual_sampling = manual_sampling;
    strlcpy(s_guided_profile_mode, "incomplete", sizeof(s_guided_profile_mode));
    strlcpy(s_guided_result, "incomplete", sizeof(s_guided_result));
}

static void clear_guided_stage_metrics_locked(csi_setup_stage_t stage)
{
    if (stage <= CSI_SETUP_COMPLETE) {
        memset(&s_guided_metrics[stage], 0, sizeof(s_guided_metrics[stage]));
    }
}

static float setup_metric_mean(float sum, uint16_t samples)
{
    return (samples == 0U) ? 0.0f : (sum / (float)samples);
}

static void prepare_guided_stage_locked(csi_setup_stage_t stage)
{
    s_guided_stage = stage;
    s_guided_sampling = false;
    s_guided_settle_frames_remaining = CSI_GUIDED_SETTLE_FRAMES;
    clear_guided_stage_metrics_locked(stage);
}

static void maybe_start_guided_sampling_locked(void)
{
    if (!s_guided_active || s_guided_manual_sampling ||
        (s_guided_stage == CSI_SETUP_IDLE) ||
        (s_guided_stage == CSI_SETUP_COMPLETE)) {
        return;
    }

    if (s_guided_settle_frames_remaining > 0U) {
        s_guided_settle_frames_remaining--;
        return;
    }

    s_guided_sampling = true;
}

static void update_guided_suggestions_locked(void)
{
    csi_setup_metrics_t *empty = &s_guided_metrics[CSI_SETUP_EMPTY_ROOM];
    csi_setup_metrics_t *walk = &s_guided_metrics[CSI_SETUP_WALK_TEST];
    csi_setup_metrics_t *still = &s_guided_metrics[CSI_SETUP_STILL_TEST];
    csi_setup_metrics_t *clear = &s_guided_metrics[CSI_SETUP_CLEAR_TEST];
    float empty_motion = setup_metric_mean(empty->motion_sum, empty->samples);
    float walk_motion_mean = setup_metric_mean(walk->motion_sum, walk->samples);
    float still_occ = setup_metric_mean(still->occupancy_sum, still->samples);
    float clear_occ = setup_metric_mean(clear->occupancy_sum, clear->samples);
    float motion_contrast = walk->motion_peak - empty_motion;
    float static_contrast = still_occ - clear_occ;
    bool has_motion_data = walk->samples > 0U;
    bool has_static_data = (still->samples > 0U) && (clear->samples > 0U);
    bool static_unreliable = !has_static_data ||
        (static_contrast < 0.18f) ||
        (still_occ <= (clear_occ + 0.10f));
    float motion_suggestion;
    float presence_suggestion;

    s_guided_motion_contrast = motion_contrast;
    s_guided_static_contrast = static_contrast;

    if (!has_motion_data || !has_static_data) {
        motion_suggestion = 0.0f;
        presence_suggestion = 0.0f;
        strlcpy(s_guided_profile_mode, "incomplete",
                sizeof(s_guided_profile_mode));
    } else if (motion_contrast < 0.45f) {
        motion_suggestion = fmaxf(1.0f, walk->motion_peak * 0.55f);
        presence_suggestion = 6.0f;
        strlcpy(s_guided_profile_mode, "rejected",
                sizeof(s_guided_profile_mode));
    } else if (static_unreliable) {
        motion_suggestion = clampf_local(walk->motion_peak * 0.55f,
                                         1.05f, 1.35f);
        presence_suggestion = 6.0f;
        strlcpy(s_guided_profile_mode, "motion_only",
                sizeof(s_guided_profile_mode));
    } else {
        motion_suggestion = clampf_local(fmaxf(empty_motion * 1.45f,
                                               walk->motion_peak * 0.52f),
                                         1.05f, 2.20f);
        presence_suggestion = clampf_local(clear_occ + (static_contrast * 0.55f),
                                           0.80f, 3.00f);
        strlcpy(s_guided_profile_mode, "balanced",
                sizeof(s_guided_profile_mode));
    }

    s_guided_suggested_motion_threshold =
        clampf_local(motion_suggestion, CSI_MIN_MOTION_THRESHOLD,
                     CSI_MAX_MOTION_THRESHOLD);
    s_guided_suggested_presence_enter =
        clampf_local(presence_suggestion, 0.80f, 6.00f);
    if (s_guided_active) {
        s_guided_profile_applied = false;
    }

    if ((walk->samples == 0U) || (still->samples == 0U) || (clear->samples == 0U)) {
        strlcpy(s_guided_result, "incomplete", sizeof(s_guided_result));
    } else if (strcmp(s_guided_profile_mode, "rejected") == 0) {
        strlcpy(s_guided_result, "weak", sizeof(s_guided_result));
    } else if (strcmp(s_guided_profile_mode, "motion_only") == 0) {
        strlcpy(s_guided_result, "motion_only", sizeof(s_guided_result));
    } else if ((motion_contrast >= 1.40f) && (static_contrast >= 0.45f)) {
        strlcpy(s_guided_result, "strong", sizeof(s_guided_result));
    } else if ((motion_contrast >= 0.90f) && (static_contrast >= 0.25f)) {
        strlcpy(s_guided_result, "good", sizeof(s_guided_result));
    } else if ((motion_contrast >= 0.55f) || (static_contrast >= 0.18f)) {
        strlcpy(s_guided_result, "fair", sizeof(s_guided_result));
    } else {
        strlcpy(s_guided_result, "weak", sizeof(s_guided_result));
    }

    (void)walk_motion_mean;
}

static void accumulate_guided_setup_locked(void)
{
    csi_setup_metrics_t *metrics;

    maybe_start_guided_sampling_locked();
    if (!s_guided_active || !s_baseline_ready ||
        !s_guided_sampling ||
        (s_guided_stage == CSI_SETUP_IDLE) ||
        (s_guided_stage == CSI_SETUP_COMPLETE)) {
        return;
    }

    metrics = &s_guided_metrics[s_guided_stage];
    if (metrics->samples == UINT16_MAX) {
        return;
    }
    metrics->samples++;
    metrics->occupancy_sum += s_occupancy_score;
    metrics->motion_sum += s_motion_score;
    metrics->micromotion_sum += s_micromotion_score;
    metrics->stability_sum += s_stability_score;
    metrics->occupancy_peak = fmaxf(metrics->occupancy_peak, s_occupancy_score);
    metrics->motion_peak = fmaxf(metrics->motion_peak, s_motion_score);
    update_guided_suggestions_locked();
}

static float score_rssi_quality(float rssi_avg)
{
    if (rssi_avg <= -126.0f) {
        return 0.0f;
    }
    if (rssi_avg < -88.0f) {
        return 0.0f;
    }
    if (rssi_avg < -72.0f) {
        return clampf_local((rssi_avg + 88.0f) / 16.0f, 0.0f, 1.0f);
    }
    if (rssi_avg > -22.0f) {
        return 0.55f;
    }
    if (rssi_avg > -32.0f) {
        return 0.75f;
    }
    return 1.0f;
}

static void fill_setup_quality_locked(csi_status_t *out)
{
    float rate_score = clampf_local((s_packet_rate_hz - 10.0f) / 35.0f,
                                    0.0f, 1.0f);
    float jitter_score = 1.0f - clampf_local(s_packet_rate_jitter_hz / 18.0f,
                                             0.0f, 1.0f);
    float rssi_score = score_rssi_quality(s_rssi_avg);
    float subcarrier_score = clampf_local((float)s_valid_sc / 34.0f, 0.0f, 1.0f);
    float baseline_score = s_baseline_ready ?
        (1.0f - clampf_local(s_empty_jitter / 0.90f, 0.0f, 1.0f)) : 0.35f;
    float drop_score = 1.0f - clampf_local((float)s_csi_queue_drops / 24.0f,
                                           0.0f, 1.0f);
    float score =
        (rate_score * 26.0f) +
        (jitter_score * 16.0f) +
        (rssi_score * 18.0f) +
        (subcarrier_score * 16.0f) +
        (baseline_score * 16.0f) +
        (drop_score * 8.0f);

    out->setup_quality_score = clampf_local(score, 0.0f, 100.0f);
    out->deployment_ready = s_detection_enabled && s_baseline_ready &&
        (out->setup_quality_score >= 65.0f);

    if (out->setup_quality_score >= 85.0f) {
        strlcpy(out->setup_quality, "excellent", sizeof(out->setup_quality));
    } else if (out->setup_quality_score >= 70.0f) {
        strlcpy(out->setup_quality, "good", sizeof(out->setup_quality));
    } else if (out->setup_quality_score >= 45.0f) {
        strlcpy(out->setup_quality, "fair", sizeof(out->setup_quality));
    } else {
        strlcpy(out->setup_quality, "poor", sizeof(out->setup_quality));
    }

    if (s_wifi_role == CSI_WIFI_ROLE_DEDICATED_AP) {
        out->deployment_ready = true;
        out->setup_quality_score = 100.0f;
        strlcpy(out->setup_quality, "excellent", sizeof(out->setup_quality));
        strlcpy(out->setup_step, "Place the sensor ESP32 across the sensing area.",
                sizeof(out->setup_step));
        strlcpy(out->setup_reason, "dedicated_ap_provider",
                sizeof(out->setup_reason));
    } else if (!s_wifi_is_connected) {
        strlcpy(out->setup_step, "Wait for Wi-Fi connection.",
                sizeof(out->setup_step));
        strlcpy(out->setup_reason, "wifi_disconnected",
                sizeof(out->setup_reason));
    } else if (!s_detection_enabled) {
        strlcpy(out->setup_step, "Start detection with the sensing area empty.",
                sizeof(out->setup_step));
        strlcpy(out->setup_reason, "detection_paused",
                sizeof(out->setup_reason));
    } else if (s_calibration_guard_blocked && !s_force_calibration_enabled) {
        strlcpy(out->setup_step, "Clear the sensing area and recalibrate.",
                sizeof(out->setup_step));
        strlcpy(out->setup_reason, "calibration_guard_blocked",
                sizeof(out->setup_reason));
    } else if (!s_baseline_ready) {
        strlcpy(out->setup_step, "Keep the area empty until baseline reaches 100%.",
                sizeof(out->setup_step));
        strlcpy(out->setup_reason, "empty_room_calibration",
                sizeof(out->setup_reason));
    } else if (s_packet_rate_hz < 15.0f) {
        strlcpy(out->setup_step, "Improve packet cadence or use dedicated AP mode.",
                sizeof(out->setup_step));
        strlcpy(out->setup_reason, "low_packet_rate",
                sizeof(out->setup_reason));
    } else if (s_packet_rate_jitter_hz > 18.0f) {
        strlcpy(out->setup_step, "Reduce bursty traffic or move to a cleaner channel.",
                sizeof(out->setup_step));
        strlcpy(out->setup_reason, "unstable_packet_rate",
                sizeof(out->setup_reason));
    } else if (s_rssi_avg < -82.0f) {
        strlcpy(out->setup_step, "Move the ESP32 and AP closer or improve line of sight.",
                sizeof(out->setup_step));
        strlcpy(out->setup_reason, "weak_rssi",
                sizeof(out->setup_reason));
    } else if (s_rssi_avg > -25.0f) {
        strlcpy(out->setup_step, "Increase ESP32-to-AP separation for better multipath contrast.",
                sizeof(out->setup_step));
        strlcpy(out->setup_reason, "rssi_too_strong",
                sizeof(out->setup_reason));
    } else if (s_valid_sc < CSI_MIN_EFFECTIVE_SUBCARRIERS) {
        strlcpy(out->setup_step, "Adjust placement until more subcarriers are usable.",
                sizeof(out->setup_step));
        strlcpy(out->setup_reason, "too_few_subcarriers",
                sizeof(out->setup_reason));
    } else if (s_empty_jitter > 0.90f) {
        strlcpy(out->setup_step, "Remove periodic movement, then recalibrate the empty room.",
                sizeof(out->setup_step));
        strlcpy(out->setup_reason, "noisy_baseline",
                sizeof(out->setup_reason));
    } else if (out->deployment_ready) {
        strlcpy(out->setup_step, "Walk through, stand still, then leave to validate this location.",
                sizeof(out->setup_step));
        strlcpy(out->setup_reason, "ready_for_validation",
                sizeof(out->setup_reason));
    } else {
        strlcpy(out->setup_step, "Improve placement until setup quality reaches good.",
                sizeof(out->setup_step));
        strlcpy(out->setup_reason, "quality_below_target",
                sizeof(out->setup_reason));
    }
}

static void update_presence_state_locked(float occupancy_energy,
                                         float motion_avg_energy,
                                         float motion_energy,
                                         const float *display_amps,
                                         const float *slow_snapshot,
                                         uint8_t num_sc,
                                         uint8_t valid_sc,
                                         uint64_t timestamp_us,
                                         int8_t rssi)
{
    float occupancy_ref;
    float motion_ref;
    float raw_occupancy_score;
    float raw_micromotion_score;
    float raw_motion_score;
    float raw_stability_score;
    float presence_enter;
    float presence_clear;
    float micro_enter;
    float micro_clear;
    float motion_enter;
    float motion_clear;
    float static_motion_cap_enter;
    float static_motion_cap_hold;
    float static_micro_cap_enter;
    float static_micro_cap_hold;
    float micro_motion_floor;
    bool macro_candidate;
    bool micro_candidate;
    bool static_candidate;
    bool maintain_motion_state;
    bool maintain_micro_state;
    bool maintain_static_state;
    bool empty_candidate;
    bool quiet_scene;
    csi_presence_state_t new_state;

    if (valid_sc < CSI_MIN_EFFECTIVE_SUBCARRIERS) {
        return;
    }

    s_processed_frames++;
    update_live_packet_stats_locked(timestamp_us, rssi, valid_sc);

    if (s_calibration_frames_remaining > 0) {
        bool occupied_calibration_scene = false;

        if (s_calibration_guard_cooldown > 0U) {
            s_calibration_guard_cooldown--;
        }
        adapt_empty_baseline_locked(slow_snapshot, num_sc,
                                    CSI_EMPTY_BASELINE_FAST_ALPHA);
        s_empty_floor = (s_processed_frames == 1) ? occupancy_energy :
            ema_blend(s_empty_floor, occupancy_energy, CSI_EMPTY_FLOOR_ALPHA);
        s_empty_jitter = (s_processed_frames == 1) ? 0.10f :
            ema_blend(s_empty_jitter, fabsf(occupancy_energy - s_empty_floor),
                      CSI_EMPTY_JITTER_ALPHA);
        s_motion_floor = (s_processed_frames == 1) ? motion_energy :
            ema_blend(s_motion_floor, motion_energy, CSI_MOTION_FLOOR_ALPHA);
        s_motion_jitter = (s_processed_frames == 1) ? 0.10f :
            ema_blend(s_motion_jitter, fabsf(motion_energy - s_motion_floor),
                      CSI_MOTION_JITTER_ALPHA);

        s_occupancy_score = 0.0f;
        s_micromotion_score = 0.0f;
        s_motion_score = 0.0f;
        s_stability_score = 1.0f;
        s_last_occupancy_energy = occupancy_energy;
        s_last_motion_avg_energy = motion_avg_energy;
        s_last_motion_energy = motion_energy;
        s_last_occupancy_ref = s_empty_floor;
        s_last_motion_ref = s_motion_floor;
        s_last_motion_enter = s_detection_profile.motion_enter;
        s_last_motion_clear = 0.0f;
        s_last_micro_enter = 0.0f;
        s_last_micro_clear = 0.0f;
        s_last_presence_enter = 0.0f;
        s_last_presence_clear = 0.0f;
        s_presence_state = CSI_PRESENCE_NO_PRESENCE;
        push_display_amplitudes_locked(display_amps, num_sc);

        if (!s_force_calibration_enabled && (s_processed_frames > 6U)) {
            float occupancy_guard =
                s_empty_floor + fmaxf(0.18f, s_empty_jitter * 2.8f);
            float motion_guard =
                s_motion_floor + fmaxf(0.18f, s_motion_jitter * 3.2f);

            occupied_calibration_scene =
                (occupancy_energy > occupancy_guard) ||
                (motion_energy > motion_guard) ||
                (motion_avg_energy > (motion_guard * 0.82f));
        }

        if (occupied_calibration_scene) {
            if (s_calibration_guard_cooldown == 0U) {
                ESP_LOGW(TAG,
                         "Calibration blocked by occupied scene "
                         "(occ=%.2f motion=%.2f avg=%.2f). "
                         "Clear the area or enable force calibration.",
                         occupancy_energy, motion_energy, motion_avg_energy);
            }
            restart_calibration_locked(true);
            return;
        }

        s_calibration_frames_remaining--;
        if (s_calibration_frames_remaining == 0) {
            s_baseline_ready = true;
            s_calibration_guard_blocked = false;
            ESP_LOGI(TAG, "Empty-room calibration complete.");
        }
        return;
    }

    occupancy_ref = fmaxf(CSI_MIN_OCCUPANCY_REF,
                          s_empty_floor + (s_empty_jitter * CSI_OCCUPANCY_JITTER_GAIN));
    motion_ref = fmaxf(CSI_MIN_MOTION_REF,
                       s_motion_floor + (s_motion_jitter * CSI_MOTION_JITTER_GAIN));

    raw_occupancy_score = occupancy_energy / occupancy_ref;
    raw_micromotion_score = motion_avg_energy / motion_ref;
    raw_motion_score = motion_energy / motion_ref;
    raw_stability_score = occupancy_energy /
        (occupancy_energy + motion_avg_energy + 0.001f);

    smooth_scores_locked(raw_occupancy_score, raw_micromotion_score,
                         raw_motion_score, raw_stability_score);
    update_hybrid_detector_locked();
    accumulate_guided_setup_locked();

    motion_enter = s_detection_profile.motion_enter;
    motion_clear = s_detection_profile.motion_clear;
    micro_enter = s_detection_profile.micro_enter;
    micro_clear = s_detection_profile.micro_clear;
    presence_enter = s_detection_profile.presence_enter;
    presence_clear = s_detection_profile.presence_clear;
    static_motion_cap_enter = fmaxf(0.70f, motion_clear * 0.88f);
    static_motion_cap_hold = fmaxf(0.76f, motion_clear * 0.96f);
    static_micro_cap_enter = fmaxf(0.52f, micro_enter * 0.88f);
    static_micro_cap_hold = fmaxf(0.58f, micro_enter * 0.96f);
    micro_motion_floor = fmaxf(0.42f, micro_clear * 0.96f);
    s_last_occupancy_energy = occupancy_energy;
    s_last_motion_avg_energy = motion_avg_energy;
    s_last_motion_energy = motion_energy;
    s_last_occupancy_ref = occupancy_ref;
    s_last_motion_ref = motion_ref;
    s_last_motion_enter = motion_enter;
    s_last_motion_clear = motion_clear;
    s_last_micro_enter = micro_enter;
    s_last_micro_clear = micro_clear;
    s_last_presence_enter = presence_enter;
    s_last_presence_clear = presence_clear;
    macro_candidate = (s_motion_score >= motion_enter) &&
        ((s_motion_score >= (s_micromotion_score + 0.12f)) ||
         (s_motion_score >= (motion_enter * 1.10f)) ||
         (s_presence_state == CSI_PRESENCE_MOTION));
    micro_candidate = (s_micromotion_score >= micro_enter) &&
        (s_motion_score < (motion_enter * 0.96f)) &&
        ((s_occupancy_score >= (presence_clear * 0.90f)) ||
         (s_presence_state != CSI_PRESENCE_NO_PRESENCE)) &&
        ((s_motion_score >= micro_motion_floor) ||
         (s_micromotion_score >= (micro_enter * 1.08f)) ||
         (s_presence_state == CSI_PRESENCE_MICROMOTION));
    static_candidate = ((s_occupancy_score >= presence_enter) &&
        (s_stability_score >= 0.52f) &&
        (s_motion_score <= static_motion_cap_enter) &&
        (s_micromotion_score <= static_micro_cap_enter)) ||
        ((s_presence_state != CSI_PRESENCE_NO_PRESENCE) &&
         (s_occupancy_score >= (presence_clear * 0.98f)) &&
         (s_stability_score >= 0.42f) &&
         (s_motion_score <= static_motion_cap_hold) &&
         (s_micromotion_score <= static_micro_cap_hold));
    maintain_motion_state = (s_presence_state == CSI_PRESENCE_MOTION) &&
        ((s_motion_score >= motion_clear) ||
         (s_micromotion_score >= (micro_enter * 0.95f)));
    maintain_micro_state = (s_presence_state == CSI_PRESENCE_MICROMOTION) &&
        (s_micromotion_score >= micro_clear) &&
        (s_occupancy_score >= (presence_clear * 0.88f)) &&
        (s_motion_score <= motion_enter);
    maintain_static_state = (s_presence_state == CSI_PRESENCE_STATIC) &&
        (s_occupancy_score >= (presence_clear * 0.98f)) &&
        (s_stability_score >= 0.42f) &&
        (s_motion_score <= static_motion_cap_hold) &&
        (s_micromotion_score <= static_micro_cap_hold);

    if (macro_candidate || micro_candidate || static_candidate ||
        maintain_motion_state || maintain_micro_state || maintain_static_state) {
        s_presence_hold_frames = CSI_PRESENCE_HOLD_FRAMES;
    } else if (s_presence_hold_frames > 0) {
        s_presence_hold_frames--;
    }

    empty_candidate = (s_occupancy_score <= (presence_clear * 0.88f)) &&
        (s_micromotion_score <= micro_clear) &&
        (s_motion_score <= motion_clear) &&
        (s_presence_hold_frames == 0);

    s_macro_frames = macro_candidate ?
        (uint8_t)fminf((float)UINT8_MAX, (float)(s_macro_frames + 1)) : 0;
    s_micro_frames = micro_candidate ?
        (uint8_t)fminf((float)UINT8_MAX, (float)(s_micro_frames + 1)) : 0;
    s_static_frames = static_candidate ?
        (uint8_t)fminf((float)UINT8_MAX, (float)(s_static_frames + 1)) : 0;
    s_clear_frames = empty_candidate ?
        (uint8_t)fminf((float)UINT8_MAX, (float)(s_clear_frames + 1)) : 0;

    new_state = s_presence_state;
    if ((s_macro_frames >= s_confirm_motion) || maintain_motion_state) {
        new_state = CSI_PRESENCE_MOTION;
    } else if ((s_micro_frames >= CSI_MICRO_CONFIRM_FRAMES) || maintain_micro_state) {
        new_state = CSI_PRESENCE_MICROMOTION;
    } else if ((s_static_frames >= CSI_STATIC_CONFIRM_FRAMES) || maintain_static_state) {
        new_state = CSI_PRESENCE_STATIC;
    } else if (s_clear_frames >= s_confirm_clear) {
        new_state = CSI_PRESENCE_NO_PRESENCE;
    }

    if ((s_presence_state != new_state) && (new_state != CSI_PRESENCE_NO_PRESENCE)) {
        s_state_event_count++;
    }
    if (s_presence_state != new_state) {
        ESP_LOGI(TAG,
                 "Radar state -> %s (occ=%.2f micro=%.2f motion=%.2f stability=%.2f rssi=%d)",
                 csi_presence_state_to_string(new_state),
                 s_occupancy_score,
                 s_micromotion_score,
                 s_motion_score,
                 s_stability_score,
                 rssi);
        s_presence_state = new_state;
    }

    if ((s_presence_state == CSI_PRESENCE_NO_PRESENCE) &&
        (s_motion_score >= micro_enter) &&
        (s_occupancy_score < presence_clear) &&
        (s_disturbance_cooldown == 0)) {
        s_disturbance_event_count++;
        s_disturbance_cooldown = CSI_DISTURBANCE_COOLDOWN_FRAMES;
        ESP_LOGW(TAG,
                 "Environmental disturbance detected (occ=%.2f motion=%.2f). "
                 "Check fans, doors, router traffic, or changing multipath.",
                 s_occupancy_score, s_motion_score);
    } else if (s_disturbance_cooldown > 0) {
        s_disturbance_cooldown--;
    }

    quiet_scene = (s_presence_state == CSI_PRESENCE_NO_PRESENCE) &&
        (s_occupancy_score <= presence_clear) &&
        (s_motion_score <= micro_clear);

    if (quiet_scene) {
        adapt_empty_baseline_locked(slow_snapshot, num_sc,
                                    CSI_EMPTY_BASELINE_DRIFT_ALPHA);
        s_empty_floor = ema_blend(s_empty_floor, occupancy_energy,
                                  CSI_EMPTY_FLOOR_ALPHA);
        s_empty_jitter = ema_blend(s_empty_jitter,
                                   fabsf(occupancy_energy - s_empty_floor),
                                   CSI_EMPTY_JITTER_ALPHA);
        s_motion_floor = ema_blend(s_motion_floor, motion_energy,
                                   CSI_MOTION_FLOOR_ALPHA);
        s_motion_jitter = ema_blend(s_motion_jitter,
                                    fabsf(motion_energy - s_motion_floor),
                                    CSI_MOTION_JITTER_ALPHA);
    } else {
        adapt_empty_baseline_locked(slow_snapshot, num_sc,
                                    CSI_EMPTY_BASELINE_HOLD_ALPHA);
        s_empty_jitter = ema_blend(s_empty_jitter,
                                   fabsf(occupancy_energy - s_empty_floor),
                                   0.01f);
        s_motion_jitter = ema_blend(s_motion_jitter,
                                    fabsf(motion_energy - s_motion_floor),
                                    0.01f);
    }

    s_empty_jitter = fmaxf(s_empty_jitter, 0.02f);
    s_motion_jitter = fmaxf(s_motion_jitter, 0.02f);
    push_display_amplitudes_locked(display_amps, num_sc);
}

static void log_raw_frame_locked(const csi_frame_t *frame)
{
    bool serial_enabled;
    bool capture_enabled;
    bool log_stream_enabled;
    bool emit_serial;
    bool enqueue_capture;
    bool enqueue_log_stream;
    uint16_t offset;
    uint8_t num_sc;
    uint8_t sc;
    size_t pos = 0;

    serial_enabled = s_raw_logging_enabled;
    capture_enabled = http_server_capture_is_enabled();
    log_stream_enabled = http_server_log_stream_is_enabled();
    emit_serial = false;
    enqueue_capture = false;
    enqueue_log_stream = false;

    if ((!serial_enabled && !capture_enabled && !log_stream_enabled) || !frame) {
        return;
    }

    offset = (frame->first_word_invalid && frame->len >= 4) ? 4 : 0;
    if (frame->len <= offset) {
        return;
    }

    num_sc = (uint8_t)((frame->len - offset) / 2);
    if (num_sc > CSI_MAX_SUBCARRIERS) {
        num_sc = CSI_MAX_SUBCARRIERS;
    }

    if (serial_enabled && !s_raw_log_header_emitted) {
        printf("CSI_DATA_HEADER,"
               "timestamp_us,rssi,num_subcarriers,valid_subcarriers,processed_frames,"
               "packet_rate_hz,detection_enabled,baseline_ready,calibration_percent,presence_state,"
               "manual_annotation,force_calibration_enabled,calibration_guard_blocked,calibration_guard_hits,"
               "occupancy_score,micromotion_score,motion_score,stability_score,"
               "occupancy_energy,motion_avg_energy,motion_energy,"
               "occupancy_ref,motion_ref,"
               "motion_threshold,motion_enter,motion_clear,"
               "micro_enter,micro_clear,presence_enter,presence_clear,"
               "selected_presence_state,selected_shift_score,selected_macro_score,selected_micro_score,"
               "selected_shift_raw,selected_macro_raw,selected_micro_raw,"
               "selected_shift_ref,selected_macro_ref,selected_micro_ref,selected_subcarrier_count,"
               "shape_shift_score,rhythm_motion_score,patch_activity_score,"
               "shape_shift_raw,rhythm_motion_raw,patch_activity_raw,"
               "shape_shift_ref,rhythm_motion_ref,patch_activity_ref,micro_activity,micro_activity_frames,"
               "hybrid_presence_state,hybrid_motion_frames,hybrid_micro_frames,hybrid_static_frames,hybrid_clear_frames,hybrid_motion_hold_frames,hybrid_static_hold_frames,"
               "macro_frames,micro_frames,static_frames,clear_frames,presence_hold_frames,"
               "state_event_count,disturbance_count,notch_hz,last_timestamp_us,src_mac,iq_vector\n");
        s_raw_log_header_emitted = true;
    }

    if (serial_enabled &&
        ((s_last_serial_log_us == 0) ||
         ((frame->timestamp_us - s_last_serial_log_us) >=
          CSI_SERIAL_LOG_MIN_INTERVAL_US))) {
        emit_serial = true;
        s_last_serial_log_us = frame->timestamp_us;
    }

    if (capture_enabled &&
        ((s_last_capture_enqueue_us == 0) ||
         ((frame->timestamp_us - s_last_capture_enqueue_us) >=
          CSI_BROWSER_CAPTURE_MIN_INTERVAL_US))) {
        enqueue_capture = true;
        s_last_capture_enqueue_us = frame->timestamp_us;
    }

    if (log_stream_enabled) {
        enqueue_log_stream = true;
    }

    if (!emit_serial && !enqueue_capture && !enqueue_log_stream) {
        return;
    }

    pos += (size_t)snprintf(
        s_csv_row_buf + pos, sizeof(s_csv_row_buf) - pos,
        "CSI_DATA,%llu,%d,%u,%u,%lu,%.3f,%u,%u,%u,%s,"
        "%s,%u,%u,%lu,"
        "%.3f,%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,"
        "%.3f,%.3f,"
        "%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,%.3f,"
        "%s,%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,%u,"
        "%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,%u,%u,"
        "%s,%u,%u,%u,%u,%u,%u,"
        "%u,%u,%u,%u,%u,"
        "%lu,%lu,%.3f,%llu,"
        "%02x:%02x:%02x:%02x:%02x:%02x,",
        (unsigned long long)frame->timestamp_us,
        frame->rssi,
        (unsigned)num_sc,
        (unsigned)s_valid_sc,
        (unsigned long)s_processed_frames,
        (double)s_packet_rate_hz,
        s_detection_enabled ? 1U : 0U,
        s_baseline_ready ? 1U : 0U,
        s_baseline_ready ? 100U :
            (unsigned)(100U - ((s_calibration_frames_remaining * 100U) /
                               CSI_BASELINE_CALIBRATION_FRAMES)),
        csi_presence_state_to_string(s_presence_state),
        s_manual_annotation,
        s_force_calibration_enabled ? 1U : 0U,
        s_calibration_guard_blocked ? 1U : 0U,
        (unsigned long)s_calibration_guard_hits,
        (double)s_occupancy_score,
        (double)s_micromotion_score,
        (double)s_motion_score,
        (double)s_stability_score,
        (double)s_last_occupancy_energy,
        (double)s_last_motion_avg_energy,
        (double)s_last_motion_energy,
        (double)s_last_occupancy_ref,
        (double)s_last_motion_ref,
        (double)s_motion_threshold,
        (double)s_last_motion_enter,
        (double)s_last_motion_clear,
        (double)s_last_micro_enter,
        (double)s_last_micro_clear,
        (double)s_last_presence_enter,
        (double)s_last_presence_clear,
        csi_presence_state_to_string(s_selected_presence_state),
        (double)s_selected_shift_score,
        (double)s_selected_macro_score,
        (double)s_selected_micro_score,
        (double)s_selected_shift_raw,
        (double)s_selected_macro_raw,
        (double)s_selected_micro_raw,
        (double)s_selected_shift_ref,
        (double)s_selected_macro_ref,
        (double)s_selected_micro_ref,
        (unsigned)s_selected_present_count,
        (double)s_shape_shift_score,
        (double)s_rhythm_motion_score,
        (double)s_patch_activity_score,
        (double)s_shape_shift_raw,
        (double)s_rhythm_motion_raw,
        (double)s_patch_activity_raw,
        (double)s_shape_shift_ref,
        (double)s_rhythm_motion_ref,
        (double)s_patch_activity_ref,
        s_micro_activity ? 1U : 0U,
        (unsigned)s_micro_activity_frames,
        csi_presence_state_to_string(s_hybrid_presence_state),
        (unsigned)s_hybrid_motion_frames,
        (unsigned)s_hybrid_micro_frames,
        (unsigned)s_hybrid_static_frames,
        (unsigned)s_hybrid_clear_frames,
        (unsigned)s_hybrid_motion_hold_frames,
        (unsigned)s_hybrid_static_hold_frames,
        (unsigned)s_macro_frames,
        (unsigned)s_micro_frames,
        (unsigned)s_static_frames,
        (unsigned)s_clear_frames,
        (unsigned)s_presence_hold_frames,
        (unsigned long)s_state_event_count,
        (unsigned long)s_disturbance_event_count,
        (double)s_notch_freq_hz,
        (unsigned long long)s_last_timestamp_us,
        frame->src_mac[0], frame->src_mac[1], frame->src_mac[2],
        frame->src_mac[3], frame->src_mac[4], frame->src_mac[5]);

    for (sc = 0; sc < num_sc; sc++) {
        int16_t idx = (int16_t)(offset + (2 * sc));
        int imag = frame->raw[idx];
        int real = frame->raw[idx + 1];

        if (pos >= sizeof(s_csv_row_buf)) {
            break;
        }
        pos += (size_t)snprintf(
            s_csv_row_buf + pos, sizeof(s_csv_row_buf) - pos,
            sc > 0 ? "|%d:%d" : "%d:%d",
            imag, real);
    }
    s_csv_row_buf[sizeof(s_csv_row_buf) - 1U] = '\0';

    if (emit_serial) {
        printf("%s\n", s_csv_row_buf);
    }

    if (enqueue_capture) {
        http_server_capture_enqueue_csv(s_csv_row_buf);
    }

    if (enqueue_log_stream) {
        http_server_log_enqueue_csv(s_csv_row_buf);
    }
}

static void process_csi_frame(const csi_frame_t *frame)
{
    float raw_amps[CSI_DISPLAY_SUBCARRIERS];
    float display_amps[CSI_DISPLAY_SUBCARRIERS];
    float slow_snapshot[CSI_DISPLAY_SUBCARRIERS];
    uint16_t offset;
    uint8_t num_sc;
    uint8_t sc;
    float frame_mean = 0.0f;
    uint8_t valid_sc = 0;
    float occupancy_sum = 0.0f;
    float motion_sum = 0.0f;
    float peak_motion = 0.0f;
    bool initialized = s_feature_filters_ready;

    memset(raw_amps, 0, sizeof(raw_amps));
    memset(display_amps, 0, sizeof(display_amps));
    memset(slow_snapshot, 0, sizeof(slow_snapshot));

    offset = (frame->first_word_invalid && frame->len >= 4) ? 4 : 0;
    if (frame->len <= offset) {
        return;
    }

    num_sc = (uint8_t)((frame->len - offset) / 2);
    if (num_sc > CSI_MAX_SUBCARRIERS) {
        num_sc = CSI_MAX_SUBCARRIERS;
    }

    for (sc = 0; sc < num_sc; sc++) {
        float imag = (float)frame->raw[offset + (2 * sc)];
        float real = (float)frame->raw[offset + (2 * sc) + 1];
        float mag = sqrtf((imag * imag) + (real * real));

        raw_amps[sc] = mag;
        s_display_features[sc] = initialized ?
            ema_blend(s_display_features[sc], mag, CSI_DISPLAY_ALPHA) : mag;
        display_amps[sc] = s_display_features[sc];

        if (is_stable_subcarrier(sc)) {
            frame_mean += mag;
            valid_sc++;
        }
    }

    if (valid_sc < CSI_MIN_EFFECTIVE_SUBCARRIERS) {
        return;
    }

    update_live_packet_stats_locked(frame->timestamp_us, frame->rssi, valid_sc);
    push_display_amplitudes_locked(display_amps, num_sc);

    if (!s_detection_enabled) {
        log_raw_frame_locked(frame);
        return;
    }

    frame_mean /= (float)valid_sc;
    for (sc = 0; sc < num_sc; sc++) {
        float normalized;
        float filtered;
        float band_energy;

        if (!is_stable_subcarrier(sc)) {
            continue;
        }

        normalized = ((raw_amps[sc] - frame_mean) / fmaxf(frame_mean, 1.0f)) * 100.0f;
        filtered = apply_notch_locked(sc, normalized);

        if (!initialized) {
            s_fast_features[sc] = filtered;
            s_slow_features[sc] = filtered;
            s_empty_baseline[sc] = filtered;
            continue;
        }

        s_fast_features[sc] = ema_blend(s_fast_features[sc], filtered,
                                        CSI_FAST_FEATURE_ALPHA);
        s_slow_features[sc] = ema_blend(s_slow_features[sc], filtered,
                                        CSI_SLOW_FEATURE_ALPHA);
        slow_snapshot[sc] = s_slow_features[sc];

        band_energy = fabsf(s_fast_features[sc] - s_slow_features[sc]);
        motion_sum += band_energy;
        if (band_energy > peak_motion) {
            peak_motion = band_energy;
        }

        occupancy_sum += fabsf(s_slow_features[sc] - s_empty_baseline[sc]);
    }

    if (!initialized) {
        s_feature_filters_ready = true;
        log_raw_frame_locked(frame);
        return;
    }

    update_selected_detector_locked(raw_amps, num_sc, frame_mean,
                                    s_calibration_frames_remaining > 0U);
    update_visual_features_locked(raw_amps, num_sc, frame_mean,
                                    s_calibration_frames_remaining > 0U);
    update_presence_state_locked(
        occupancy_sum / (float)valid_sc,
        motion_sum / (float)valid_sc,
        ((motion_sum / (float)valid_sc) * 0.65f) + (peak_motion * 0.35f),
        display_amps,
        slow_snapshot,
        num_sc,
        valid_sc,
        frame->timestamp_us,
        frame->rssi);
    log_raw_frame_locked(frame);
}

static void csi_worker_task(void *arg)
{
    csi_frame_t frame;

    (void)arg;

    while (1) {
        if (xQueueReceive(s_csi_queue, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (!s_mutex) {
            continue;
        }

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        process_csi_frame(&frame);
        xSemaphoreGive(s_mutex);
    }
}

static void csi_rx_callback(void *ctx, wifi_csi_info_t *csi_info)
{
    csi_frame_t frame;
    uint16_t max_len;

    (void)ctx;

    if (!csi_info || !csi_info->buf || !s_csi_queue) {
        return;
    }

    if (s_ap_bssid_valid && memcmp(csi_info->mac, s_ap_bssid, 6) != 0) {
        return;
    }

    if (csi_info->rx_ctrl.rssi < CSI_FRAME_DROP_RSSI) {
        return;
    }

    memset(&frame, 0, sizeof(frame));
    frame.timestamp_us = (uint64_t)esp_timer_get_time();
    frame.first_word_invalid = csi_info->first_word_invalid;
    frame.rssi = csi_info->rx_ctrl.rssi;
    memcpy(frame.src_mac, csi_info->mac, sizeof(frame.src_mac));

    max_len = sizeof(frame.raw);
    frame.len = (csi_info->len > max_len) ? max_len : csi_info->len;
    memcpy(frame.raw, csi_info->buf, frame.len);

    if (xQueueSend(s_csi_queue, &frame, 0) != pdTRUE) {
        s_csi_queue_drops++;
    }
}

static void inject_timer_cb(TimerHandle_t xTimer)
{
    esp_err_t ret;

    (void)xTimer;

    if ((s_wifi_role != CSI_WIFI_ROLE_SENSOR_STA) || !s_null_frame_ready) {
        return;
    }

    ret = esp_wifi_80211_tx(WIFI_IF_STA, s_null_frame, sizeof(s_null_frame), true);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Null frame inject failed: %s", esp_err_to_name(ret));
    }
}

static void reconnect_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;

    if (s_wifi_role != CSI_WIFI_ROLE_SENSOR_STA) {
        return;
    }

    ESP_LOGI(TAG, "Backoff over. Retrying connection...");
    s_wifi_is_error = false;
    esp_wifi_connect();
}

static void ap_traffic_task(void *arg)
{
    struct sockaddr_in dest_addr;
    int sock = -1;
    int broadcast = 1;

    (void)arg;

    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(CSI_AP_TRAFFIC_PORT);
    dest_addr.sin_addr.s_addr = inet_addr("192.168.4.255");

    while (1) {
        TickType_t delay_ticks = pdMS_TO_TICKS(s_ap_broadcast_interval_ms);

        if (s_wifi_role != CSI_WIFI_ROLE_DEDICATED_AP) {
            break;
        }

        if (!s_wifi_is_connected || s_ap_client_count == 0U) {
            vTaskDelay(delay_ticks);
            continue;
        }

        if (sock < 0) {
            sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
            if (sock < 0) {
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
            if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST,
                           &broadcast, sizeof(broadcast)) != 0) {
                close(sock);
                sock = -1;
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
        }

        if (sendto(sock, s_ap_broadcast_payload,
                   sizeof(s_ap_broadcast_payload) - 1U, 0,
                   (struct sockaddr *)&dest_addr,
                   sizeof(dest_addr)) < 0) {
            close(sock);
            sock = -1;
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        vTaskDelay(delay_ticks);
    }

    if (sock >= 0) {
        close(sock);
    }
    s_ap_traffic_task = NULL;
    vTaskDelete(NULL);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;

    if (s_wifi_role == CSI_WIFI_ROLE_DEDICATED_AP) {
        if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
            s_wifi_is_connected = true;
            s_wifi_is_error = false;
            refresh_netif_ip_locked();
            xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            ESP_LOGW(TAG, "====================================");
            ESP_LOGW(TAG, "  DEDICATED AP ONLINE: http://%s/", s_remote_addr);
            ESP_LOGW(TAG, "====================================");
            return;
        }

        if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STOP) {
            s_wifi_is_connected = false;
            s_ap_client_count = 0;
            s_wifi_is_error = false;
            strlcpy(s_remote_addr, "0.0.0.0", sizeof(s_remote_addr));
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            return;
        }

        if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
            const wifi_event_ap_staconnected_t *event =
                (const wifi_event_ap_staconnected_t *)event_data;

            if (s_ap_client_count < UINT8_MAX) {
                s_ap_client_count++;
            }
            if (event) {
                ESP_LOGI(TAG, "AP client joined: " MACSTR " aid=%u",
                         MAC2STR(event->mac), event->aid);
            }
            return;
        }

        if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
            const wifi_event_ap_stadisconnected_t *event =
                (const wifi_event_ap_stadisconnected_t *)event_data;

            if (s_ap_client_count > 0U) {
                s_ap_client_count--;
            }
            if (event) {
                ESP_LOGI(TAG, "AP client left: " MACSTR " aid=%u",
                         MAC2STR(event->mac), event->aid);
            }
            return;
        }
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
        s_wifi_is_error = false;
        s_null_frame_ready = false;
        esp_wifi_connect();
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        const wifi_event_sta_connected_t *event =
            (const wifi_event_sta_connected_t *)event_data;

        if (event) {
            memcpy(s_ap_bssid, event->bssid, sizeof(s_ap_bssid));
            memcpy(s_null_frame + 4, event->bssid, sizeof(s_ap_bssid));
            memcpy(s_null_frame + 16, event->bssid, sizeof(s_ap_bssid));
            s_ap_bssid_valid = true;
            s_null_frame_ready = true;
        }
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *event =
            (const wifi_event_sta_disconnected_t *)event_data;

        s_wifi_is_connected = false;
        s_wifi_is_error = (s_retry_cnt >= WIFI_CONNECT_RETRIES);
        s_null_frame_ready = false;
        s_ap_bssid_valid = false;
        strlcpy(s_remote_addr, "0.0.0.0", sizeof(s_remote_addr));
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

        if (event) {
            ESP_LOGW(TAG, "STA disconnected, reason=%d", event->reason);
        }

        if (s_retry_cnt < WIFI_CONNECT_RETRIES) {
            xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
            s_retry_cnt++;
            esp_wifi_connect();
            ESP_LOGW(TAG, "Reconnecting (%d/%d)...", s_retry_cnt,
                     WIFI_CONNECT_RETRIES);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGW(TAG, "Connection failed %d times. Waiting 30s before next attempt...",
                     WIFI_CONNECT_RETRIES);
            if (s_reconnect_timer) {
                xTimerReset(s_reconnect_timer, 0);
            }
        }

        if (s_mutex) {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            reset_processing_state_locked();
            xSemaphoreGive(s_mutex);
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

        snprintf(s_remote_addr, sizeof(s_remote_addr), IPSTR,
                 IP2STR(&event->ip_info.ip));
        esp_wifi_set_ps(WIFI_PS_NONE);

        ESP_LOGW(TAG, "====================================");
        ESP_LOGW(TAG, "  DEVICE IS ONLINE: http://%s/", s_remote_addr);
        ESP_LOGW(TAG, "====================================");

        s_retry_cnt = 0;
        s_wifi_is_connected = true;
        s_wifi_is_error = false;
        xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        if (s_reconnect_timer) {
            xTimerStop(s_reconnect_timer, 0);
        }
    }
}

static void led_status_task(void *arg)
{
    (void)arg;

    gpio_reset_pin(BOARD_LED_GPIO);
    gpio_set_direction(BOARD_LED_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        csi_presence_state_t state;

        if (!s_wifi_is_connected) {
            if (s_wifi_is_error) {
                gpio_set_level(BOARD_LED_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
                gpio_set_level(BOARD_LED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(100));
            } else {
                gpio_set_level(BOARD_LED_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(BOARD_LED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
            continue;
        }

        if (s_wifi_role == CSI_WIFI_ROLE_DEDICATED_AP) {
            gpio_set_level(BOARD_LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(s_ap_client_count > 0U ? 80 : 20));
            gpio_set_level(BOARD_LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(s_ap_client_count > 0U ? 420 : 1980));
            continue;
        }

        state = s_presence_state;
    if (!s_detection_enabled) {
        gpio_set_level(BOARD_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(2000));
    } else if (state == CSI_PRESENCE_MOTION) {
        gpio_set_level(BOARD_LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
        } else if (state == CSI_PRESENCE_MICROMOTION) {
            gpio_set_level(BOARD_LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_set_level(BOARD_LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(150));
            gpio_set_level(BOARD_LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_set_level(BOARD_LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(1750));
        } else {
            gpio_set_level(BOARD_LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_set_level(BOARD_LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(1950));
        }
    }
}

esp_err_t csi_engine_init(const csi_config_t *cfg)
{
    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifi_cfg = {0};
    wifi_csi_config_t csi_cfg = {
        .lltf_en = true,
        .htltf_en = true,
        .stbc_htltf2_en = true,
        .ltf_merge_en = true,
        .channel_filter_en = true,
        .manu_scale = false,
        .shift = 0,
    };
    uint8_t mac[6];
    EventBits_t bits = 0;
    size_t ssid_len;
    size_t password_len;

    if (!cfg || !cfg->ssid || cfg->ssid[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    ssid_len = strlen(cfg->ssid);
    password_len = cfg->password ? strlen(cfg->password) : 0U;
    if (ssid_len >= sizeof(((wifi_config_t *)0)->sta.ssid)) {
        ESP_LOGE(TAG, "SSID is too long for the ESP32 Wi-Fi stack.");
        return ESP_ERR_INVALID_ARG;
    }
    if (password_len >= sizeof(((wifi_config_t *)0)->sta.password)) {
        ESP_LOGE(TAG, "Password is too long for the ESP32 Wi-Fi stack.");
        return ESP_ERR_INVALID_ARG;
    }
    if ((cfg->wifi_role == CSI_WIFI_ROLE_DEDICATED_AP) &&
        (password_len > 0U) && (password_len < 8U)) {
        ESP_LOGE(TAG, "Dedicated AP password must be at least 8 characters or empty.");
        return ESP_ERR_INVALID_ARG;
    }

    s_motion_threshold = clamp_motion_threshold(cfg->motion_threshold);
    s_detection_profile =
        sanitize_detection_profile(profile_from_motion_threshold(s_motion_threshold));
    s_detection_profile_custom = false;
    s_confirm_motion = clamp_confirm_frames(cfg->motion_confirm_frames, 2);
    s_confirm_clear = clamp_confirm_frames(cfg->clear_confirm_frames, 8);
    s_raw_logging_enabled = cfg->raw_logging_enabled;
    s_detection_enabled = false;
    s_wifi_role = cfg->wifi_role;
    s_ap_channel = clamp_ap_channel(cfg->ap_channel);
    s_ap_broadcast_interval_ms =
        clamp_ap_broadcast_interval(cfg->ap_broadcast_interval_ms);
    s_ap_client_count = 0;
    s_wifi_is_connected = false;
    s_wifi_is_error = false;
    s_null_frame_ready = false;
    s_wifi_netif = NULL;

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) {
        return ESP_ERR_NO_MEM;
    }

    if (s_wifi_role == CSI_WIFI_ROLE_SENSOR_STA) {
        s_csi_queue = xQueueCreate(CSI_QUEUE_LEN, sizeof(csi_frame_t));
        if (!s_csi_queue) {
            return ESP_ERR_NO_MEM;
        }
    } else {
        s_csi_queue = NULL;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    reset_processing_state_locked();
    configure_notch_locked(0.0f);
    xSemaphoreGive(s_mutex);

    s_wifi_event_group = xEventGroupCreate();
    if (!s_wifi_event_group) {
        return ESP_ERR_NO_MEM;
    }

    if (s_wifi_role == CSI_WIFI_ROLE_SENSOR_STA) {
        s_reconnect_timer = xTimerCreate("wifi_reconnect", pdMS_TO_TICKS(30000),
                                         pdFALSE, NULL, reconnect_timer_cb);
        s_inject_timer = xTimerCreate("csi_inject", pdMS_TO_TICKS(INJECT_PERIOD_MS),
                                      pdTRUE, NULL, inject_timer_cb);
        if (!s_reconnect_timer || !s_inject_timer) {
            return ESP_ERR_NO_MEM;
        }
    } else {
        s_reconnect_timer = NULL;
        s_inject_timer = NULL;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_wifi_netif = (s_wifi_role == CSI_WIFI_ROLE_SENSOR_STA) ?
        esp_netif_create_default_wifi_sta() :
        esp_netif_create_default_wifi_ap();
    if (!s_wifi_netif) {
        ESP_LOGE(TAG, "Failed to create default Wi-Fi netif for role %s.",
                 csi_wifi_role_to_string(s_wifi_role));
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    if (s_wifi_role == CSI_WIFI_ROLE_SENSOR_STA) {
        ESP_ERROR_CHECK(esp_event_handler_register(
            IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

        strlcpy((char *)wifi_cfg.sta.ssid, cfg->ssid, sizeof(wifi_cfg.sta.ssid));
        strlcpy((char *)wifi_cfg.sta.password,
                cfg->password ? cfg->password : "",
                sizeof(wifi_cfg.sta.password));
        wifi_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
        wifi_cfg.sta.pmf_cfg.capable = true;
        wifi_cfg.sta.pmf_cfg.required = false;

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
        ESP_ERROR_CHECK(esp_wifi_start());

        ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
        memcpy(s_null_frame + 10, mac, sizeof(mac));

        ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_cfg));
        ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(csi_rx_callback, NULL));
        ESP_ERROR_CHECK(esp_wifi_set_csi(true));
        ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

        bits = xEventGroupWaitBits(s_wifi_event_group,
                                   WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                   pdFALSE, pdFALSE,
                                   pdMS_TO_TICKS(15000));
        if (!(bits & WIFI_CONNECTED_BIT)) {
            ESP_LOGE(TAG, "Failed to connect to AP '%s'.", cfg->ssid);
        }

        if (xTaskCreate(csi_worker_task, "csi_worker", 6144, NULL, 5,
                        &s_csi_worker_task) != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
        xTimerStart(s_inject_timer, 0);
    } else {
        strlcpy((char *)wifi_cfg.ap.ssid, cfg->ssid, sizeof(wifi_cfg.ap.ssid));
        strlcpy((char *)wifi_cfg.ap.password,
                cfg->password ? cfg->password : "",
                sizeof(wifi_cfg.ap.password));
        wifi_cfg.ap.ssid_len = (uint8_t)ssid_len;
        wifi_cfg.ap.channel = s_ap_channel;
        wifi_cfg.ap.max_connection = CSI_AP_MAX_CLIENTS;
        wifi_cfg.ap.authmode = password_len > 0U ?
            WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
        wifi_cfg.ap.pmf_cfg.capable = true;
        wifi_cfg.ap.pmf_cfg.required = false;

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));
        ESP_ERROR_CHECK(esp_wifi_start());
        refresh_netif_ip_locked();

        if (xTaskCreate(ap_traffic_task, "ap_traffic", 4096, NULL, 4,
                        &s_ap_traffic_task) != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
    }
    xTaskCreate(led_status_task, "led_status", 3072, NULL, 4, NULL);

    ESP_LOGI(TAG,
             "Wi-Fi role=%s started. motion_threshold=%.2f motion_frames=%u "
             "clear_frames=%u raw_logging=%s ap_channel=%u ap_broadcast_ms=%u",
             csi_wifi_role_to_string(s_wifi_role),
             s_motion_threshold,
             s_confirm_motion,
             s_confirm_clear,
             s_raw_logging_enabled ? "on" : "off",
             s_ap_channel,
             (unsigned)s_ap_broadcast_interval_ms);
    return ESP_OK;
}

void csi_engine_get_status(csi_status_t *out)
{
    if (!out || !s_mutex) {
        return;
    }

    memset(out, 0, sizeof(*out));
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    out->presence_state = s_presence_state;
    out->occupancy_score = s_occupancy_score;
    out->micromotion_score = s_micromotion_score;
    out->motion_score = s_motion_score;
    out->stability_score = s_stability_score;
    out->packet_rate_hz = s_packet_rate_hz;
    out->packet_rate_jitter_hz = s_packet_rate_jitter_hz;
    out->rssi_avg = s_rssi_avg;
    out->occupancy_energy = s_last_occupancy_energy;
    out->motion_avg_energy = s_last_motion_avg_energy;
    out->motion_energy = s_last_motion_energy;
    out->occupancy_ref = s_last_occupancy_ref;
    out->motion_ref = s_last_motion_ref;
    out->motion_enter_threshold = s_last_motion_enter;
    out->motion_clear_threshold = s_last_motion_clear;
    out->micro_enter_threshold = s_last_micro_enter;
    out->micro_clear_threshold = s_last_micro_clear;
    out->presence_enter_threshold = s_last_presence_enter;
    out->presence_clear_threshold = s_last_presence_clear;
    out->selected_presence_state = s_selected_presence_state;
    out->hybrid_presence_state = s_hybrid_presence_state;
    out->selected_shift_score = s_selected_shift_score;
    out->selected_macro_score = s_selected_macro_score;
    out->selected_micro_score = s_selected_micro_score;
    out->selected_shift_raw = s_selected_shift_raw;
    out->selected_macro_raw = s_selected_macro_raw;
    out->selected_micro_raw = s_selected_micro_raw;
    out->selected_shift_ref = s_selected_shift_ref;
    out->selected_macro_ref = s_selected_macro_ref;
    out->selected_micro_ref = s_selected_micro_ref;
    out->shape_shift_score = s_shape_shift_score;
    out->rhythm_motion_score = s_rhythm_motion_score;
    out->patch_activity_score = s_patch_activity_score;
    out->shape_shift_raw = s_shape_shift_raw;
    out->rhythm_motion_raw = s_rhythm_motion_raw;
    out->patch_activity_raw = s_patch_activity_raw;
    out->shape_shift_ref = s_shape_shift_ref;
    out->rhythm_motion_ref = s_rhythm_motion_ref;
    out->patch_activity_ref = s_patch_activity_ref;
    out->micro_activity = s_micro_activity;
    out->micro_activity_frames = s_micro_activity_frames;
    out->selected_subcarrier_count = s_selected_present_count;
    out->hybrid_motion_frames = s_hybrid_motion_frames;
    out->hybrid_micro_frames = s_hybrid_micro_frames;
    out->hybrid_static_frames = s_hybrid_static_frames;
    out->hybrid_clear_frames = s_hybrid_clear_frames;
    out->hybrid_motion_hold_frames = s_hybrid_motion_hold_frames;
    out->hybrid_static_hold_frames = s_hybrid_static_hold_frames;
    out->num_subcarriers = s_num_sc;
    out->valid_subcarriers = s_valid_sc;
    out->state_event_count = s_state_event_count;
    out->disturbance_event_count = s_disturbance_event_count;
    out->processed_frame_count = s_processed_frames;
    out->queue_drop_count = s_csi_queue_drops;
    out->macro_frames = s_macro_frames;
    out->micro_frames = s_micro_frames;
    out->static_frames = s_static_frames;
    out->clear_frames = s_clear_frames;
    out->presence_hold_frames = s_presence_hold_frames;
    out->detection_enabled = s_detection_enabled;
    out->motion_threshold = s_motion_threshold;
    out->notch_frequency_hz = s_notch_freq_hz;
    out->wifi_connected = s_wifi_is_connected;
    out->baseline_ready = s_baseline_ready;
    out->raw_logging_enabled = s_raw_logging_enabled;
    out->force_calibration_enabled = s_force_calibration_enabled;
    out->calibration_guard_blocked = s_calibration_guard_blocked;
    out->wifi_role = s_wifi_role;
    out->ap_client_count = s_ap_client_count;
    out->ap_broadcast_interval_ms = s_ap_broadcast_interval_ms;
    out->calibration_percent = s_baseline_ready ? 100U :
        (uint8_t)(100U - ((s_calibration_frames_remaining * 100U) /
                          CSI_BASELINE_CALIBRATION_FRAMES));
    out->calibration_guard_hits = s_calibration_guard_hits;
    out->last_rssi = s_last_rssi;
    out->min_rssi = (s_min_rssi == 127) ? -127 : s_min_rssi;
    out->max_rssi = (s_max_rssi == -127) ? -127 : s_max_rssi;
    out->last_timestamp_us = s_last_timestamp_us;
    fill_setup_quality_locked(out);
    out->guided_stage = s_guided_stage;
    out->guided_active = s_guided_active;
    out->guided_stage_samples = (s_guided_stage <= CSI_SETUP_COMPLETE) ?
        s_guided_metrics[s_guided_stage].samples : 0U;
    out->guided_empty_motion_mean = setup_metric_mean(
        s_guided_metrics[CSI_SETUP_EMPTY_ROOM].motion_sum,
        s_guided_metrics[CSI_SETUP_EMPTY_ROOM].samples);
    out->guided_walk_motion_peak =
        s_guided_metrics[CSI_SETUP_WALK_TEST].motion_peak;
    out->guided_walk_motion_mean = setup_metric_mean(
        s_guided_metrics[CSI_SETUP_WALK_TEST].motion_sum,
        s_guided_metrics[CSI_SETUP_WALK_TEST].samples);
    out->guided_still_occupancy_mean = setup_metric_mean(
        s_guided_metrics[CSI_SETUP_STILL_TEST].occupancy_sum,
        s_guided_metrics[CSI_SETUP_STILL_TEST].samples);
    out->guided_still_motion_mean = setup_metric_mean(
        s_guided_metrics[CSI_SETUP_STILL_TEST].motion_sum,
        s_guided_metrics[CSI_SETUP_STILL_TEST].samples);
    out->guided_clear_occupancy_mean = setup_metric_mean(
        s_guided_metrics[CSI_SETUP_CLEAR_TEST].occupancy_sum,
        s_guided_metrics[CSI_SETUP_CLEAR_TEST].samples);
    out->guided_clear_motion_mean = setup_metric_mean(
        s_guided_metrics[CSI_SETUP_CLEAR_TEST].motion_sum,
        s_guided_metrics[CSI_SETUP_CLEAR_TEST].samples);
    out->guided_suggested_motion_threshold =
        s_guided_suggested_motion_threshold;
    out->guided_suggested_presence_enter =
        s_guided_suggested_presence_enter;
    out->guided_profile_applied = s_guided_profile_applied;
    out->guided_manual_sampling = s_guided_manual_sampling;
    out->guided_sampling = s_guided_sampling;
    out->guided_settle_frames_remaining = s_guided_settle_frames_remaining;
    out->guided_motion_contrast = s_guided_motion_contrast;
    out->guided_static_contrast = s_guided_static_contrast;
    strlcpy(out->guided_profile_mode, s_guided_profile_mode,
            sizeof(out->guided_profile_mode));
    strlcpy(out->guided_result, s_guided_result, sizeof(out->guided_result));
    strlcpy(out->manual_annotation, s_manual_annotation,
            sizeof(out->manual_annotation));
    memcpy(out->amplitudes, s_latest_amps, sizeof(s_latest_amps));
    strlcpy(out->remote_addr, s_remote_addr, sizeof(out->remote_addr));
    xSemaphoreGive(s_mutex);
}

void csi_engine_recalibrate(void)
{
    if (!s_mutex) {
        return;
    }
    if (!detection_supported()) {
        ESP_LOGI(TAG, "Recalibrate ignored in %s mode.",
                 csi_wifi_role_to_string(s_wifi_role));
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    reset_processing_state_locked();
    xSemaphoreGive(s_mutex);
    ESP_LOGI(TAG, "Recalibrating empty-room baseline.");
}

void csi_engine_set_motion_threshold(float threshold)
{
    float applied;

    if (!s_mutex) {
        return;
    }
    if (!detection_supported()) {
        ESP_LOGI(TAG, "Motion threshold ignored in %s mode.",
                 csi_wifi_role_to_string(s_wifi_role));
        return;
    }

    applied = clamp_motion_threshold(threshold);
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    set_profile_from_motion_threshold_locked(applied);
    xSemaphoreGive(s_mutex);

    if (!isfinite(threshold) || fabsf(applied - threshold) > 0.001f) {
        ESP_LOGW(TAG, "Motion threshold %.2f was clamped to %.2f.", threshold, applied);
    }
    ESP_LOGI(TAG, "Motion threshold updated to %.2f", applied);
}

void csi_engine_set_profile_threshold(const char *name, float threshold)
{
    csi_detection_profile_t profile;

    if (!s_mutex || !name) {
        return;
    }
    if (!detection_supported()) {
        ESP_LOGI(TAG, "Profile threshold ignored in %s mode.",
                 csi_wifi_role_to_string(s_wifi_role));
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    profile = s_detection_profile;
    if (strcmp(name, "motion_enter") == 0) {
        profile.motion_enter = threshold;
    } else if (strcmp(name, "motion_clear") == 0) {
        profile.motion_clear = threshold;
    } else if (strcmp(name, "micro_enter") == 0) {
        profile.micro_enter = threshold;
    } else if (strcmp(name, "micro_clear") == 0) {
        profile.micro_clear = threshold;
    } else if (strcmp(name, "presence_enter") == 0) {
        profile.presence_enter = threshold;
    } else if (strcmp(name, "presence_clear") == 0) {
        profile.presence_clear = threshold;
    } else {
        xSemaphoreGive(s_mutex);
        ESP_LOGW(TAG, "Unknown profile threshold '%s'.", name);
        return;
    }
    set_custom_detection_profile_locked(profile);
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Profile threshold %s updated to %.2f.", name, threshold);
}

void csi_engine_set_raw_logging(bool enabled)
{
    if (!s_mutex) {
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_raw_logging_enabled = enabled;
    if (enabled) {
        s_raw_log_header_emitted = false;
    }
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Raw CSI serial logging %s.", enabled ? "enabled" : "disabled");
}

void csi_engine_set_force_calibration(bool enabled)
{
    if (!s_mutex) {
        return;
    }
    if (!detection_supported()) {
        ESP_LOGI(TAG, "Force calibration ignored in %s mode.",
                 csi_wifi_role_to_string(s_wifi_role));
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_force_calibration_enabled = enabled;
    if (enabled) {
        s_calibration_guard_blocked = false;
    }
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Force calibration %s.", enabled ? "enabled" : "disabled");
}

void csi_engine_set_manual_annotation(const char *annotation)
{
    char sanitized[CSI_MANUAL_ANNOTATION_MAX];

    if (!s_mutex) {
        return;
    }

    sanitize_annotation(annotation, sanitized, sizeof(sanitized));
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    strlcpy(s_manual_annotation, sanitized, sizeof(s_manual_annotation));
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Manual annotation set to '%s'.", sanitized);
}

void csi_engine_set_detection_enabled(bool enabled)
{
    if (!s_mutex) {
        return;
    }
    if (!detection_supported()) {
        ESP_LOGI(TAG, "Detection control ignored in %s mode.",
                 csi_wifi_role_to_string(s_wifi_role));
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_detection_enabled = enabled;
    reset_processing_state_locked();
    reset_guided_setup_locked();
    xSemaphoreGive(s_mutex);

    if (enabled) {
        ESP_LOGI(TAG, "Detection enabled. Starting fresh baseline calibration.");
    } else {
        ESP_LOGI(TAG, "Detection paused.");
    }
}

void csi_engine_set_notch(float freq_hz)
{
    float applied;

    if (!s_mutex) {
        return;
    }
    if (!detection_supported()) {
        ESP_LOGI(TAG, "Notch filter ignored in %s mode.",
                 csi_wifi_role_to_string(s_wifi_role));
        return;
    }

    applied = clamp_notch_frequency(freq_hz);
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    configure_notch_locked(applied);
    xSemaphoreGive(s_mutex);

    if (!isfinite(freq_hz) || fabsf(applied - freq_hz) > 0.001f) {
        ESP_LOGW(TAG, "Notch request %.2f Hz was clamped to %.2f Hz.",
                 freq_hz, applied);
    }
    if (applied == 0.0f) {
        ESP_LOGI(TAG, "Notch filter disabled.");
    } else {
        ESP_LOGI(TAG, "Notch filter enabled at %.1f Hz.", applied);
    }
}

void csi_engine_guided_setup_start(void)
{
    if (!s_mutex) {
        return;
    }
    if (!detection_supported()) {
        ESP_LOGI(TAG, "Guided setup ignored in %s mode.",
                 csi_wifi_role_to_string(s_wifi_role));
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    reset_processing_state_locked();
    reset_guided_setup_locked();
    s_detection_enabled = true;
    s_guided_active = true;
    prepare_guided_stage_locked(CSI_SETUP_EMPTY_ROOM);
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Guided setup started: %s.",
             guided_stage_to_string(CSI_SETUP_EMPTY_ROOM));
}

void csi_engine_guided_setup_advance(void)
{
    csi_setup_stage_t next_stage;

    if (!s_mutex) {
        return;
    }
    if (!detection_supported()) {
        ESP_LOGI(TAG, "Guided setup advance ignored in %s mode.",
                 csi_wifi_role_to_string(s_wifi_role));
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (!s_guided_active && s_guided_stage != CSI_SETUP_COMPLETE) {
        s_guided_active = true;
        prepare_guided_stage_locked(CSI_SETUP_EMPTY_ROOM);
        next_stage = s_guided_stage;
    } else if ((s_guided_stage == CSI_SETUP_EMPTY_ROOM) && !s_baseline_ready) {
        next_stage = s_guided_stage;
    } else if (s_guided_stage >= CSI_SETUP_CLEAR_TEST) {
        s_guided_stage = CSI_SETUP_COMPLETE;
        s_guided_active = false;
        s_guided_sampling = false;
        s_guided_settle_frames_remaining = 0;
        update_guided_suggestions_locked();
        next_stage = s_guided_stage;
    } else if (s_guided_stage == CSI_SETUP_COMPLETE) {
        next_stage = s_guided_stage;
    } else {
        prepare_guided_stage_locked((csi_setup_stage_t)(s_guided_stage + 1));
        next_stage = s_guided_stage;
    }
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Guided setup stage: %s.",
             guided_stage_to_string(next_stage));
}

void csi_engine_guided_setup_cancel(void)
{
    if (!s_mutex) {
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    reset_guided_setup_locked();
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Guided setup canceled.");
}

void csi_engine_guided_setup_apply_profile(void)
{
    csi_detection_profile_t profile;

    if (!s_mutex) {
        return;
    }
    if (!detection_supported()) {
        ESP_LOGI(TAG, "Guided setup profile ignored in %s mode.",
                 csi_wifi_role_to_string(s_wifi_role));
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if ((s_guided_stage == CSI_SETUP_COMPLETE) &&
        (s_guided_suggested_motion_threshold >= CSI_MIN_MOTION_THRESHOLD) &&
        (s_guided_suggested_presence_enter > 0.0f) &&
        (strcmp(s_guided_profile_mode, "rejected") != 0)) {
        profile = profile_from_motion_threshold(s_guided_suggested_motion_threshold);
        profile.presence_enter = s_guided_suggested_presence_enter;
        profile.presence_clear = fmaxf(0.62f, profile.presence_enter * 0.86f);
        if (strcmp(s_guided_profile_mode, "motion_only") == 0) {
            profile.motion_clear = fminf(profile.motion_enter,
                fmaxf(profile.motion_clear, fmaxf(0.98f,
                                                  profile.motion_enter * 0.85f)));
        }
        set_custom_detection_profile_locked(profile);
        s_guided_profile_applied = true;
    }
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Guided setup profile applied.");
}

void csi_engine_guided_setup_set_manual_sampling(bool enabled)
{
    if (!s_mutex) {
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_guided_manual_sampling = enabled;
    if (s_guided_active &&
        (s_guided_stage != CSI_SETUP_IDLE) &&
        (s_guided_stage != CSI_SETUP_COMPLETE)) {
        s_guided_sampling = false;
        s_guided_settle_frames_remaining = enabled ? 0U : CSI_GUIDED_SETTLE_FRAMES;
        clear_guided_stage_metrics_locked(s_guided_stage);
    }
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Guided setup manual sampling %s.",
             enabled ? "enabled" : "disabled");
}

void csi_engine_guided_setup_start_sampling(void)
{
    if (!s_mutex) {
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_guided_active &&
        (s_guided_stage != CSI_SETUP_IDLE) &&
        (s_guided_stage != CSI_SETUP_COMPLETE) &&
        s_baseline_ready) {
        s_guided_sampling = true;
        s_guided_settle_frames_remaining = 0;
        clear_guided_stage_metrics_locked(s_guided_stage);
    }
    xSemaphoreGive(s_mutex);

    ESP_LOGI(TAG, "Guided setup sampling started.");
}
