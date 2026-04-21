#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define CSI_RING_DEPTH             100U
#define CSI_MAX_SUBCARRIERS        128U
#define CSI_WS_MAX_SUBCARRIERS      64U
#define CSI_MANUAL_ANNOTATION_MAX   24U
#define CSI_SAMPLE_RATE_HZ          50.0f
#define CSI_WARMUP_FRAMES           10U
#define CSI_MAX_CONFIRM_FRAMES      CSI_RING_DEPTH
#define CSI_MIN_MOTION_THRESHOLD     1.0f
#define CSI_MAX_MOTION_THRESHOLD    10.0f
#define CSI_MAX_NOTCH_FREQ_HZ      (CSI_SAMPLE_RATE_HZ * 0.45f)
#define CSI_GUIDED_SETTLE_FRAMES   120U

typedef enum {
    CSI_PRESENCE_NO_PRESENCE = 0,
    CSI_PRESENCE_STATIC      = 1,
    CSI_PRESENCE_MOTION      = 2,
} csi_presence_state_t;

typedef enum {
    CSI_WIFI_ROLE_SENSOR_STA = 0,
    CSI_WIFI_ROLE_DEDICATED_AP = 1,
} csi_wifi_role_t;

typedef enum {
    CSI_SETUP_IDLE = 0,
    CSI_SETUP_EMPTY_ROOM = 1,
    CSI_SETUP_WALK_TEST = 2,
    CSI_SETUP_STILL_TEST = 3,
    CSI_SETUP_CLEAR_TEST = 4,
    CSI_SETUP_COMPLETE = 5,
} csi_setup_stage_t;

typedef struct {
    float motion_enter;
    float motion_clear;
    float presence_enter;
    float presence_clear;
} csi_detection_profile_t;

/**
 * @brief Snapshot of the latest CSI processing results.
 *
 * Populated by the CSI engine; read by the HTTP/WebSocket layer.
 * Simplified for v1: 3-state detection using location-agnostic
 * relative features (frame-normalized, self-calibrating).
 */
typedef struct {
    csi_presence_state_t presence_state;   /**< 3-state radar output               */
    float                occupancy_score;  /**< Empty-room deviation score         */
    float                motion_score;     /**< Macro-motion score                 */
    float                stability_score;  /**< High when presence is mostly static*/
    uint8_t              num_subcarriers;  /**< Number of active subcarriers       */
    uint8_t              valid_subcarriers;/**< Stable subcarriers used in scoring */
    float                amplitudes[CSI_MAX_SUBCARRIERS]; /**< Latest amplitudes   */
    uint32_t             state_event_count; /**< State transitions into presence   */
    uint32_t             disturbance_event_count; /**< Low-confidence disturbances */
    uint32_t             processed_frame_count; /**< CSI frames accepted by worker  */
    uint32_t             queue_drop_count; /**< CSI callback frames dropped by queue */
    float                motion_threshold; /**< Runtime macro-motion threshold     */
    float                notch_frequency_hz; /**< Active notch frequency (0=off)   */
    float                packet_rate_hz;   /**< Effective processed packet rate    */
    float                packet_rate_jitter_hz; /**< EMA abs packet-rate deviation */
    float                rssi_avg;         /**< EMA RSSI for placement diagnostics */
    float                occupancy_energy; /**< Raw empty-room deviation energy    */
    float                motion_avg_energy;/**< Raw short-window motion energy     */
    float                motion_energy;    /**< Raw peak-weighted motion energy    */
    float                occupancy_ref;    /**< Normalization reference for occ    */
    float                motion_ref;       /**< Normalization reference for motion */
    float                motion_enter_threshold; /**< Motion state entry threshold */
    float                motion_clear_threshold; /**< Motion state clear threshold */
    float                presence_enter_threshold; /**< Presence entry threshold   */
    float                presence_clear_threshold; /**< Presence clear threshold   */
    uint8_t              macro_frames;     /**< Consecutive macro-motion frames    */
    uint8_t              static_frames;    /**< Consecutive static-presence frames */
    uint8_t              clear_frames;     /**< Consecutive clear frames           */
    uint8_t              presence_hold_frames; /**< Hold counter before clearing   */
    bool                 detection_enabled; /**< True when classifier is running  */
    bool                 wifi_connected;   /**< True once the station has an IP    */
    bool                 baseline_ready;   /**< Empty-room calibration complete    */
    bool                 raw_logging_enabled; /**< Serial CSI_DATA stream enabled  */
    bool                 force_calibration_enabled; /**< Ignore occupied-room guard  */
    bool                 calibration_guard_blocked; /**< Guard recently blocked cal  */
    csi_wifi_role_t      wifi_role;        /**< Sensor STA or dedicated AP role    */
    uint8_t              ap_client_count;  /**< Stations joined to dedicated AP    */
    uint16_t             ap_broadcast_interval_ms; /**< AP UDP cadence in ms      */
    uint8_t              calibration_percent; /**< 0..100 baseline calibration     */
    uint32_t             calibration_guard_hits; /**< Count of blocked attempts    */
    int8_t               last_rssi;        /**< RSSI for the latest processed pkt  */
    int8_t               min_rssi;         /**< Lowest RSSI seen since reset       */
    int8_t               max_rssi;         /**< Highest RSSI seen since reset      */
    uint64_t             last_timestamp_us;/**< Device timestamp for latest packet */
    bool                 deployment_ready; /**< True when setup quality is usable  */
    float                setup_quality_score; /**< 0..100 location/setup score    */
    char                 setup_quality[12];/**< poor/fair/good/excellent           */
    char                 setup_step[64];   /**< Guided setup next action           */
    char                 setup_reason[64]; /**< Primary readiness limiter          */
    csi_setup_stage_t    guided_stage;     /**< Current guided setup stage         */
    bool                 guided_active;    /**< True while guided setup is running */
    uint16_t             guided_stage_samples; /**< Samples in current stage       */
    float                guided_empty_motion_mean; /**< Empty-room motion score mean */
    float                guided_walk_motion_peak; /**< Walk-test peak motion score  */
    float                guided_walk_motion_mean; /**< Walk-test mean motion score  */
    float                guided_still_occupancy_mean; /**< Still occupancy mean     */
    float                guided_still_motion_mean; /**< Still motion mean           */
    float                guided_clear_occupancy_mean; /**< Clear occupancy mean     */
    float                guided_clear_motion_mean; /**< Clear motion mean          */
    float                guided_suggested_motion_threshold; /**< Suggested macro threshold */
    float                guided_suggested_presence_enter; /**< Suggested static threshold */
    bool                 guided_profile_applied; /**< True once suggestions applied */
    bool                 guided_sampling; /**< True while stage metrics collect   */
    uint16_t             guided_settle_frames_remaining; /**< Transition ignore frames */
    float                guided_motion_contrast; /**< Walk peak above empty motion */
    float                guided_static_contrast; /**< Still occupancy above clear */
    char                 guided_profile_mode[16]; /**< balanced/motion_only/rejected */
    char                 guided_result[16]; /**< incomplete/weak/fair/good/strong   */
    char                 manual_annotation[CSI_MANUAL_ANNOTATION_MAX]; /**< UI tag */
    char                 remote_addr[20];  /**< IPv4 assigned to the ESP32 station */
} csi_status_t;

/**
 * @brief Configuration passed to csi_engine_init().
 */
typedef struct {
    const char *ssid;               /**< AP SSID to connect to                 */
    const char *password;           /**< AP password                           */
    float       motion_threshold;   /**< Macro-motion detection threshold      */
    uint8_t     motion_confirm_frames; /**< Consecutive motion frames          */
    uint8_t     clear_confirm_frames;  /**< Consecutive empty-room frames      */
    bool        raw_logging_enabled;/**< Emit CSI_DATA lines over serial      */
    csi_wifi_role_t wifi_role;      /**< Sensor STA or dedicated AP mode      */
    uint8_t     ap_channel;         /**< SoftAP channel when in AP mode       */
    uint16_t    ap_broadcast_interval_ms; /**< AP UDP broadcast cadence in ms */
} csi_config_t;

/**
 * @brief Initialise the Wi-Fi stack and start CSI capture.
 *
 * Must be called after nvs_flash_init().
 *
 * @param cfg Pointer to configuration structure (copied internally).
 * @return ESP_OK on success.
 */
esp_err_t csi_engine_init(const csi_config_t *cfg);

/**
 * @brief Get a snapshot of the current CSI status (thread-safe copy).
 *
 * @param out Destination structure to fill.
 */
void csi_engine_get_status(csi_status_t *out);

/**
 * @brief Reset the empty-room baseline and restart calibration.
 */
void csi_engine_recalibrate(void);
void csi_engine_set_force_calibration(bool enabled);
void csi_engine_set_manual_annotation(const char *annotation);

/**
 * @brief Update the macro-motion threshold at runtime.
 *
 * @param threshold New motion threshold.
 */
void csi_engine_set_motion_threshold(float threshold);
void csi_engine_set_profile_threshold(const char *name, float threshold);

/**
 * @brief Enable or disable raw CSI serial logging.
 *
 * @param enabled True to emit `CSI_DATA,...` lines.
 */
void csi_engine_set_raw_logging(bool enabled);

/**
 * @brief Start or stop the presence detector.
 *
 * Starting detection resets the baseline and begins a fresh calibration.
 */
void csi_engine_set_detection_enabled(bool enabled);

/**
 * @brief Set the specific DSP Notch Filter center frequency.
 *
 * @param freq_hz The target frequency in Hz. Use 0.0 to disable the notch.
 */
void csi_engine_set_notch(float freq_hz);

void csi_engine_guided_setup_start(void);
void csi_engine_guided_setup_advance(void);
void csi_engine_guided_setup_cancel(void);
void csi_engine_guided_setup_apply_profile(void);
void csi_engine_guided_setup_start_sampling(void);

/**
 * @brief Human-readable label for a presence state.
 */
const char *csi_presence_state_to_string(csi_presence_state_t state);
const char *csi_wifi_role_to_string(csi_wifi_role_t role);
