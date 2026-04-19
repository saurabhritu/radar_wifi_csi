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

typedef enum {
    CSI_PRESENCE_NO_PRESENCE = 0,
    CSI_PRESENCE_STATIC      = 1,
    CSI_PRESENCE_MICROMOTION = 2,
    CSI_PRESENCE_MOTION      = 3,
} csi_presence_state_t;

typedef enum {
    CSI_WIFI_ROLE_SENSOR_STA = 0,
    CSI_WIFI_ROLE_DEDICATED_AP = 1,
} csi_wifi_role_t;

/**
 * @brief Snapshot of the latest CSI processing results.
 *
 * Populated by the CSI engine; read by the HTTP/WebSocket layer.
 */
typedef struct {
    csi_presence_state_t presence_state;   /**< 4-state radar output               */
    float                occupancy_score;  /**< Empty-room deviation score         */
    float                micromotion_score;/**< Low-energy temporal motion score   */
    float                motion_score;     /**< Macro-motion score                 */
    float                stability_score;  /**< High when presence is mostly static*/
    uint8_t              num_subcarriers;  /**< Number of active subcarriers       */
    uint8_t              valid_subcarriers;/**< Stable subcarriers used in scoring */
    float                amplitudes[CSI_MAX_SUBCARRIERS]; /**< Latest amplitudes   */
    uint32_t             state_event_count; /**< State transitions into presence   */
    uint32_t             disturbance_event_count; /**< Low-confidence disturbances */
    uint32_t             processed_frame_count; /**< CSI frames accepted by worker  */
    float                motion_threshold; /**< Runtime macro-motion threshold     */
    float                notch_frequency_hz; /**< Active notch frequency (0=off)   */
    float                packet_rate_hz;   /**< Effective processed packet rate    */
    float                occupancy_energy; /**< Raw empty-room deviation energy    */
    float                motion_avg_energy;/**< Raw short-window motion energy     */
    float                motion_energy;    /**< Raw peak-weighted motion energy    */
    float                occupancy_ref;    /**< Normalization reference for occ    */
    float                motion_ref;       /**< Normalization reference for motion */
    float                motion_enter_threshold; /**< Motion state entry threshold */
    float                motion_clear_threshold; /**< Motion state clear threshold */
    float                micro_enter_threshold;  /**< Micromotion entry threshold */
    float                micro_clear_threshold;  /**< Micromotion clear threshold */
    float                presence_enter_threshold; /**< Presence entry threshold   */
    float                presence_clear_threshold; /**< Presence clear threshold   */
    uint8_t              macro_frames;     /**< Consecutive macro-motion frames    */
    uint8_t              micro_frames;     /**< Consecutive micromotion frames     */
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
    uint64_t             last_timestamp_us;/**< Device timestamp for latest packet */
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

/**
 * @brief Human-readable label for a presence state.
 */
const char *csi_presence_state_to_string(csi_presence_state_t state);
const char *csi_wifi_role_to_string(csi_wifi_role_t role);
