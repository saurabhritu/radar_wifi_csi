#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Motion detection state machine states.
 */
typedef enum {
    CSI_STATE_CLEAR   = 0,
    CSI_STATE_MOTION  = 1,
} csi_motion_state_t;

typedef enum {
    CSI_ACT_STILL     = 0,
    CSI_ACT_BREATHING = 1,
    CSI_ACT_MOVING    = 2
} csi_activity_t;

typedef enum {
    CSI_DIR_NONE      = 0,
    CSI_DIR_TOWARD    = 1,
    CSI_DIR_AWAY      = 2
} csi_direction_t;

/**
 * @brief Snapshot of the latest CSI processing results.
 *
 * Populated by the CSI engine; read by the HTTP/WebSocket layer.
 */
typedef struct {
    csi_motion_state_t motion_state;    /**< Current detection state           */
    csi_activity_t     activity;        /**< Classified activity level         */
    csi_direction_t    direction;       /**< Detected movement direction       */
    float              variance_score;  /**< Latest computed variance score     */
    uint8_t            num_subcarriers; /**< Number of active subcarriers       */
    float              amplitudes[128]; /**< Per-subcarrier amplitudes (latest) */
    uint32_t           motion_event_count; /**< Total motion events since boot  */
    char               remote_addr[20]; /**< IP of the AP we are associated to  */
} csi_status_t;

/**
 * @brief Configuration passed to csi_engine_init().
 */
typedef struct {
    const char *ssid;           /**< AP SSID to connect to                     */
    const char *password;       /**< AP password                               */
    float       alert_threshold;/**< Variance score that triggers motion alert  */
    uint8_t     motion_confirm_frames; /**< # consecutive frames above threshold  */
    uint8_t     clear_confirm_frames;  /**< # consecutive frames below threshold  */
} csi_config_t;

/**
 * @brief Initialise the Wi-Fi stack and start CSI capture.
 *
 * Must be called after nvs_flash_init() and esp_netif_init().
 *
 * @param cfg   Pointer to configuration structure (copied internally).
 * @return ESP_OK on success.
 */
esp_err_t csi_engine_init(const csi_config_t *cfg);

/**
 * @brief Get a snapshot of the current CSI status (thread-safe copy).
 *
 * @param out  Destination structure to fill.
 */
void csi_engine_get_status(csi_status_t *out);

/**
 * @brief Reset the signal baseline (clears history ring buffer).
 *
 * Useful after the Recalibrate button is pressed via WebSocket.
 */
void csi_engine_recalibrate(void);

/**
 * @brief Update the alert threshold at runtime.
 *
 * @param threshold New variance threshold.
 */
void csi_engine_set_threshold(float threshold);
