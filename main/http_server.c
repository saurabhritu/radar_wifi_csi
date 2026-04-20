/**
 * @file http_server.c
 * @brief ESP-IDF HTTP server with split WebSocket lanes.
 *
 * Endpoints:
 *   GET /        → Serves the embedded dashboard HTML.
 *   GET /ws      → Dashboard control + status JSON.
 *   GET /ws/log  → Dedicated high-rate CSV log stream.
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "dashboard.h"
#include "http_server.h"
#include "wifi_csi.h"

static const char *TAG = "http_server";

static httpd_handle_t s_server = NULL;

#define MAX_STATUS_WS_CLIENTS        4
#define MAX_LOG_WS_CLIENTS           4
#define WS_MAX_CONTROL_PAYLOAD     256
#define BROWSER_CAPTURE_QUEUE_LEN    8
#define LOG_STREAM_QUEUE_LEN        16
#define CAPTURE_ROW_MAX           1408
#define BROWSER_CAPTURE_ROWS_PER_PUSH 2
#define LOG_STREAM_ROWS_PER_PUSH     8

static int s_status_ws_fds[MAX_STATUS_WS_CLIENTS];
static int s_status_ws_count = 0;
static int s_log_ws_fds[MAX_LOG_WS_CLIENTS];
static bool s_log_ws_header_sent[MAX_LOG_WS_CLIENTS];
static int s_log_ws_count = 0;
static SemaphoreHandle_t s_ws_mutex = NULL;
static TaskHandle_t s_status_ws_task = NULL;
static TaskHandle_t s_log_ws_task = NULL;
static volatile bool s_ws_running = false;
static volatile bool s_capture_enabled = false;
static QueueHandle_t s_browser_capture_queue = NULL;
static QueueHandle_t s_log_stream_queue = NULL;
static char s_browser_capture_chunk[(CAPTURE_ROW_MAX * BROWSER_CAPTURE_ROWS_PER_PUSH) + 1];
static char s_log_stream_chunk[(CAPTURE_ROW_MAX * LOG_STREAM_ROWS_PER_PUSH) + 1];

typedef struct {
    char payload[CAPTURE_ROW_MAX];
} capture_row_t;

static const char *CAPTURE_HEADER =
    "CSI_DATA_HEADER,"
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
    "state_event_count,disturbance_count,notch_hz,last_timestamp_us,src_mac,iq_vector";

static void ws_remove_client(int *fds, int *count, int max_clients, int fd)
{
    if (!s_ws_mutex) {
        return;
    }

    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    for (int i = 0; i < *count; i++) {
        if (fds[i] == fd) {
            fds[i] = fds[--(*count)];
            if (*count < max_clients) {
                fds[*count] = -1;
            }
            break;
        }
    }
    xSemaphoreGive(s_ws_mutex);
}

static void ws_add_client(int *fds, int *count, int max_clients, int fd)
{
    if (!s_ws_mutex) {
        return;
    }

    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    for (int i = 0; i < *count; i++) {
        if (fds[i] == fd) {
            xSemaphoreGive(s_ws_mutex);
            return;
        }
    }

    if (*count < max_clients) {
        fds[(*count)++] = fd;
    } else {
        ESP_LOGW(TAG, "Rejecting websocket fd=%d because the client limit is %d.",
                 fd, max_clients);
    }
    xSemaphoreGive(s_ws_mutex);
}

static void log_ws_remove_client(int fd)
{
    if (!s_ws_mutex) {
        return;
    }

    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    for (int i = 0; i < s_log_ws_count; i++) {
        if (s_log_ws_fds[i] == fd) {
            int last = s_log_ws_count - 1;
            s_log_ws_fds[i] = s_log_ws_fds[last];
            s_log_ws_header_sent[i] = s_log_ws_header_sent[last];
            s_log_ws_count = last;
            if (s_log_ws_count < MAX_LOG_WS_CLIENTS) {
                s_log_ws_fds[s_log_ws_count] = -1;
                s_log_ws_header_sent[s_log_ws_count] = false;
            }
            break;
        }
    }
    xSemaphoreGive(s_ws_mutex);
}

static void log_ws_add_client(int fd)
{
    if (!s_ws_mutex) {
        return;
    }

    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    for (int i = 0; i < s_log_ws_count; i++) {
        if (s_log_ws_fds[i] == fd) {
            s_log_ws_header_sent[i] = false;
            xSemaphoreGive(s_ws_mutex);
            return;
        }
    }

    if (s_log_ws_count < MAX_LOG_WS_CLIENTS) {
        s_log_ws_fds[s_log_ws_count] = fd;
        s_log_ws_header_sent[s_log_ws_count] = false;
        s_log_ws_count++;
    } else {
        ESP_LOGW(TAG, "Rejecting log websocket fd=%d because the client limit is %d.",
                 fd, MAX_LOG_WS_CLIENTS);
    }
    xSemaphoreGive(s_ws_mutex);
}

static int ws_client_count(const int *fds, int count)
{
    (void)fds;

    if (!s_ws_mutex) {
        return 0;
    }

    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    int snapshot_count = count;
    xSemaphoreGive(s_ws_mutex);

    return snapshot_count;
}

static int ws_snapshot_clients(const int *src_fds, int count, int *dst_fds,
                               size_t max_fds)
{
    int snapshot_count = 0;

    if (!s_ws_mutex || !dst_fds || max_fds == 0) {
        return 0;
    }

    xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
    snapshot_count = count;
    if ((size_t)snapshot_count > max_fds) {
        snapshot_count = (int)max_fds;
    }
    memcpy(dst_fds, src_fds, snapshot_count * sizeof(int));
    xSemaphoreGive(s_ws_mutex);

    return snapshot_count;
}

static void ws_broadcast_clients(const int *src_fds, int count,
                                 int *live_fds, int *live_count, int max_clients,
                                 const char *payload, size_t len)
{
    int snapshot[MAX_LOG_WS_CLIENTS > MAX_STATUS_WS_CLIENTS ?
                 MAX_LOG_WS_CLIENTS : MAX_STATUS_WS_CLIENTS];
    int client_count;

    if (!s_server || !payload || len == 0) {
        return;
    }

    client_count = ws_snapshot_clients(src_fds, count, snapshot,
                                       sizeof(snapshot) / sizeof(snapshot[0]));
    for (int i = 0; i < client_count; i++) {
        httpd_ws_frame_t pkt = {
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)payload,
            .len = len,
        };
        esp_err_t ret = httpd_ws_send_frame_async(s_server, snapshot[i], &pkt);
        if (ret != ESP_OK) {
            ESP_LOGD(TAG, "WS client %d gone, removing.", snapshot[i]);
            if (live_fds == s_log_ws_fds) {
                log_ws_remove_client(snapshot[i]);
            } else {
                ws_remove_client(live_fds, live_count, max_clients, snapshot[i]);
            }
        }
    }
}

static void status_ws_broadcast(const char *payload, size_t len)
{
    ws_broadcast_clients(s_status_ws_fds, s_status_ws_count,
                         s_status_ws_fds, &s_status_ws_count,
                         MAX_STATUS_WS_CLIENTS, payload, len);
}

static void log_ws_broadcast(const char *payload, size_t len)
{
    ws_broadcast_clients(s_log_ws_fds, s_log_ws_count,
                         s_log_ws_fds, &s_log_ws_count,
                         MAX_LOG_WS_CLIENTS, payload, len);
}

static void queue_reset(QueueHandle_t queue)
{
    capture_row_t row;

    if (!queue) {
        return;
    }

    while (xQueueReceive(queue, &row, 0) == pdTRUE) {
    }
}

static esp_err_t queue_ensure_created(QueueHandle_t *queue, UBaseType_t depth)
{
    if (*queue) {
        return ESP_OK;
    }

    *queue = xQueueCreate(depth, sizeof(capture_row_t));
    if (!*queue) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

static void status_ws_push_task(void *arg)
{
    (void)arg;

    while (s_ws_running) {
        capture_row_t capture_row;

        vTaskDelay(pdMS_TO_TICKS(100));

        if (!s_server || ws_client_count(s_status_ws_fds, s_status_ws_count) == 0) {
            continue;
        }

        csi_status_t st;
        csi_engine_get_status(&st);

        cJSON *root = cJSON_CreateObject();
        if (!root) {
            continue;
        }

        cJSON_AddStringToObject(root, "presence",
                                csi_presence_state_to_string(st.presence_state));
        cJSON_AddNumberToObject(root, "occupied",
                                st.presence_state != CSI_PRESENCE_NO_PRESENCE ? 1 : 0);
        cJSON_AddNumberToObject(root, "occupancy_score",
                                (double)st.occupancy_score);
        cJSON_AddNumberToObject(root, "micromotion_score",
                                (double)st.micromotion_score);
        cJSON_AddNumberToObject(root, "motion_score",
                                (double)st.motion_score);
        cJSON_AddNumberToObject(root, "stability_score",
                                (double)st.stability_score);
        cJSON_AddNumberToObject(root, "packet_rate_hz",
                                (double)st.packet_rate_hz);
        cJSON_AddNumberToObject(root, "packet_rate_jitter_hz",
                                (double)st.packet_rate_jitter_hz);
        cJSON_AddNumberToObject(root, "rssi_avg",
                                (double)st.rssi_avg);
        cJSON_AddNumberToObject(root, "occupancy_energy",
                                (double)st.occupancy_energy);
        cJSON_AddNumberToObject(root, "motion_avg_energy",
                                (double)st.motion_avg_energy);
        cJSON_AddNumberToObject(root, "motion_energy",
                                (double)st.motion_energy);
        cJSON_AddNumberToObject(root, "occupancy_ref",
                                (double)st.occupancy_ref);
        cJSON_AddNumberToObject(root, "motion_ref",
                                (double)st.motion_ref);
        cJSON_AddNumberToObject(root, "motion_enter_threshold",
                                (double)st.motion_enter_threshold);
        cJSON_AddNumberToObject(root, "motion_clear_threshold",
                                (double)st.motion_clear_threshold);
        cJSON_AddNumberToObject(root, "micro_enter_threshold",
                                (double)st.micro_enter_threshold);
        cJSON_AddNumberToObject(root, "micro_clear_threshold",
                                (double)st.micro_clear_threshold);
        cJSON_AddNumberToObject(root, "presence_enter_threshold",
                                (double)st.presence_enter_threshold);
        cJSON_AddNumberToObject(root, "presence_clear_threshold",
                                (double)st.presence_clear_threshold);
        cJSON_AddStringToObject(root, "selected_presence",
                                csi_presence_state_to_string(st.selected_presence_state));
        cJSON_AddStringToObject(root, "hybrid_presence",
                                csi_presence_state_to_string(st.hybrid_presence_state));
        cJSON_AddNumberToObject(root, "selected_shift_score",
                                (double)st.selected_shift_score);
        cJSON_AddNumberToObject(root, "selected_macro_score",
                                (double)st.selected_macro_score);
        cJSON_AddNumberToObject(root, "selected_micro_score",
                                (double)st.selected_micro_score);
        cJSON_AddNumberToObject(root, "selected_shift_raw",
                                (double)st.selected_shift_raw);
        cJSON_AddNumberToObject(root, "selected_macro_raw",
                                (double)st.selected_macro_raw);
        cJSON_AddNumberToObject(root, "selected_micro_raw",
                                (double)st.selected_micro_raw);
        cJSON_AddNumberToObject(root, "selected_shift_ref",
                                (double)st.selected_shift_ref);
        cJSON_AddNumberToObject(root, "selected_macro_ref",
                                (double)st.selected_macro_ref);
        cJSON_AddNumberToObject(root, "selected_micro_ref",
                                (double)st.selected_micro_ref);
        cJSON_AddNumberToObject(root, "shape_shift_score",
                                (double)st.shape_shift_score);
        cJSON_AddNumberToObject(root, "rhythm_motion_score",
                                (double)st.rhythm_motion_score);
        cJSON_AddNumberToObject(root, "patch_activity_score",
                                (double)st.patch_activity_score);
        cJSON_AddNumberToObject(root, "shape_shift_raw",
                                (double)st.shape_shift_raw);
        cJSON_AddNumberToObject(root, "rhythm_motion_raw",
                                (double)st.rhythm_motion_raw);
        cJSON_AddNumberToObject(root, "patch_activity_raw",
                                (double)st.patch_activity_raw);
        cJSON_AddNumberToObject(root, "shape_shift_ref",
                                (double)st.shape_shift_ref);
        cJSON_AddNumberToObject(root, "rhythm_motion_ref",
                                (double)st.rhythm_motion_ref);
        cJSON_AddNumberToObject(root, "patch_activity_ref",
                                (double)st.patch_activity_ref);
        cJSON_AddNumberToObject(root, "micro_activity",
                                st.micro_activity ? 1 : 0);
        cJSON_AddNumberToObject(root, "micro_activity_frames",
                                st.micro_activity_frames);
        cJSON_AddNumberToObject(root, "selected_subcarrier_count",
                                st.selected_subcarrier_count);
        cJSON_AddNumberToObject(root, "hybrid_motion_frames",
                                st.hybrid_motion_frames);
        cJSON_AddNumberToObject(root, "hybrid_micro_frames",
                                st.hybrid_micro_frames);
        cJSON_AddNumberToObject(root, "hybrid_static_frames",
                                st.hybrid_static_frames);
        cJSON_AddNumberToObject(root, "hybrid_clear_frames",
                                st.hybrid_clear_frames);
        cJSON_AddNumberToObject(root, "hybrid_motion_hold_frames",
                                st.hybrid_motion_hold_frames);
        cJSON_AddNumberToObject(root, "hybrid_static_hold_frames",
                                st.hybrid_static_hold_frames);
        cJSON_AddNumberToObject(root, "subcarriers", st.num_subcarriers);
        cJSON_AddNumberToObject(root, "valid_subcarriers",
                                st.valid_subcarriers);
        cJSON_AddNumberToObject(root, "event_count", st.state_event_count);
        cJSON_AddNumberToObject(root, "disturbance_count",
                                st.disturbance_event_count);
        cJSON_AddNumberToObject(root, "processed_frame_count",
                                st.processed_frame_count);
        cJSON_AddNumberToObject(root, "queue_drop_count",
                                st.queue_drop_count);
        cJSON_AddNumberToObject(root, "macro_frames", st.macro_frames);
        cJSON_AddNumberToObject(root, "micro_frames", st.micro_frames);
        cJSON_AddNumberToObject(root, "static_frames", st.static_frames);
        cJSON_AddNumberToObject(root, "clear_frames", st.clear_frames);
        cJSON_AddNumberToObject(root, "presence_hold_frames",
                                st.presence_hold_frames);
        cJSON_AddNumberToObject(root, "motion_threshold",
                                (double)st.motion_threshold);
        cJSON_AddNumberToObject(root, "notch_hz",
                                (double)st.notch_frequency_hz);
        cJSON_AddNumberToObject(root, "wifi_connected",
                                st.wifi_connected ? 1 : 0);
        cJSON_AddStringToObject(root, "wifi_role",
                                csi_wifi_role_to_string(st.wifi_role));
        cJSON_AddNumberToObject(root, "ap_client_count",
                                st.ap_client_count);
        cJSON_AddNumberToObject(root, "ap_broadcast_interval_ms",
                                st.ap_broadcast_interval_ms);
        cJSON_AddNumberToObject(root, "detection_enabled",
                                st.detection_enabled ? 1 : 0);
        cJSON_AddNumberToObject(root, "baseline_ready",
                                st.baseline_ready ? 1 : 0);
        cJSON_AddNumberToObject(root, "calibration_percent",
                                st.calibration_percent);
        cJSON_AddNumberToObject(root, "raw_logging_enabled",
                                st.raw_logging_enabled ? 1 : 0);
        cJSON_AddNumberToObject(root, "force_calibration_enabled",
                                st.force_calibration_enabled ? 1 : 0);
        cJSON_AddNumberToObject(root, "calibration_guard_blocked",
                                st.calibration_guard_blocked ? 1 : 0);
        cJSON_AddNumberToObject(root, "calibration_guard_hits",
                                (double)st.calibration_guard_hits);
        cJSON_AddStringToObject(root, "manual_annotation",
                                st.manual_annotation);
        cJSON_AddNumberToObject(root, "capture_enabled",
                                s_capture_enabled ? 1 : 0);
        cJSON_AddNumberToObject(root, "log_stream_clients",
                                ws_client_count(s_log_ws_fds, s_log_ws_count));
        cJSON_AddNumberToObject(root, "last_rssi", st.last_rssi);
        cJSON_AddNumberToObject(root, "min_rssi", st.min_rssi);
        cJSON_AddNumberToObject(root, "max_rssi", st.max_rssi);
        cJSON_AddNumberToObject(root, "last_timestamp_us",
                                (double)st.last_timestamp_us);
        cJSON_AddNumberToObject(root, "deployment_ready",
                                st.deployment_ready ? 1 : 0);
        cJSON_AddNumberToObject(root, "setup_quality_score",
                                (double)st.setup_quality_score);
        cJSON_AddStringToObject(root, "setup_quality", st.setup_quality);
        cJSON_AddStringToObject(root, "setup_step", st.setup_step);
        cJSON_AddStringToObject(root, "setup_reason", st.setup_reason);
        cJSON_AddNumberToObject(root, "guided_stage", st.guided_stage);
        cJSON_AddNumberToObject(root, "guided_active",
                                st.guided_active ? 1 : 0);
        cJSON_AddNumberToObject(root, "guided_stage_samples",
                                st.guided_stage_samples);
        cJSON_AddNumberToObject(root, "guided_empty_motion_mean",
                                (double)st.guided_empty_motion_mean);
        cJSON_AddNumberToObject(root, "guided_walk_motion_peak",
                                (double)st.guided_walk_motion_peak);
        cJSON_AddNumberToObject(root, "guided_walk_motion_mean",
                                (double)st.guided_walk_motion_mean);
        cJSON_AddNumberToObject(root, "guided_still_occupancy_mean",
                                (double)st.guided_still_occupancy_mean);
        cJSON_AddNumberToObject(root, "guided_still_motion_mean",
                                (double)st.guided_still_motion_mean);
        cJSON_AddNumberToObject(root, "guided_clear_occupancy_mean",
                                (double)st.guided_clear_occupancy_mean);
        cJSON_AddNumberToObject(root, "guided_clear_motion_mean",
                                (double)st.guided_clear_motion_mean);
        cJSON_AddNumberToObject(root, "guided_suggested_motion_threshold",
                                (double)st.guided_suggested_motion_threshold);
        cJSON_AddNumberToObject(root, "guided_suggested_presence_enter",
                                (double)st.guided_suggested_presence_enter);
        cJSON_AddNumberToObject(root, "guided_profile_applied",
                                st.guided_profile_applied ? 1 : 0);
        cJSON_AddNumberToObject(root, "guided_manual_sampling",
                                st.guided_manual_sampling ? 1 : 0);
        cJSON_AddNumberToObject(root, "guided_sampling",
                                st.guided_sampling ? 1 : 0);
        cJSON_AddNumberToObject(root, "guided_settle_frames_remaining",
                                st.guided_settle_frames_remaining);
        cJSON_AddNumberToObject(root, "guided_motion_contrast",
                                (double)st.guided_motion_contrast);
        cJSON_AddNumberToObject(root, "guided_static_contrast",
                                (double)st.guided_static_contrast);
        cJSON_AddStringToObject(root, "guided_profile_mode",
                                st.guided_profile_mode);
        cJSON_AddStringToObject(root, "guided_result", st.guided_result);
        cJSON_AddStringToObject(root, "remote_addr", st.remote_addr);

        cJSON_AddStringToObject(root, "status",
                                st.presence_state == CSI_PRESENCE_MOTION ?
                                "MOTION_DETECTED" :
                                csi_presence_state_to_string(st.presence_state));
        cJSON_AddNumberToObject(root, "variance", (double)st.motion_score);
        cJSON_AddNumberToObject(root, "threshold", (double)st.motion_threshold);

        uint8_t sc = st.num_subcarriers;
        if (sc > CSI_WS_MAX_SUBCARRIERS) {
            sc = CSI_WS_MAX_SUBCARRIERS;
        }
        cJSON *amps = cJSON_CreateArray();
        if (!amps) {
            cJSON_Delete(root);
            continue;
        }
        for (uint8_t i = 0; i < sc; i++) {
            cJSON_AddItemToArray(amps,
                                 cJSON_CreateNumber((double)st.amplitudes[i]));
        }
        cJSON_AddItemToObject(root, "amplitudes", amps);

        char *str = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);

        if (str) {
            status_ws_broadcast(str, strlen(str));
            free(str);
        }

        if (s_capture_enabled && s_browser_capture_queue) {
            size_t chunk_len = 0;
            int rows_in_chunk = 0;

            memset(s_browser_capture_chunk, 0, sizeof(s_browser_capture_chunk));
            while ((rows_in_chunk < BROWSER_CAPTURE_ROWS_PER_PUSH) &&
                   (xQueueReceive(s_browser_capture_queue, &capture_row, 0) == pdTRUE)) {
                size_t row_len = strnlen(capture_row.payload,
                                         sizeof(capture_row.payload));
                if ((chunk_len + row_len + 1U) >= sizeof(s_browser_capture_chunk)) {
                    break;
                }
                memcpy(s_browser_capture_chunk + chunk_len, capture_row.payload, row_len);
                chunk_len += row_len;
                s_browser_capture_chunk[chunk_len++] = '\n';
                rows_in_chunk++;
            }

            if (rows_in_chunk > 0) {
                cJSON *msg = cJSON_CreateObject();
                char *msg_str;

                if (!msg) {
                    continue;
                }
                s_browser_capture_chunk[chunk_len] = '\0';
                cJSON_AddStringToObject(msg, "type", "capture_chunk");
                cJSON_AddStringToObject(msg, "csv", s_browser_capture_chunk);
                msg_str = cJSON_PrintUnformatted(msg);
                cJSON_Delete(msg);
                if (msg_str) {
                    status_ws_broadcast(msg_str, strlen(msg_str));
                    free(msg_str);
                }
            }
        }
    }

    s_status_ws_task = NULL;
    vTaskDelete(NULL);
}

static void log_ws_push_task(void *arg)
{
    (void)arg;

    while (s_ws_running) {
        capture_row_t log_row;
        size_t chunk_len = 0;
        int rows_in_chunk = 0;

        vTaskDelay(pdMS_TO_TICKS(20));

        if (!s_server || !s_log_stream_queue ||
            ws_client_count(s_log_ws_fds, s_log_ws_count) == 0) {
            continue;
        }

        if (s_ws_mutex) {
            xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
            for (int i = 0; i < s_log_ws_count; i++) {
                int fd = s_log_ws_fds[i];
                httpd_ws_client_info_t info = httpd_ws_get_fd_info(s_server, fd);

                if (info != HTTPD_WS_CLIENT_WEBSOCKET) {
                    continue;
                }

                if (!s_log_ws_header_sent[i]) {
                    httpd_ws_frame_t pkt = {
                        .type = HTTPD_WS_TYPE_TEXT,
                        .payload = (uint8_t *)CAPTURE_HEADER,
                        .len = strlen(CAPTURE_HEADER),
                    };
                    esp_err_t ret = httpd_ws_send_frame_async(s_server, fd, &pkt);
                    if (ret == ESP_OK) {
                        s_log_ws_header_sent[i] = true;
                    } else {
                        ESP_LOGD(TAG, "Failed to send log header to fd=%d, removing client.", fd);
                        xSemaphoreGive(s_ws_mutex);
                        log_ws_remove_client(fd);
                        xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
                        i = -1;
                    }
                }
            }
            xSemaphoreGive(s_ws_mutex);
        }

        memset(s_log_stream_chunk, 0, sizeof(s_log_stream_chunk));
        while ((rows_in_chunk < LOG_STREAM_ROWS_PER_PUSH) &&
               (xQueueReceive(s_log_stream_queue, &log_row, 0) == pdTRUE)) {
            size_t row_len = strnlen(log_row.payload, sizeof(log_row.payload));
            if ((chunk_len + row_len + 1U) >= sizeof(s_log_stream_chunk)) {
                break;
            }
            memcpy(s_log_stream_chunk + chunk_len, log_row.payload, row_len);
            chunk_len += row_len;
            s_log_stream_chunk[chunk_len++] = '\n';
            rows_in_chunk++;
        }

        if (rows_in_chunk == 0) {
            continue;
        }

        s_log_stream_chunk[chunk_len] = '\0';
        log_ws_broadcast(s_log_stream_chunk, chunk_len);
    }

    s_log_ws_task = NULL;
    vTaskDelete(NULL);
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, DASHBOARD_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "Dashboard websocket handshake from fd=%d", fd);
        ws_add_client(s_status_ws_fds, &s_status_ws_count,
                      MAX_STATUS_WS_CLIENTS, fd);
        return ESP_OK;
    }

    httpd_ws_frame_t pkt;
    memset(&pkt, 0, sizeof(httpd_ws_frame_t));
    pkt.type = HTTPD_WS_TYPE_TEXT;

    esp_err_t ret = httpd_ws_recv_frame(req, &pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }

    if (pkt.len == 0) {
        return ESP_OK;
    }

    if (pkt.len > WS_MAX_CONTROL_PAYLOAD) {
        ESP_LOGW(TAG, "Dropping oversized WS control payload (%u bytes).",
                 (unsigned)pkt.len);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t *buf = calloc(1, pkt.len + 1);
    if (!buf) {
        return ESP_ERR_NO_MEM;
    }

    pkt.payload = buf;
    ret = httpd_ws_recv_frame(req, &pkt, pkt.len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
        free(buf);
        return ret;
    }

    buf[pkt.len] = '\0';
    ESP_LOGD(TAG, "WS recv: %s", buf);

    if (pkt.type == HTTPD_WS_TYPE_TEXT) {
        cJSON *msg = cJSON_Parse((char *)buf);
        if (msg) {
            cJSON *cmd = cJSON_GetObjectItem(msg, "cmd");
            if (cJSON_IsString(cmd)) {
                if (strcmp(cmd->valuestring, "recalibrate") == 0) {
                    csi_engine_recalibrate();
                    ESP_LOGI(TAG, "Recalibration requested via WS");
                } else if (strcmp(cmd->valuestring, "guided_setup_start") == 0) {
                    csi_engine_guided_setup_start();
                } else if (strcmp(cmd->valuestring, "guided_setup_advance") == 0) {
                    csi_engine_guided_setup_advance();
                } else if (strcmp(cmd->valuestring, "guided_setup_cancel") == 0) {
                    csi_engine_guided_setup_cancel();
                } else if (strcmp(cmd->valuestring, "guided_setup_apply_profile") == 0) {
                    csi_engine_guided_setup_apply_profile();
                } else if (strcmp(cmd->valuestring, "guided_setup_start_sampling") == 0) {
                    csi_engine_guided_setup_start_sampling();
                } else if (strcmp(cmd->valuestring, "guided_setup_set_manual_sampling") == 0) {
                    cJSON *val = cJSON_GetObjectItem(msg, "value");
                    if (cJSON_IsBool(val)) {
                        csi_engine_guided_setup_set_manual_sampling(cJSON_IsTrue(val));
                    } else if (cJSON_IsNumber(val)) {
                        csi_engine_guided_setup_set_manual_sampling(
                            val->valuedouble != 0.0);
                    }
                } else if (strcmp(cmd->valuestring, "set_force_calibration") == 0) {
                    cJSON *val = cJSON_GetObjectItem(msg, "value");
                    if (cJSON_IsBool(val)) {
                        csi_engine_set_force_calibration(cJSON_IsTrue(val));
                    } else if (cJSON_IsNumber(val)) {
                        csi_engine_set_force_calibration(val->valuedouble != 0.0);
                    }
                } else if (strcmp(cmd->valuestring, "set_annotation") == 0) {
                    cJSON *val = cJSON_GetObjectItem(msg, "value");
                    if (cJSON_IsString(val)) {
                        csi_engine_set_manual_annotation(val->valuestring);
                    }
                } else if (strcmp(cmd->valuestring, "set_detection") == 0) {
                    cJSON *val = cJSON_GetObjectItem(msg, "value");
                    if (cJSON_IsBool(val)) {
                        csi_engine_set_detection_enabled(cJSON_IsTrue(val));
                    } else if (cJSON_IsNumber(val)) {
                        csi_engine_set_detection_enabled(val->valuedouble != 0.0);
                    }
                } else if ((strcmp(cmd->valuestring, "set_motion_threshold") == 0) ||
                           (strcmp(cmd->valuestring, "set_threshold") == 0)) {
                    cJSON *val = cJSON_GetObjectItem(msg, "value");
                    if (cJSON_IsNumber(val)) {
                        csi_engine_set_motion_threshold((float)val->valuedouble);
                    }
                } else if (strcmp(cmd->valuestring, "set_profile_threshold") == 0) {
                    cJSON *name = cJSON_GetObjectItem(msg, "name");
                    cJSON *val = cJSON_GetObjectItem(msg, "value");
                    if (cJSON_IsString(name) && cJSON_IsNumber(val)) {
                        csi_engine_set_profile_threshold(
                            name->valuestring, (float)val->valuedouble);
                    }
                } else if (strcmp(cmd->valuestring, "set_notch") == 0) {
                    cJSON *val = cJSON_GetObjectItem(msg, "value");
                    if (cJSON_IsNumber(val)) {
                        csi_engine_set_notch((float)val->valuedouble);
                    }
                } else if (strcmp(cmd->valuestring, "set_logging") == 0) {
                    cJSON *val = cJSON_GetObjectItem(msg, "value");
                    if (cJSON_IsBool(val)) {
                        csi_engine_set_raw_logging(cJSON_IsTrue(val));
                    } else if (cJSON_IsNumber(val)) {
                        csi_engine_set_raw_logging(val->valuedouble != 0.0);
                    }
                } else if (strcmp(cmd->valuestring, "set_capture") == 0) {
                    cJSON *val = cJSON_GetObjectItem(msg, "value");
                    bool enable = false;

                    if (cJSON_IsBool(val)) {
                        enable = cJSON_IsTrue(val);
                    } else if (cJSON_IsNumber(val)) {
                        enable = (val->valuedouble != 0.0);
                    }

                    if (enable) {
                        if (queue_ensure_created(&s_browser_capture_queue,
                                                 BROWSER_CAPTURE_QUEUE_LEN) != ESP_OK) {
                            ESP_LOGE(TAG, "Unable to allocate browser capture queue.");
                            free(buf);
                            cJSON_Delete(msg);
                            return ESP_ERR_NO_MEM;
                        }
                    }

                    s_capture_enabled = enable;
                    queue_reset(s_browser_capture_queue);
                    if (enable) {
                        http_server_capture_enqueue_csv(CAPTURE_HEADER);
                        ESP_LOGI(TAG, "Dashboard browser capture enabled.");
                    } else {
                        ESP_LOGI(TAG, "Dashboard browser capture disabled.");
                    }
                }
            }
            cJSON_Delete(msg);
        } else {
            ESP_LOGW(TAG, "Ignoring malformed WS JSON payload.");
        }
    }

    free(buf);
    return ESP_OK;
}

static esp_err_t log_ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "Log websocket handshake from fd=%d", fd);

        if (queue_ensure_created(&s_log_stream_queue, LOG_STREAM_QUEUE_LEN) != ESP_OK) {
            ESP_LOGE(TAG, "Unable to allocate log stream queue.");
            return ESP_ERR_NO_MEM;
        }

        log_ws_add_client(fd);
        return ESP_OK;
    }

    httpd_ws_frame_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.type = HTTPD_WS_TYPE_TEXT;
    return httpd_ws_recv_frame(req, &pkt, 0);
}

esp_err_t http_server_start(void)
{
    if (s_server) {
        return ESP_OK;
    }

    memset(s_status_ws_fds, -1, sizeof(s_status_ws_fds));
    memset(s_log_ws_fds, -1, sizeof(s_log_ws_fds));
    memset(s_log_ws_header_sent, 0, sizeof(s_log_ws_header_sent));
    s_status_ws_count = 0;
    s_log_ws_count = 0;

    if (!s_ws_mutex) {
        s_ws_mutex = xSemaphoreCreateMutex();
        if (!s_ws_mutex) {
            return ESP_ERR_NO_MEM;
        }
    }

    s_capture_enabled = false;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.max_open_sockets = 7;
    cfg.lru_purge_enable = true;
    cfg.stack_size = 10240;

    ESP_LOGI(TAG, "Starting HTTP server on port %d", cfg.server_port);
    if (httpd_start(&s_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server.");
        return ESP_FAIL;
    }

    static const httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_get_handler,
    };
    static const httpd_uri_t status_ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = status_ws_handler,
        .is_websocket = true,
    };
    static const httpd_uri_t log_ws_uri = {
        .uri = "/ws/log",
        .method = HTTP_GET,
        .handler = log_ws_handler,
        .is_websocket = true,
    };

    if ((httpd_register_uri_handler(s_server, &root_uri) != ESP_OK) ||
        (httpd_register_uri_handler(s_server, &status_ws_uri) != ESP_OK) ||
        (httpd_register_uri_handler(s_server, &log_ws_uri) != ESP_OK)) {
        httpd_stop(s_server);
        s_server = NULL;
        return ESP_FAIL;
    }

    s_ws_running = true;
    if (xTaskCreatePinnedToCore(status_ws_push_task, "ws_status_push", 8192,
                                NULL, 5, &s_status_ws_task, 1) != pdPASS) {
        s_ws_running = false;
        httpd_stop(s_server);
        s_server = NULL;
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreatePinnedToCore(log_ws_push_task, "ws_log_push", 6144,
                                NULL, 5, &s_log_ws_task, 1) != pdPASS) {
        s_ws_running = false;
        if (s_status_ws_task) {
            vTaskDelete(s_status_ws_task);
            s_status_ws_task = NULL;
        }
        httpd_stop(s_server);
        s_server = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "HTTP server started. Open http://<device-ip>/ in your browser.");
    return ESP_OK;
}

void http_server_stop(void)
{
    s_ws_running = false;

    if (s_status_ws_task) {
        vTaskDelete(s_status_ws_task);
        s_status_ws_task = NULL;
    }
    if (s_log_ws_task) {
        vTaskDelete(s_log_ws_task);
        s_log_ws_task = NULL;
    }

    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
    }

    if (s_ws_mutex) {
        xSemaphoreTake(s_ws_mutex, portMAX_DELAY);
        memset(s_status_ws_fds, -1, sizeof(s_status_ws_fds));
        memset(s_log_ws_fds, -1, sizeof(s_log_ws_fds));
        memset(s_log_ws_header_sent, 0, sizeof(s_log_ws_header_sent));
        s_status_ws_count = 0;
        s_log_ws_count = 0;
        xSemaphoreGive(s_ws_mutex);
    }

    s_capture_enabled = false;
    queue_reset(s_browser_capture_queue);
    queue_reset(s_log_stream_queue);
}

bool http_server_capture_is_enabled(void)
{
    return s_capture_enabled;
}

void http_server_capture_enqueue_csv(const char *csv_line)
{
    capture_row_t row;

    if (!s_capture_enabled || !s_browser_capture_queue ||
        !csv_line || csv_line[0] == '\0') {
        return;
    }

    memset(&row, 0, sizeof(row));
    strlcpy(row.payload, csv_line, sizeof(row.payload));
    (void)xQueueSend(s_browser_capture_queue, &row, 0);
}

bool http_server_log_stream_is_enabled(void)
{
    return ws_client_count(s_log_ws_fds, s_log_ws_count) > 0;
}

void http_server_log_enqueue_csv(const char *csv_line)
{
    capture_row_t row;

    if (!http_server_log_stream_is_enabled() || !s_log_stream_queue ||
        !csv_line || csv_line[0] == '\0') {
        return;
    }

    memset(&row, 0, sizeof(row));
    strlcpy(row.payload, csv_line, sizeof(row.payload));
    (void)xQueueSend(s_log_stream_queue, &row, 0);
}
