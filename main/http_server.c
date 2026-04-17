/**
 * @file http_server.c
 * @brief ESP-IDF HTTP server with WebSocket support.
 *
 * Endpoints:
 *   GET /    → Serves the embedded dashboard HTML.
 *   GET /ws  → WebSocket. Pushes CSI status JSON every 100 ms.
 *              Accepts: {"cmd":"recalibrate"} and {"cmd":"set_threshold","value":N}
 */

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "wifi_csi.h"
#include "http_server.h"
#include "dashboard.h"

static const char *TAG = "http_server";

static httpd_handle_t s_server = NULL;

/* Keep track of all live WebSocket file descriptors */
#define MAX_WS_CLIENTS 4
static int s_ws_fds[MAX_WS_CLIENTS];
static int s_ws_count = 0;

/* ──────────────────────────────────────────────────────
 * Helper: broadcast a string to all WebSocket clients
 * ────────────────────────────────────────────────────── */
static void ws_broadcast(const char *payload, size_t len)
{
    for (int i = 0; i < s_ws_count; ) {
        httpd_ws_frame_t pkt = {
            .type    = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)payload,
            .len     = len,
        };
        esp_err_t ret = httpd_ws_send_frame_async(s_server, s_ws_fds[i], &pkt);
        if (ret != ESP_OK) {
            /* client gone — remove from list */
            ESP_LOGD(TAG, "WS client %d gone, removing.", s_ws_fds[i]);
            s_ws_fds[i] = s_ws_fds[--s_ws_count];
        } else {
            i++;
        }
    }
}

/* ──────────────────────────────────────────────────────
 * WebSocket push task (~10 Hz)
 * ────────────────────────────────────────────────────── */
static void ws_push_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));

        if (s_ws_count == 0) continue;

        csi_status_t st;
        csi_engine_get_status(&st);

        /* Build JSON */
        cJSON *root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "status",
            st.motion_state == CSI_STATE_MOTION ? "MOTION_DETECTED" : "CLEAR");
        
        /* Activity classification */
        const char *act_str = "STILL";
        if (st.activity == CSI_ACT_BREATHING) act_str = "BREATHING";
        if (st.activity == CSI_ACT_MOVING)    act_str = "MOVING";
        cJSON_AddStringToObject(root, "activity", act_str);

        /* Direction detection */
        const char *dir_str = "NONE";
        if (st.direction == CSI_DIR_TOWARD) dir_str = "TOWARD";
        if (st.direction == CSI_DIR_AWAY)   dir_str = "AWAY";
        cJSON_AddStringToObject(root, "direction", dir_str);

        cJSON_AddNumberToObject(root, "variance",     (double)st.variance_score);
        cJSON_AddNumberToObject(root, "subcarriers",  st.num_subcarriers);
        cJSON_AddNumberToObject(root, "event_count",  st.motion_event_count);
        cJSON_AddStringToObject(root, "remote_addr",  st.remote_addr);

        /* Amplitudes array (capped at 64 for WS payload size) */
        uint8_t sc = st.num_subcarriers;
        if (sc > 64) sc = 64;
        cJSON *amps = cJSON_CreateArray();
        for (int i = 0; i < sc; i++) {
            cJSON_AddItemToArray(amps,
                cJSON_CreateNumber((double)st.amplitudes[i]));
        }
        cJSON_AddItemToObject(root, "amplitudes", amps);

        char *str = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);

        if (str) {
            ws_broadcast(str, strlen(str));
            free(str);
        }
    }
}

/* ──────────────────────────────────────────────────────
 * URI: GET /  → serves dashboard HTML
 * ────────────────────────────────────────────────────── */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, DASHBOARD_HTML, HTTPD_RESP_USE_STRLEN);
}

/* ──────────────────────────────────────────────────────
 * URI: GET /ws  → WebSocket upgrade
 * ────────────────────────────────────────────────────── */
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        /* New WebSocket connection */
        int fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "WebSocket handshake from fd=%d", fd);

        /* Register client fd */
        if (s_ws_count < MAX_WS_CLIENTS) {
            s_ws_fds[s_ws_count++] = fd;
        }
        return ESP_OK;
    }

    /* Receive a frame from the client */
    httpd_ws_frame_t pkt = {.type = HTTPD_WS_TYPE_TEXT};
    uint8_t buf[128];
    pkt.payload = buf;
    pkt.len = sizeof(buf) - 1;

    esp_err_t ret = httpd_ws_recv_frame(req, &pkt, sizeof(buf) - 1);
    if (ret != ESP_OK) return ret;

    buf[pkt.len] = '\0';
    ESP_LOGD(TAG, "WS recv: %s", buf);

    cJSON *msg = cJSON_Parse((char *)buf);
    if (msg) {
        cJSON *cmd = cJSON_GetObjectItem(msg, "cmd");
        if (cJSON_IsString(cmd)) {
            if (strcmp(cmd->valuestring, "recalibrate") == 0) {
                csi_engine_recalibrate();
            } else if (strcmp(cmd->valuestring, "set_threshold") == 0) {
                cJSON *val = cJSON_GetObjectItem(msg, "value");
                if (cJSON_IsNumber(val)) {
                    csi_engine_set_threshold((float)val->valuedouble);
                }
            }
        }
        cJSON_Delete(msg);
    }
    return ESP_OK;
}

/* ──────────────────────────────────────────────────────
 * Public API
 * ────────────────────────────────────────────────────── */

esp_err_t http_server_start(void)
{
    memset(s_ws_fds, -1, sizeof(s_ws_fds));
    s_ws_count = 0;

    httpd_config_t cfg    = HTTPD_DEFAULT_CONFIG();
    cfg.server_port       = 80;
    cfg.max_open_sockets  = 4;      /* Lowered to fit within LWIP_MAX_SOCKETS limit */
    cfg.lru_purge_enable  = true;
    cfg.stack_size        = 10240;  /* Increased for robust processing */

    ESP_LOGI(TAG, "Starting HTTP server on port %d", cfg.server_port);
    if (httpd_start(&s_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server.");
        return ESP_FAIL;
    }

    /* Register URI handlers */
    static const httpd_uri_t root_uri = {
        .uri     = "/",
        .method  = HTTP_GET,
        .handler = root_get_handler,
    };
    static const httpd_uri_t ws_uri = {
        .uri         = "/ws",
        .method      = HTTP_GET,
        .handler     = ws_handler,
        .is_websocket= true,
    };
    httpd_register_uri_handler(s_server, &root_uri);
    httpd_register_uri_handler(s_server, &ws_uri);

    /* Start WebSocket push task on Core 1 (CSI runs on Core 0) */
    xTaskCreatePinnedToCore(ws_push_task, "ws_push", 4096, NULL,
                            5, NULL, 1);

    ESP_LOGI(TAG, "HTTP server started. Open http://<device-ip>/ in your browser.");
    return ESP_OK;
}

void http_server_stop(void)
{
    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
    }
}
