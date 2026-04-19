#pragma once

#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Start the HTTP server.
 *
 * Registers:
 *   GET /    → Dashboard HTML page
 *   GET /ws  → WebSocket endpoint (pushes JSON to connected browsers)
 *
 * @return ESP_OK on success.
 */
esp_err_t http_server_start(void);

/**
 * @brief Stop the HTTP server and clean up.
 */
void http_server_stop(void);

/**
 * @brief True when dashboard-side CSV capture is enabled.
 */
bool http_server_capture_is_enabled(void);

/**
 * @brief Queue a CSV line for live dashboard capture.
 *
 * The line should not include a trailing newline.
 */
void http_server_capture_enqueue_csv(const char *csv_line);

/**
 * @brief True when at least one dedicated log-stream client is connected.
 */
bool http_server_log_stream_is_enabled(void);

/**
 * @brief Queue a CSV line for the dedicated log-stream endpoint.
 *
 * The line should not include a trailing newline.
 */
void http_server_log_enqueue_csv(const char *csv_line);
