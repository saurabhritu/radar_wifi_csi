#pragma once

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
