#pragma once

#include "esp_err.h"


/*
 * Stage-one transport test: echo each received ESP-NOW frame back to
 * its sender without interpreting or modifying its payload.
 */
esp_err_t espnow_echo_start(void);
