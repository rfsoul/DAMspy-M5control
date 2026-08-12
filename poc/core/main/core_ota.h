#pragma once

#include <stdbool.h>

#include "espnow_echo.h"

bool core_ota_handle_message(const espnow_hid_message_t *message, bool operation_busy);
void core_ota_poll(void);
