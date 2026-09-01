#pragma once

#include <stdbool.h>

#include "espnow_echo.h"

bool core_usb_proxy_start(void);
bool core_usb_proxy_handle(const espnow_hid_message_t *request);

