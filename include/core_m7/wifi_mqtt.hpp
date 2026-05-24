#pragma once

#include <stdint.h>

void wifi_mqtt_begin();
void wifi_mqtt_update(uint32_t now_ms);
bool wifi_mqtt_is_wifi_connected();
bool wifi_mqtt_is_mqtt_connected();
bool wifi_mqtt_is_safety_enabled();
