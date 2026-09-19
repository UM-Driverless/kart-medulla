#pragma once

#include "esp_err.h"
#include <stdint.h>

#define KM_HALL_FIELDS 8

/* Call once at boot. Classic ESP32 returns ESP_ERR_NOT_SUPPORTED. */
esp_err_t KM_HALL_Begin(void);

/* [init_err, bits, edges_h1, edges_h2, edges_h3, age_ms, interval_us, multi_changes]
 * init_err == ESP_OK means capture started, NOT that sensors are connected.
 * On failure, bits/age/interval = -1. Age = -1 before the first edge; interval
 * = -1 before the second observation. Timings saturate at INT32_MAX.
 * Counters are uint32 bit patterns transported in int32 fields, modulo 2^32.
 * No edges cannot distinguish standstill from a stuck/disconnected sensor. */
void KM_HALL_GetTelemetry(int32_t fields[KM_HALL_FIELDS]);
