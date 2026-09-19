#include "km_hall.h"
#include "km_hall_state.h"
#include "km_gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include <limits.h>

static esp_err_t s_status = ESP_ERR_INVALID_STATE;
static bool s_started;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static km_hall_state_t s_state;

#ifdef CONFIG_IDF_TARGET_ESP32S3
#include "soc/gpio_reg.h"
#include "soc/soc.h"

_Static_assert(PIN_MOTOR_HALL_1 < 32 && PIN_MOTOR_HALL_3 < 32
               && PIN_MOTOR_HALL_2 >= 32 && PIN_MOTOR_HALL_2 < 64,
               "Hall register sampling must match the GPIO banks");

/* Two register reads for the two GPIO banks. A transition between these reads
 * can give an ambiguous state; multi_changes exposes some, not all, missed edges.
 * Register access and the inlined accumulator stay usable with flash cache off. */
static uint8_t IRAM_ATTR read_bits(void)
{
    uint32_t low = REG_READ(GPIO_IN_REG);
    uint32_t high = REG_READ(GPIO_IN1_REG);
    return ((low >> PIN_MOTOR_HALL_1) & 1u)
         | (((high >> (PIN_MOTOR_HALL_2 - 32)) & 1u) << 1)
         | (((low >> PIN_MOTOR_HALL_3) & 1u) << 2);
}

static void IRAM_ATTR hall_isr(void *arg)
{
    (void)arg;
    portENTER_CRITICAL_ISR(&s_mux);
    km_hall_observe(&s_state, read_bits(), esp_timer_get_time());
    portEXIT_CRITICAL_ISR(&s_mux);
}
#endif

esp_err_t KM_HALL_Begin(void)
{
    if (s_started) return s_status;
    s_started = true;
#ifndef CONFIG_IDF_TARGET_ESP32S3
    s_status = ESP_ERR_NOT_SUPPORTED;
#else
    const gpio_num_t pins[] = {PIN_MOTOR_HALL_1, PIN_MOTOR_HALL_2, PIN_MOTOR_HALL_3};
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_MOTOR_HALL_1) | (1ULL << PIN_MOTOR_HALL_2)
                      | (1ULL << PIN_MOTOR_HALL_3),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    s_status = gpio_config(&cfg);
    if (s_status != ESP_OK) return s_status;
    s_status = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    /* An existing service can be shared, but its allocation flags are its owner's. */
    if (s_status == ESP_ERR_INVALID_STATE) s_status = ESP_OK;
    if (s_status != ESP_OK) return s_status;

    unsigned added = 0;
    for (; added < 3; ++added) {
        s_status = gpio_isr_handler_add(pins[added], hall_isr, NULL);
        if (s_status != ESP_OK) goto fail;
    }
    s_state = (km_hall_state_t){.bits = read_bits(), .interval_us = -1};
    for (unsigned i = 0; i < 3; ++i) {
        s_status = gpio_set_intr_type(pins[i], GPIO_INTR_ANYEDGE);
        if (s_status != ESP_OK) goto fail;
        s_status = gpio_intr_enable(pins[i]);
        if (s_status != ESP_OK) goto fail;
    }
    return ESP_OK;
fail:
    for (unsigned i = 0; i < added; ++i) {
        gpio_intr_disable(pins[i]);
        gpio_isr_handler_remove(pins[i]);
    }
#endif
    return s_status;
}

static int32_t clamp_time(int64_t value)
{
    return value > INT32_MAX ? INT32_MAX : (int32_t)value;
}

void KM_HALL_GetTelemetry(int32_t fields[KM_HALL_FIELDS])
{
    portENTER_CRITICAL(&s_mux);
    km_hall_state_t state = s_state;
    int64_t now = esp_timer_get_time();
    portEXIT_CRITICAL(&s_mux);
    bool ready = s_status == ESP_OK;
    fields[0] = s_status;
    fields[1] = ready ? state.bits : -1;
    for (unsigned i = 0; i < 3; ++i)
        fields[2 + i] = ready ? (int32_t)state.edges[i] : 0;
    fields[5] = ready && state.seen_edge ? clamp_time((now - state.last_edge_us) / 1000) : -1;
    fields[6] = ready ? clamp_time(state.interval_us) : -1;
    fields[7] = ready ? (int32_t)state.multi_changes : 0;
}
