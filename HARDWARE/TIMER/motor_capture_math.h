#ifndef MOTOR_CAPTURE_MATH_H
#define MOTOR_CAPTURE_MATH_H

#include <stdint.h>

#define MOTOR_CAPTURE_POLES 14U
#define MOTOR_CAPTURE_MIN_TIMEOUT_MS 100U
#define MOTOR_CAPTURE_MAX_TIMEOUT_MS 30000U

typedef struct {
    uint32_t last_timestamp_us;
    uint32_t last_edge_ms;
    uint32_t period_us;
    uint8_t primed;
    uint8_t valid;
} MotorCaptureState;

/* Reconstruct the most recent captured edge from the CURRENT extended time.
   This also handles a pre-wrap CCR serviced after the update IRQ advanced
   epoch. Capture IRQ latency must be less than one 65536 us timer cycle. */
static uint32_t MotorCapture_Timestamp(uint32_t epoch, uint32_t captured,
                                       uint32_t counter, uint8_t wrap_pending)
{
    if (wrap_pending != 0U)
        epoch += 65536UL;
    return epoch + counter - ((counter - captured) & 0xffffUL);
}

static uint32_t MotorCapture_TimeoutMs(uint32_t period_us)
{
    uint32_t timeout_ms;
    if ((period_us == 0U) || (period_us >= 10000000UL))
        return MOTOR_CAPTURE_MAX_TIMEOUT_MS;
    timeout_ms = ((period_us + 999UL) / 1000UL) * 3UL;
    if (timeout_ms < MOTOR_CAPTURE_MIN_TIMEOUT_MS)
        timeout_ms = MOTOR_CAPTURE_MIN_TIMEOUT_MS;
    return timeout_ms;
}

static uint8_t MotorCapture_Expired(uint32_t now_ms, uint32_t last_ms,
                                    uint32_t period_us)
{
    return ((uint32_t)(now_ms - last_ms) > MotorCapture_TimeoutMs(period_us));
}

static void MotorCapture_Update(volatile MotorCaptureState *state,
                                 uint32_t timestamp_us, uint32_t now_ms)
{
    uint32_t elapsed_us;
    if ((state->primed == 0U) ||
        MotorCapture_Expired(now_ms, state->last_edge_ms,
                             state->valid ? state->period_us : 0U))
    {
        state->valid = 0U;
        state->period_us = 0U;
    }
    else
    {
        elapsed_us = timestamp_us - state->last_timestamp_us;
        state->period_us = elapsed_us;
        state->valid = (elapsed_us != 0U);
    }
    state->last_timestamp_us = timestamp_us;
    state->last_edge_ms = now_ms;
    state->primed = 1U;
}

static uint16_t MotorCapture_RPM(const MotorCaptureState *state, uint32_t now_ms)
{
    uint64_t rpm;
    if ((state->valid == 0U) || (state->period_us == 0U) ||
        MotorCapture_Expired(now_ms, state->last_edge_ms, state->period_us))
        return 0U;
    rpm = 120000000ULL / ((uint64_t)state->period_us * MOTOR_CAPTURE_POLES);
    return (rpm > 65535ULL) ? 65535U : (uint16_t)rpm;
}

#endif
