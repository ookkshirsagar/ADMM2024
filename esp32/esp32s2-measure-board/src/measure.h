#ifndef MEASURE_H
#define MEASURE_H

#include <cstdint>

void measure_init(volatile bool *is_measuring);
void calibrate_measurement();

uint32_t trigger_measure(uint16_t signal_frequency, uint16_t signal_current, uint16_t measure_duration);

static void periodic_timer_callback(void *arg);

float* get_last_impedance();
#endif