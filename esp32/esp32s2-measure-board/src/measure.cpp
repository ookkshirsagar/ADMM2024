#include <Arduino.h>
#include <measure.h>
#include "esp_timer.h"
#include "defines.h"
#include <driver/dac.h>

#include <driver/adc.h>
#include "esp_adc_cal.h"
#include <hal/adc_types.h>

#include <stdio.h>
#include <stdlib.h>
#include <esp32-hal-adc.h>

volatile uint32_t measure_progress = 0;
double phase_resolution_steps = 360;
uint16_t measure_frequency = 1000;
uint16_t measure_current = 100;
uint16_t measure_duration = 5;
uint16_t dac_amplitude = 100;
float last_impedance[2] = {0, 0};
uint32_t measure_duration_samples = 0;
long double impedance_rms = 0;
double impedance_angle = 0;

static esp_adc_cal_characteristics_t *adc_chars;

uint16_t consecutive_lower_threshold = 10;
uint16_t consecutive_lower_count = 0;
uint32_t first_measured_voltage = 0;
double max_measured_voltage = 0;
uint32_t max_index_measured_voltage = 0;
bool flag_max_found = 0;
volatile bool *currently_measuring;

const esp_timer_create_args_t periodic_timer_args = {
    .callback = &periodic_timer_callback,
    // .arg = (void*),
    /* name is optional, but may help identify the timer when debugging */
    .name = "periodic"};

esp_timer_handle_t periodic_timer;

void measure_init(volatile bool *is_measuring)
{
    currently_measuring = is_measuring;
    esp_timer_create(&periodic_timer_args, &periodic_timer);

    dac_output_enable(DAC_PIN);
    dac_output_voltage(DAC_PIN, 0);

    adc1_config_width(ADC_RES);                              // Set ADC resolution to 12 bits
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11); // Set attenuation to 0 dB
    adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT, ADC_ATTEN_DB_11, ADC_RES, 1100, adc_chars);
}

uint32_t trigger_measure(uint16_t signal_frequency, uint16_t signal_current, uint16_t signal_duration)
{
    measure_frequency = signal_frequency;
    measure_current = signal_current;
    measure_duration = signal_duration;
    consecutive_lower_threshold = (DAC_SAMPLERATE / signal_frequency) / 3;
    first_measured_voltage = 0;
    consecutive_lower_count = 0;

    phase_resolution_steps = 2 * PI / (DAC_SAMPLERATE / phase_resolution_steps);

    uint32_t measure_duration_ms = (uint32_t)(1000 * (float)measure_duration / (float)measure_frequency) + 1;
    measure_duration_samples = (u_int32_t)(DAC_SAMPLERATE * ((float)measure_duration / (float)measure_frequency));

    double real_voltage = RS * signal_current;
    dac_amplitude = (uint16_t)((255 * (real_voltage / 1000000)) / 3.3);

    if (!esp_timer_is_active(periodic_timer))
    {
        digitalWrite(LED_3, 1);
        *currently_measuring = true;
        impedance_rms = 0;
        measure_progress = 0;
        flag_max_found = 0;
        max_index_measured_voltage = 0;
        impedance_angle = 0;
        uint64_t period = 1000000 / DAC_SAMPLERATE;
        esp_timer_start_periodic(periodic_timer, period);
        return measure_duration_ms;
    }
    else
    {
        return 0;
    }
}

static void periodic_timer_callback(void *arg)
{
    if (measure_progress > measure_duration_samples)
    {
        uint32_t adc_conversion = adc1_get_raw(ADC_CHANNEL);
        impedance_rms = impedance_rms + pow((double)adc_conversion - (double)first_measured_voltage, 2);
        impedance_rms /= measure_duration_samples; // Mean of squared ADC values
        impedance_rms = sqrt(impedance_rms);       // Square root to get RMS

        impedance_angle = ((float)(max_index_measured_voltage * 2 * PI * measure_frequency) / (float)DAC_SAMPLERATE) - PI;
        impedance_angle = ((uint16_t)(impedance_angle / phase_resolution_steps)) * phase_resolution_steps;
        dac_output_voltage(DAC_PIN, 0);
        digitalWrite(LED_3, 0);
        *currently_measuring = false;
        esp_timer_stop(periodic_timer);
    }
    else
    {
        float sin_value = cos(2 * PI * measure_frequency * measure_progress / DAC_SAMPLERATE);
        uint16_t dac_output = dac_amplitude * (-1 * sin_value + 1) / 2; // Convert [-1, 1] range to [0, amplitude]^

        uint32_t adc_conversion = adc1_get_raw(ADC_CHANNEL);
        if (measure_progress == 0)
        {
            first_measured_voltage = adc_conversion;
        }
        double power_adc = pow((int)adc_conversion - (int)first_measured_voltage, 2);
        impedance_rms = impedance_rms + power_adc;

        dac_output_voltage(DAC_PIN, dac_output + DAC_OFFSET);

        if (!flag_max_found && (power_adc > max_measured_voltage))
        {
            max_measured_voltage = power_adc;
            max_index_measured_voltage = measure_progress;
            consecutive_lower_count = 0;
        }
        else if (power_adc < max_measured_voltage)
        {
            consecutive_lower_count++;
            if (consecutive_lower_count >= consecutive_lower_threshold)
            {
                flag_max_found = 1;
            }
        }
        else
        {
            consecutive_lower_count = 0;
        }
    }
    measure_progress++;
}

float *get_last_impedance()
{
    uint32_t voltage = esp_adc_cal_raw_to_voltage(impedance_rms, adc_chars);
    last_impedance[0] = (float)(voltage);

    last_impedance[1] = (float)impedance_angle;
    return last_impedance;
}