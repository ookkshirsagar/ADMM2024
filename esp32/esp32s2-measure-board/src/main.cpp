#include <Arduino.h>

#include <driver/uart.h>
#include "esp32-hal-gpio.h"
#include "hal/gpio_hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "measure.h"
#include "defines.h"
#include "commands.h"

uint8_t *uart_rx_buffer = (uint8_t *)malloc(UART_BUFF_SIZE);
uint8_t *uart_tx_buffer = (uint8_t *)malloc(UART_BUFF_SIZE);

void uart_init();
void uart_send_buffer();
void uart_send_error();
void uart_return_freq_hz();
void uart_return_current_ua();
void uart_return_measure_duration();

volatile bool is_measuring = false;
uint8_t received_length = 0;

uint32_t signal_frequency = 500;
uint32_t signal_current = 200;
uint32_t signal_periods = 2;

typedef union _data
{
    float float_value;
    char bytes_value[4];
} Impedance_Union;

void setup()
{
    pinMode(LED_2, OUTPUT);
    pinMode(LED_3, OUTPUT);
    digitalWrite(LED_2, 0);
    digitalWrite(LED_3, 0);
    measure_init(&is_measuring);
    uart_init();
    delay(100);
}

void loop()
{

    if (!is_measuring)
    {
        uart_get_buffered_data_len(UART_PORT, (size_t *)&received_length);
        received_length = uart_read_bytes(UART_PORT, uart_rx_buffer, received_length, 100);

        if (received_length != 0)
        {
            digitalWrite(LED_2, 1);
            if (received_length == UART_BUFF_SIZE)
            {
                uint8_t command = uart_rx_buffer[0];

                uint32_t data1 = (uint32_t)((uart_rx_buffer[1] << 24) | (uart_rx_buffer[2] << 16) | (uart_rx_buffer[3] << 8) | uart_rx_buffer[4]);
                uint32_t data2 = (uint32_t)((uart_rx_buffer[5] << 24) | (uart_rx_buffer[6] << 16) | (uart_rx_buffer[7] << 8) | uart_rx_buffer[8]);

                switch (command)
                {
                case CMD_SET_FREQ_HZ:
                {
                    if (data1 <= 1000 && data1 > 1)
                    {
                        signal_frequency = data1;
                    }
                    uart_return_freq_hz();
                    break;
                }
                case CMD_GET_FREQ_HZ:
                {
                    uart_return_freq_hz();
                    break;
                }
                case CMD_SET_CURRENT_UA:
                {
                    if (data1 <= 330 && data1 > 50)
                    {
                        signal_current = data1;
                    }
                    uart_return_current_ua();
                    break;
                }
                case CMD_GET_CURRENT_UA:
                {
                    uart_return_current_ua();
                    break;
                }
                case CMD_SET_MEASURE_DURATION:
                {
                    if (data1 <= 100 && data1 >= 1)
                    {
                        signal_periods = data1;
                    }
                    uart_return_measure_duration();
                    break;
                }
                case CMD_GET_MEASURE_DURATION:
                {
                    uart_return_measure_duration();
                    break;
                }
                case CMD_START_MEASUREMENT:
                {
                    uint32_t measure_duration_ms = trigger_measure(signal_frequency, signal_current, signal_periods);
                    uart_tx_buffer[0] = CMD_ANSWER_START_IMPEDANCE;
                    uart_tx_buffer[1] = (measure_duration_ms >> 24) & 0xFF; // MSB
                    uart_tx_buffer[2] = (measure_duration_ms >> 16) & 0xFF;
                    uart_tx_buffer[3] = (measure_duration_ms >> 8) & 0xFF;
                    uart_tx_buffer[4] = measure_duration_ms & 0xFF;                                    // LSB
                    uart_tx_buffer[5] = uart_tx_buffer[6] = uart_tx_buffer[7] = uart_tx_buffer[8] = 0; // Zeros
                    uart_send_buffer();
                    break;
                }
                case CMD_GET_IMPEDANCE:
                {
                    float *impedance = get_last_impedance();
                    Impedance_Union impedance_abs;
                    impedance_abs.float_value = impedance[0];
                    Impedance_Union impedance_angle;
                    impedance_angle.float_value = impedance[1];

                    uart_tx_buffer[0] = CMD_ANSWER_GET_IMPEDANCE;
                    // uart_return_current_ua();

                    uart_tx_buffer[1] = impedance_abs.bytes_value[3];
                    uart_tx_buffer[2] = impedance_abs.bytes_value[2];
                    uart_tx_buffer[3] = impedance_abs.bytes_value[1];
                    uart_tx_buffer[4] = impedance_abs.bytes_value[0];

                    uart_tx_buffer[5] = impedance_angle.bytes_value[3];
                    uart_tx_buffer[6] = impedance_angle.bytes_value[2];
                    uart_tx_buffer[7] = impedance_angle.bytes_value[1];
                    uart_tx_buffer[8] = impedance_angle.bytes_value[0];

                    uart_send_buffer();
                    break;
                }
                default:
                {
                    uart_send_error();
                    break;
                }
                }
            }
        }
        digitalWrite(LED_2, 0);
    }
    else
    {
        uart_send_error();
    }
}
void uart_return_freq_hz()
{
    uart_tx_buffer[0] = CMD_ANSWER_GET_FREQ_HZ;
    uart_tx_buffer[1] = (signal_frequency >> 24) & 0xFF; // MSB
    uart_tx_buffer[2] = (signal_frequency >> 16) & 0xFF;
    uart_tx_buffer[3] = (signal_frequency >> 8) & 0xFF;
    uart_tx_buffer[4] = signal_frequency & 0xFF;                                       // LSB
    uart_tx_buffer[5] = uart_tx_buffer[6] = uart_tx_buffer[7] = uart_tx_buffer[8] = 0; // Zeros
    uart_send_buffer();
}

void uart_return_current_ua()
{
    uart_tx_buffer[0] = CMD_ANSWER_GET_CURRENT_UA;
    uart_tx_buffer[1] = (signal_current >> 24) & 0xFF; // MSB
    uart_tx_buffer[2] = (signal_current >> 16) & 0xFF;
    uart_tx_buffer[3] = (signal_current >> 8) & 0xFF;
    uart_tx_buffer[4] = signal_current & 0xFF;                                         // LSB
    uart_tx_buffer[5] = uart_tx_buffer[6] = uart_tx_buffer[7] = uart_tx_buffer[8] = 0; // Zeros
    uart_send_buffer();
}

void uart_return_measure_duration()
{
    uart_tx_buffer[0] = CMD_ANSWER_GET_MEASURE_DURATION;
    uart_tx_buffer[1] = (signal_periods >> 24) & 0xFF; // MSB
    uart_tx_buffer[2] = (signal_periods >> 16) & 0xFF;
    uart_tx_buffer[3] = (signal_periods >> 8) & 0xFF;
    uart_tx_buffer[4] = signal_periods & 0xFF;                                         // LSB
    uart_tx_buffer[5] = uart_tx_buffer[6] = uart_tx_buffer[7] = uart_tx_buffer[8] = 0; // Zeros
    uart_send_buffer();
}

void uart_init()
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
    };
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT, 1024, 1024, 0, NULL, 0);
}

void uart_send_buffer()
{
    uart_write_bytes(UART_PORT, (const char *)uart_tx_buffer, UART_BUFF_SIZE);
    uart_wait_tx_done(UART_PORT, 100);
}

void uart_send_error()
{
    for (size_t i = 0; i < UART_BUFF_SIZE; i++)
    {
        uart_tx_buffer[i] = ADMM_ERR;
    }
    uart_send_buffer();
    for (size_t i = 0; i < 5; i++)
    {
        digitalWrite(LED_2, 1);
        digitalWrite(LED_3, 1);
        delay(100);
        digitalWrite(LED_2, 0);
        digitalWrite(LED_3, 0);
    }
}