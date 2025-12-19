#include "usb_ring_buffer.h"

#define UART_BUFFER_SIZE 256

static uint8_t uart_rx_buffer[UART_BUFFER_SIZE];
static volatile uint16_t uart_rx_head = 0;
static volatile uint16_t uart_rx_tail = 0;

void uart_rx_enqueue(uint8_t byte)
{
    uint16_t next_head = (uart_rx_head + 1) % UART_BUFFER_SIZE;
    if (next_head != uart_rx_tail) // tránh buffer đầy
    {
        uart_rx_buffer[uart_rx_head] = byte;
        uart_rx_head = next_head;
    }
}

bool CDC_Receive_Byte(uint8_t* byte)
{
    if (uart_rx_head == uart_rx_tail)
        return false; // buffer rỗng

    *byte = uart_rx_buffer[uart_rx_tail];
    uart_rx_tail = (uart_rx_tail + 1) % UART_BUFFER_SIZE;
    return true;
}
