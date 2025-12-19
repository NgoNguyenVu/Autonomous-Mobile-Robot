#ifndef USB_RING_BUFFER_H
#define USB_RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

void uart_rx_enqueue(uint8_t byte);
bool CDC_Receive_Byte(uint8_t* byte);

#endif // USB_RING_BUFFER_H
