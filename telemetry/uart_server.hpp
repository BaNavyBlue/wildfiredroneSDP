#ifndef UART_SERVER_HPP
#define UART_SERVER_HPP

#include "telem_defines.h"

int uart_server(int port, thread_data *shared, mav_data *package, uint8_t *buffer); // nothing special currently


#endif
