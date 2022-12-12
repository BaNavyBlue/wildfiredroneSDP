#ifndef TELEM_UART_H
#define TELEM_UART_H

#include "telem_defines.h"
void send_image(char **argv, thread_data *shared, uint16_t comp_id, mav_data *package, uint8_t *buffer);
uint8_t crc_check(mav_data *indata, uint8_t *inbuffer, uint8_t extra_crc);

#endif
