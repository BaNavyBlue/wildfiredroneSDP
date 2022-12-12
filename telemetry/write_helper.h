#ifndef WRITE_HELPER_H
#define WRITE_HELPER_H

#include "telem_defines.h"

/*typedef struct {
    pthread_mutex_t *uart_mut;
    int *fd;
}thread_data;*/

void write_packet(uint8_t *buffer, thread_data *shared);

#endif
