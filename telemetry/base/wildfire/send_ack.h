#ifndef SEND_ACK_H
#define SEND_ACK_H

#include "telem_defines.h"

typedef struct{
    uint16_t command; // using to request packet number
    uint8_t result;
    uint8_t progress;
    int32_t result_param2;
    uint8_t target_system;
    uint8_t target_component;
}ack_data;

void send_ack(mav_data *indata, ack_data *payload, thread_data *shared);
void handle_ack(ack_data *ackmsg);

#endif
