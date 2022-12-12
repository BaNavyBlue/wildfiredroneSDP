#include "send_ack.h"

void send_ack(mav_data *indata, ack_data *payload, thread_data *shared){
    mav_data ack_packet;  
    uint8_t buffer[280] = {0};
    buffer[0] = MAGIC;
    buffer[1] = 9;
    buffer[2] = 0;
    buffer[3] = 0;
    buffer[4] = indata->seq;
    buffer[5] = indata->sysid;
    buffer[6] = indata->compid;
    buffer[7] = ACK_MSG;
    buffer[8] = 0;
    buffer[9] = 0;
    buffer[10] = payload->command;
    buffer[11] = payload->command >> 8;
    buffer[12] = payload->result;
    buffer[13] = payload->progress;
    // Use lower two bytes for replacement packet
    buffer[13] = payload->result_param2; // LSB
    buffer[14] = payload->result_param2 >> 8; 
    buffer[15] = payload->result_param2 >> 16;
    buffer[16] = payload->result_param2 >> 24; // MSB
    buffer[17] = payload->target_system;
    buffer[18] = payload->target_component;
    uint16_t checksum = crc_calculate(&buffer[1], HEADER_L - 1);
    crc_accumulate_buffer(&checksum, (const char*)&buffer[10], buffer[1]);
    crc_accumulate(ACK_CRC, &checksum);
    printf("ACK checksum: 0x%x\n", checksum);
    buffer[10 + buffer[1]] = checksum;
    buffer[10 + buffer[1] + 1] = checksum >> 8;
    write_packet(buffer, shared);
}

void handle_ack(ack_data *ackmsg){



}
