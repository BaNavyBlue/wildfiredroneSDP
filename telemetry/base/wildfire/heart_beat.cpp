

#include "heart_beat.h"

//char *uart_name = (char*)"/dev/ttyUSB0";
//int baudrate = 115200;

void* heart_beat(void *data){
    thread_data *shared = (thread_data*)data;
    mavlink_message_t msg;
    //serial_port(uart_name, baudrate);
    mavlink_msg_heartbeat_pack(MAV_SYS_ID, MAV_COMP_ID, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC
                               , MAV_MODE_FLAG_TEST_ENABLED, 0, MAV_STATE_UNINIT);
    uint8_t buffer[280] = {0};
    buffer[0] = msg.magic;
    buffer[1] = msg.len;
    printf("len: %u\n", msg.len);
    buffer[2] = msg.incompat_flags;
    buffer[3] = msg.compat_flags;
    buffer[4] = msg.seq;
    buffer[5] = msg.sysid;
    buffer[6] = msg.compid;
    buffer[7] |= msg.msgid & 0xFF;
    buffer[8] |= (msg.msgid >> 8) & 0xFF;
    buffer[9] |= (msg.msgid >> 16) & 0xFF;
    unsigned int i, j;
    i = 0;
    for(j = 0; j < (msg.len / 8U) + 1U; j++){
        //printf("buffer: 0x");
        for(; i < (8 * (j + 1)) && i <(msg.len); i++){
            buffer[10 + i] = (msg.payload64[j] >> (/*56 -*/ ((i * 8) % 64))) & 0xFF;
            //printf("%02x", buffer[10 + i]);
        }
        //printf("\npayload: 0x%016llx\n", msg.payload64[j]);
        //printf("i: %u\n", i);
    }
    //printf("i: %u\n", i);
    //printf("checksum: %x\n", msg.checksum);
    buffer[10 + i++] = msg.checksum & 0xFF;
    buffer[10 + i++] = (msg.checksum >> 8 ); 

    while(1){
        //printf("writing message bytes writen: %d\n", serial_port.write_message(msg));
        write_packet(buffer, shared);
        //printf("sending heartbeat\n");
        sleep(1);
    }

}
