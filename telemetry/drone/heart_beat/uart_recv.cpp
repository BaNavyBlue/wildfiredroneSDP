#include "uart_recv.h"

//uint8_t read_it(mav_data *indata, thread_data *shared, uint8_t *buffer);
void handle_image(mav_data *indata, uint8_t *inbuffer, uint8_t extra_crc);

extern uint32_t errorz;

void *uart_recv(void *data){
    thread_data *shared = (thread_data*)data;
    mav_data indata;
    uint8_t inbuffer[280];
    char boggle = ZERO;
    // Sit and spin
    unsigned int beats = 0;
    printf("CPS: %lu\n", CLOCKS_PER_SEC);
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    int rx_count = ZERO;
    ssize_t read_bytes, sent_bytes;
    uint16_t final_form = ZERO;
    uint16_t bytes_total = MAX_PACKET;
    // Sit and spin
    for(;;){
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now = (uint64_t)ts.tv_sec * 1000000000U + (uint64_t)ts.tv_nsec;
        //pthread_mutex_lock(shared->uart_mut);
        read_bytes = read(*shared->fd, &inbuffer[rx_count], 1/*bytes_total - rx_count*/);
   
        if(read_bytes < 0){                                                                                                         printf("Error Bit\n");
             perror("read");
             //close(fd);
             //return -1;
        }
   
        if(read_bytes > 0 && rx_count == 0){
            if(inbuffer[0] == MAGIC){
                rx_count += read_bytes;
            }
        } else {
            rx_count += read_bytes;
        }
    
        if(rx_count > 8 && !boggle){
            indata.magic = inbuffer[0];
            indata.len = inbuffer[1];
            indata.incompat_flags = inbuffer[2];
            indata.compat_flags = inbuffer[3];
            indata.seq = inbuffer[4];
            indata.sysid = inbuffer[5];
            indata.compid = inbuffer[6];
            indata.msgid = 0;
            indata.msgid |= inbuffer[7];
            indata.msgid |= inbuffer[8] << 8;
            indata.msgid |= inbuffer[9] << 16;
            boggle ^= 1;
            final_form = BASE_SIZE + indata.len;
        }
        if(rx_count == final_form && final_form){
            int i;
   
            for(i = 0; i < indata.len; i++){
                indata.payload[i] = inbuffer[i + 10];
            }
   
            indata.checksum = 0;
            indata.checksum |= inbuffer[10 + i++];
            indata.checksum |= inbuffer[10 + i++] << 8;
   
            /*for(int j = 0; j < 13; j++){
                indata.signiture[j] = inbuffer[10 + i++];
            }*/
            boggle ^= 1;
   
   
            uint8_t extra_crc = 0;
   
   
           clock_gettime(CLOCK_MONOTONIC, &ts);
           uint64_t later = (uint64_t)ts.tv_sec * 1000000000U + (uint64_t)ts.tv_nsec;
           long double seconds = ((long double) (later - now)) / 1000000U;
           //printf("seconds: %lf\n", seconds);
           long double throughput = ((long double)(12 + indata.len)) / seconds;
           //printf("bytes per second: %lf\n", throughput);
  
            switch(indata.msgid){
                case HEARTBEAT:
                    {
                        //printf("Heartbeat Detected: %u\n", beats++);
                        extra_crc = H_BEAT_CRC;
                        /*printf("payload: 0x");
                        for(i = 0; i < indata.len; i++){
                            printf("%02x",indata.payload[i]);
                        }
                        printf("\n");*/
                        crc_check(&indata, inbuffer, extra_crc);
                        break;
                    }
                case SEND_IMG:
                    {
                        //printf("Receiving image\n");
                        //clock_gettime(CLOCK_MONOTONIC, &ts);
                        //uint64_t later = (uint64_t)ts.tv_sec * 1000000000U + (uint64_t)ts.tv_nsec;
                        //long double seconds = ((long double) (later - now)) / 1000000U;
                        //printf("seconds: %lf\n", seconds);
                        //long double throughput = ((long double)(12 + indata.len)) / seconds;
                        //printf("bytes per second: %lf\n", throughput);
                        extra_crc = SEND_IMG_CRC;
                        handle_image(&indata, inbuffer, extra_crc);
                        break;
                    }
               case RAD_MSG:
                    {
                        printf("Radio Messege\n");
                        printf("rssi: %u\n", indata.payload[0]);
                        printf("remrssi: %u\n", indata.payload[1]);
                        printf("txbuf: %u\n", indata.payload[2]);
                        printf("noise: %u\n", indata.payload[3]);
                        printf("remnoise: %u\n", indata.payload[4]);
                        uint16_t rxerrors = 0;
                        rxerrors |= indata.payload[5];
                        rxerrors |= indata.payload[6] << 8;
                        printf("rxerrors: %u\n", rxerrors);
                        uint16_t fixed = 0;
                        fixed |= indata.payload[7];
                        fixed |= indata.payload[8] << 8;
                        printf("fixed: %u\n", fixed);
                        crc_check(&indata, inbuffer, extra_crc);
                        break;
                    }
                case ACK_MSG:
                    {
                        printf("ACK_MSG\n");
                        extra_crc = ACK_CRC;
                        if(crc_check(&indata, inbuffer, extra_crc)){
                            printf("Ack Good\n");
                            shared->flags |= RECEIVED_START_ACK;
                        }
                        break;
                    }
                default:
                    {
                        printf("I have no idea what this msg is!\n");
                        printf("msgid: %u\n", indata.msgid);
                        crc_check(&indata, inbuffer, extra_crc);
                        break;
                    }
            }
   
            rx_count = 0;
            //bytes_total = MAX_PACKET;
        }
        // Delay required to keep tx from getting cranky
        usleep(9);
        //printf("I'm after exit delay");
            //good_read = read_it(&indata, shared, inbuffer);
            //pthread_mutex_unlock(shared->uart_mut); 

       //printf("I'm after exit delay");
    }

}

void handle_image(mav_data *indata, uint8_t *inbuffer, uint8_t extra_crc){

    uint16_t p_count = 0;
    p_count |= indata->payload[0];
    p_count |= indata->payload[1] << 8;

    crc_check(indata, inbuffer, extra_crc);
    printf("checksum 0x%02x\n", indata->checksum);
    printf("magic: %x\n", indata->magic);
    printf("len: %u\n", indata->len);
    printf("incompat_flags: %u\n", indata->incompat_flags);
    printf("compat_flags: %u\n", indata->compat_flags);
    printf("seq: %u\n", indata->seq);
    printf("sysid: %u\n", indata->sysid);
    printf("compid: %u\n", indata->compid);
    printf("msgid: %u\n", indata->msgid);
    printf("errors: %u\n", errorz);

    printf("packet no.: %u\n", p_count);
}
