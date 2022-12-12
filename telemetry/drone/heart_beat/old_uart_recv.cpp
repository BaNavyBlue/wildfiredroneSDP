#include "uart_recv.h"

uint8_t read_it(thread_data *shared, uint8_t *buffer);

void *uart_recv(void *data){
    thread_data *shared = (thread_data*)data;
    mav_data indata;
    uint8_t inbuffer[280];
    // Sit and spin
    unsigned int beats = 0;
    for(;;){
            uint8_t good_read = 0;
            //pthread_mutex_lock(shared->uart_mut);
            good_read = read_it(shared, inbuffer);
            //pthread_mutex_unlock(shared->uart_mut); 

            if(good_read){indata.magic = inbuffer[0];
                indata.len = inbuffer[1];
                indata.incompat_flags = inbuffer[2];
                indata.compat_flags = inbuffer[3];
                indata.seq = inbuffer[4];
                indata.sysid = inbuffer[5];
                indata.compid = inbuffer[6];
                indata.msgid = 0;                                                                                                       indata.msgid |= inbuffer[7];                                                                                            indata.msgid |= inbuffer[8] << 8;
                indata.msgid |= inbuffer[9] << 16;

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
    
    
                uint8_t extra_crc = 0;
    
    
    
                switch(indata.msgid){
                    case HEARTBEAT:
                        {
                            printf("Heartbeat Detected: %u\n", beats++);
                            extra_crc = 50;
                            printf("payload: 0x");
                            for(i = 0; i < indata.len; i++){
                                printf("%02x",indata.payload[i]);
                            }
                            printf("\n");
                            crc_check(&indata, inbuffer, extra_crc);
                        }
                        break;
                    case SEND_IMG:
                        {
                            printf("Receiving image\n");
                            extra_crc = 0;
                            //handle_image(&indata, inbuffer, extra_crc);
                        }
                        break;
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
                        }
                        break;
                    default:
                        {
                            printf("I have no idea what this msg is!\n");
                            printf("msgid: %u\n", indata.msgid);
                            crc_check(&indata, inbuffer, extra_crc);
                        }
                }
                //bytes_total = MAX_PACKET;
        }
        // Delay required to keep tx from getting cranky
        usleep(9);
        //printf("I'm after exit delay");
    }

}

uint8_t read_it(thread_data *shared, uint8_t *buffer){

    uint8_t bytes_to_read = 2;
    unsigned int i = 0;
    ssize_t read_bytes = 0;
    unsigned int rx_count = 0;
    for(i = 0; i < bytes_to_read; i++){
        read_bytes = read(*shared->fd, &buffer[rx_count], 1/*280 - rx_count*/);
            /*if(read_bytes > 0){ 
                printf("read_bytes %u\n",read_bytes);
            }*/
            //pthread_mutex_unlock(shared->uart_mut);
        if(read_bytes < 0){
            printf("Error Bit\n");
            perror("read");
            return 0;
                //close(fd);i
                //return -1;
        } else if(read_bytes == 0 && rx_count == 0) {
            return 0;
        }
    
        if(read_bytes > 0 && rx_count == 0){
            if(buffer[0] == MAGIC){
                rx_count += read_bytes;
                i = 0;
            } //else {
                    //pthread_mutex_unlock(shared->uart_mut);
                //}
        } else {
            if(rx_count == 1){
                 bytes_to_read = buffer[1] + BASE_SIZE;
            }
            rx_count += read_bytes;
        }
            /*if(read_bytes > 0){                                                                                                       printf("read_bytes: %d rx_count: %d\n", read_bytes, rx_count);
            }*/   
    }
    if(bytes_to_read > 2){
        return 1;
    } else {
        return 0;
    }
}
