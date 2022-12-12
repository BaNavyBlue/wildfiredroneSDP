#include "uart_recv.h"

void handle_image(mav_data *indata, uint8_t *inbuffer, thread_data *shared, std::vector<uint16_t> *data, uint16_t *prev);

extern uint32_t errors;

// These are for recovering missing or bad crc data packets.


void *uart_recv(void *data){
    thread_data *shared = (thread_data*)data;
    mav_data indata;
    uint8_t inbuffer[280];
    char boggle = ZERO;
    // Sit and spin
    unsigned int beats = 0;

    // Timer stuff
    printf("CPS: %lu\n", CLOCKS_PER_SEC);
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t last_hb, last_rgb_pckt, t_btwn_hb, t_btwn_rgb; 
    last_hb = last_rgb_pckt = t_btwn_hb = t_btwn_rgb = (uint64_t)ts.tv_sec * 1000000000u + (uint64_t)ts.tv_nsec;

    // Data For handling image transfers.
    std::vector<uint16_t> RGB_image, thermal_image, telemetry;
    uint16_t RGB[2] = {0};
    uint16_t therm[2] = {0};
    uint16_t telem[2] = {0};
    int RGB_fd = 0;
    int therm_fd = 0;
    int telem_fd = 0;

 
    int rx_count = ZERO;
    ssize_t read_bytes;
    uint16_t final_form = ZERO;
    uint64_t prev_rgb_pkt = ZERO;
    // Sit and spin
    for(;;){
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now = (uint64_t)ts.tv_sec * 1000000000u + (uint64_t)ts.tv_nsec;
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
   
   
           //clock_gettime(CLOCK_MONOTONIC, &ts);
           //uint64_t later = (uint64_t)ts.tv_sec * 1000000000U + (uint64_t)ts.tv_nsec;
           //long double seconds = ((long double) (later - now)) / 1000000U;
           //printf("seconds: %Lf\n", seconds);
           //long double throughput = ((long double)(12 + indata.len)) / seconds;
           //printf("bytes per second: %Lf\n", throughput);
  
            switch(indata.msgid){
                case HEARTBEAT:
                    {
                        //printf("Heartbeat Detected: %u\n", beats++);
                        extra_crc = H_BEAT_CRC;
                        clock_gettime(CLOCK_MONOTONIC, &ts);
                        last_hb = (uint64_t)ts.tv_sec * 1000000000U + (uint64_t)ts.tv_nsec;
                        //printf("payload: 0x");
                        /*for(i = 0; i < indata.len; i++){
                            printf("%02x",indata.payload[i]);
                        }*/
                        //printf("\n");
                        crc_check(&indata, inbuffer, extra_crc);
                        break;
                    }
                case SEND_IMG:
                    {
                        //printf("Receiving image\n");
                        //clock_gettime(CLOCK_MONOTONIC, &ts);
                        //last_rgb_pckt = (uint64_t)ts.tv_sec * 1000000000U + (uint64_t)ts.tv_nsec;
                        //long double seconds = ((long double) (later - now)) / 1000000000U;
                        //printf("seconds: %lf\n", seconds);
                        //long double throughput = ((long double)(12 + indata.len)) / seconds;
                        //printf("bytes per second: %lf\n", throughput);
                        extra_crc = SEND_IMG_CRC;
                        if(crc_check(&indata, inbuffer, extra_crc)){
                            handle_image(&indata, inbuffer, shared, &RGB_image, RGB);
			    if(inbuffer[4] == 0){
				clock_gettime(CLOCK_MONOTONIC, &ts);
                                last_rgb_pckt = (uint64_t)ts.tv_sec * 1000000000U + (uint64_t)ts.tv_nsec;    
			    } else if(inbuffer[4] == 1){
                                clock_gettime(CLOCK_MONOTONIC, &ts);
                                last_rgb_pckt = (uint64_t)ts.tv_sec * 1000000000U + (uint64_t)ts.tv_nsec;
                                /*long double seconds = ((long double) (last_rgb_pckt - prev_rgb_pkt)) / 1000000000U;
                                printf("seconds: %lf\n", seconds);
                                long double throughput = ((long double)(12 + indata.len)) / (seconds);
                                printf("bytes per second: %lf\n", throughput);
				prev_rgb_pkt = last_rgb_pckt;*/
			    }
                        }
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
                            break;
			}
                    }
                default:
                    {
                        printf("I have no idea what this msg is!\n");
                        printf("msgid: %u\n", indata.msgid);
                        crc_check(&indata, inbuffer, extra_crc);
                        break;
                    }
            }


     	    //  Incase ACK packet corrumpted resend ACK 
    	    if(indata.msgid != SEND_IMG && shared->flags & SENT_START_ACK){
                    indata.seq = 0;
    	            ack_data payload;
                    payload.command = SEND_IMG;
                    //indata.msgid = ACK_MSG;
                    payload.result = 0;
                    payload.progress = 0;
                    payload.result_param2 = 0;
                    payload.target_system = SYS_ID_DRONE;
                    payload.target_component = COMP_ID_CAMERA;
                    send_ack(&indata, &payload, shared);
                    shared->flags |= SENT_START_ACK;
    	    }  
            rx_count = 0;
            //bytes_total = MAX_PACKET;
        }

        if((now > last_hb)){
            t_btwn_hb = now - last_hb;
        } else {
            t_btwn_hb = 0;
	}

	if((now > last_rgb_pckt)){
            t_btwn_rgb = now - last_rgb_pckt;
	} else {
            t_btwn_rgb = 0;
	}

        long double seconds = ((long double) (t_btwn_hb)) / 1000000000U;
        long double seconds_rgb = ((long double) (t_btwn_rgb)) / 1000000000U;
        //printf("seconds: %lf\n", seconds_rgb);
	//printf("size: %d\n", RGB_image.size());

        if(RGB_image.size() > 0 && seconds < 4.0 && seconds_rgb > 0.25){
            printf("nothing\n");
	}

        // Delay required to keep tx from getting cranky
        usleep(9);
        //printf("I'm afte:wqr exit delay");
            //good_read = read_it(&indata, shared, inbuffer);
            //pthread_mutex_unlock(shared->uart_mut); 

       //printf("I'm after exit delay");
    }

}

// This function should handle image data being received from drone.
// The seq variable is used to represent packet events.
// 0 = starting new image
// 1 = indexed packet of image data
// 2 = final image packet
// 3 = resent packet
void handle_image(mav_data *indata, uint8_t *inbuffer, thread_data *shared, std::vector<uint16_t> *data, uint16_t *prev){
    ack_data payload;
    uint16_t p_count = 0;
    p_count |= indata->payload[0];
    p_count |= indata->payload[1] << 8;

    // Sequence 0 means first packet containig file name and total packets.
    if(indata->seq == 0){
        prev[1] |= indata->payload[0];
        prev[1] |= indata->payload[1] << 8;
        printf("packet zero: %s, packets: %u\n", &indata->payload[2], prev[1]);
        payload.command = SEND_IMG;
        indata->msgid = ACK_MSG;
        payload.result = 0;
        payload.progress = 0;
        payload.result_param2 = 0;
        payload.target_system = SYS_ID_DRONE;
        payload.target_component = COMP_ID_CAMERA;
        send_ack(indata, &payload, shared);
	shared->flags |= SENT_START_ACK;
	data->push_back(p_count - 1); // Make Sure final packet is in the vector
    } else if(indata->seq == 2){

        // This is the final packet
	if(data->front() == p_count){
            data->erase(data->begin());
            printf("The Final Packet\n");
	} else {
            printf("vect front: %u p_count: %u", data->front(), p_count);
        }

    } else if(indata->seq == 3){
        // This is if a replacement packet is being sent
	printf("recieving missing packet");
    } else {
	shared->flags &= ~SENT_START_ACK;
        // keep track of missing or corrupt packets.
        uint16_t p_diff = p_count - prev[0];
        if(p_diff > 1){
            for(uint16_t i = *prev + 1; i < p_count; i++){
                data->push_back(i);
                printf("pushing i: %u\n",i);
            }
        }
        prev[0] = p_count;
    }
    
    // send all files recieved ack when vector empty
    if(!data->size()){
        payload.command = SEND_IMG;
        indata->msgid = ACK_MSG;
        payload.result = 0;
        payload.progress = 1;
        payload.result_param2 = 0;
        payload.target_system = SYS_ID_DRONE;
        payload.target_component = COMP_ID_CAMERA;
        send_ack(indata, &payload, shared);
	//shared->flags |= SENT_START_ACK;       
    }
    //printf("checksum 0x%02x\n", indata->checksum);
    //printf("magic: %x\n", indata->magic);
    //printf("len: %u\n", indata->len);
    //printf("incompat_flags: %u\n", indata->incompat_flags);
    //printf("compat_flags: %u\n", indata->compat_flags);
    //printf("seq: %u\n", indata->seq);
    //printf("sysid: %u\n", indata->sysid);
    //printf("compid: %u\n", indata->compid);
    //printf("msgid: %u\n", indata->msgid);
    //printf("errors: %u\n", errors);

    //printf("packet no.: %u\n", p_count);
}
