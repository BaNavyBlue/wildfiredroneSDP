// Lets add some important messages to bring to your attention.

#include "uart_recv.h"

void handle_image(mav_data *indata, uint8_t *inbuffer, thread_data *shared, std::vector<uint16_t> *data, uint16_t *prev);
void send_closing_file_ack(thread_data *shared, mav_data *indata);
                        


extern uint32_t errors;

// These are for recovering missing or bad crc data packets.
static uint16_t total_packets = 0;
static uint16_t packets_received = 0;
static uint16_t max_missing_packets = 0;
int recv_fd = 0;

void *uart_recv(void *data){
    thread_data *shared = (thread_data*)data;
    mav_data indata;
    uint8_t inbuffer[280];
    char boggle = ZERO;
    // Sit and spin
    unsigned int beats = 0;

    #ifdef TIME_THD
        time_t t = time(NULL);
        struct tm *tm = localtime(&t);

        char file_d_and_time[100];
        assert(strftime(file_d_and_time, sizeof(file_d_and_time), "%c", tm));

        //sprintf(file_d_and_time, "%s_%s", __TIME__, __DATE__);

       for(int i = 0; i < 100; i++){
           if(file_d_and_time[i] == '\0'){
               break;
           } else if(file_d_and_time[i] == ':'){
               file_d_and_time[i] = '-';
           } else if(file_d_and_time[i] == ' '){
               file_d_and_time[i] = '_';
           }
       }   

       char filename[100];
       char str_buf[300];
       int data_fd = 0;

       sprintf(filename, "data/%s_data_rate.csv", file_d_and_time);
       if(!(data_fd = open(filename, O_CREAT | O_TRUNC | O_RDWR, S_IRUSR | S_IWUSR))){
           perror("Couldn't open or create file");
       } else {
           printf("%s created.\n", filename);
       }
       sprintf(str_buf, "!%s you dingus\n", filename);
       ssize_t bytes_writen = write(data_fd, str_buf, strlen(str_buf));
       if(bytes_writen < 0){
           perror("failed to write to file");
       }

       sprintf(str_buf, "Sys_clk, t_elapsed(ms), sec, Data Rate (B/s)\n");
       bytes_writen = write(data_fd, str_buf, strlen(str_buf));
       if(bytes_writen < 0){
           perror("failed to write to file");
       }
    #endif

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
    shared->recv_fd = &RGB_fd;
 
    int rx_count = ZERO;
    ssize_t read_bytes;
    uint16_t final_form = ZERO;
    uint64_t prev_rgb_pkt = ZERO;
    long double seconds_rgb = 0.0;
    long double packets_accum = 0.0;
    uint8_t time_avg = 0; 
    long double seconds_accum = 0.0;    

    // Sit and spin
    for(;;){
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t now = (uint64_t)ts.tv_sec * 1000000000u + (uint64_t)ts.tv_nsec;
        //pthread_mutex_lock(shared->uart_mut);
        read_bytes = read(*shared->fd, &inbuffer[rx_count], 1/*bytes_total - rx_count*/);
   
        if(read_bytes < 0){
             printf("Error Bit\n");
             perror("read");
             //close(fd);
             //return -1;
        }
   
        if(read_bytes > 0 && rx_count == 0){
            if(inbuffer[0] == MAGIC){
                rx_count += read_bytes;
            }
            /*if(inbuffer[0] == MAVGIC){
                printf("telemetry radio trying to send message\n");
                rx_count += read_bytes;
            }*/
        } else if(rx_count > 0 && read_bytes > 0){
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
        } /*else if(rx_count > 5 && !boggle && inbuffer[0] == MAVGIC){
            indata.magic = inbuffer[0];
            indata.len = inbuffer[1];
            printf("indata.len: %u\n", indata.len);
            indata.seq = inbuffer[2];
            indata.sysid = inbuffer[3];
            indata.compid = inbuffer[4];
            indata.msgid = inbuffer[5];
            printf("indata.msgid: %u\n", indata.msgid);
            final_form = 8 + indata.len;
        }*/
        if(rx_count == final_form && final_form){
            int i;
            uint8_t off_set = 10;

            if(indata.magic == MAVGIC){
                off_set = 6;
            }
   
            for(i = 0; i < indata.len; i++){
                indata.payload[i] = inbuffer[i + off_set];
            }
   
            indata.checksum = 0;
            indata.checksum |= inbuffer[off_set + i++];
            indata.checksum |= inbuffer[off_set + i++] << 8;
   
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

                        if(crc_check(&indata, inbuffer, extra_crc)){
                            shared->flags |= HEART_BEAT_PRESENT;
                            clock_gettime(CLOCK_MONOTONIC, &ts);
                            last_hb = (uint64_t)ts.tv_sec * 1000000000U + (uint64_t)ts.tv_nsec;
                        }
                        //printf("payload: 0x");
                        /*for(i = 0; i < indata.len; i++){
                            printf("%02x",indata.payload[i]);
                        }*/
                        //printf("\n");
                        break;
                    }
                case SEND_IMG:
                    {
                        extra_crc = SEND_IMG_CRC;
                        if(crc_check(&indata, inbuffer, extra_crc)){
                                handle_image(&indata, inbuffer, shared, &RGB_image, RGB);
			    //if(inbuffer[4] == 0){
			    //	clock_gettime(CLOCK_MONOTONIC, &ts);
                            //    last_rgb_pckt = (uint64_t)ts.tv_sec * 1000000000U + (uint64_t)ts.tv_nsec;    
			    //} else if(inbuffer[4] > 0){
                                clock_gettime(CLOCK_MONOTONIC, &ts);
                                last_rgb_pckt = (uint64_t)ts.tv_sec * 1000000000U + (uint64_t)ts.tv_nsec;

                                seconds_accum += ((long double) (last_rgb_pckt - prev_rgb_pkt)) / 1000000000U;
                                packets_accum += ((long double)(12 + indata.len));

                                if(time_avg == 19){
                                    printf("seconds for %u packets: %lf ", time_avg + 1, seconds_accum);
                                    long double throughput = (packets_accum) / (seconds_accum);
                                    printf("bytes per second: %lf\n", throughput);
                                    #ifdef TIME_THD
                                        sprintf(str_buf, "%s, %lf, %lf, %lf\n", shared->my_time->clock,
                                                shared->my_time->t_ms, seconds_accum, throughput);
                                        bytes_writen = write(data_fd, str_buf, strlen(str_buf));
                                        if(bytes_writen < 0){
                                            perror("failed to write to file");
                                        }
                                    #endif
                                    packets_accum = seconds_accum = 0.0;
                                }
                                time_avg = (time_avg + 1) % 20;
				prev_rgb_pkt = last_rgb_pckt;
			    //}
                        } else {
                            #ifdef TIME_THD
                                sprintf(str_buf, "%s, %lf, %lf, %lf\n", shared->my_time->clock,
                                        shared->my_time->t_ms, 3.33, 0.0);
                                bytes_writen = write(data_fd, str_buf, strlen(str_buf));
                                if(bytes_writen < 0){
                                    perror("failed to write to file");
                                }
                            #endif

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
			    if(indata.seq == 0){
                                shared->flags |= RECEIVED_START_ACK;
			    } else if(indata.seq == 3){
                                printf("requesting packet\n");
                                shared->replace_pckt = 0;
				shared->replace_pckt |= indata.payload[0];
				shared->replace_pckt |= indata.payload[1] << 8;
                                shared->flags |= REPLACE_PACKET_FLAG;
                                printf("index: %u\n", shared->replace_pckt);
			    } else if(indata.seq == 4){
                                printf("Finished Transfer Ack\n");
                                if(shared->flags & WAIT_FOR_FINISHED_ACK){
                                    shared->flags &= ~WAIT_FOR_FINISHED_ACK;
                                    send_closing_file_ack(shared, &indata);
                                }
                            }
                            break;
			}
                    }
                default:
                    {
                        printf("I have no idea what this msg is!\n");
                        printf("msgid: %u\n", indata.msgid);
                        crc_check(&indata, inbuffer, extra_crc);
                        #ifdef TIME_THD
                            sprintf(str_buf, "%s, %lf, %lf, %lf\n", shared->my_time->clock,
                                    shared->my_time->t_ms, 6.66, 0.0);
                                    bytes_writen = write(data_fd, str_buf, strlen(str_buf));
                            if(bytes_writen < 0){
                                perror("failed to write to file");
                            }
                        #endif
                        break;
                    }
            }


     	    //  Incase ACK packet corrupted resend ACK 
    	    if(indata.msgid != SEND_IMG && shared->flags & SENT_START_ACK && seconds_rgb > 0.2){
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
                    printf("sending start ack\n");
    	    }  
            rx_count = 0;
            //bytes_total = MAX_PACKET;
        }
        
        // Keep track of heart beat
        if((now > last_hb)){
            t_btwn_hb = now - last_hb;
        } else {
            t_btwn_hb = 0;
	}

        // Keep track of last rgb file packet
	if((now > last_rgb_pckt)){
            t_btwn_rgb = now - last_rgb_pckt;
	} else {
            t_btwn_rgb = 0;
	}

        long double seconds = ((long double) (t_btwn_hb)) / 1000000000U;
        seconds_rgb = ((long double) (t_btwn_rgb)) / 1000000000U;
        //printf("seconds: %lf\n", seconds_rgb);
	//printf("size: %d\n", RGB_image.size());
        if(seconds > 4.0){
            shared->flags &= ~HEART_BEAT_PRESENT;
        }
        // For requesting replacement packets.
        if(RGB_image.size() > 0 && seconds < 4.0 && seconds_rgb > 0.35 && shared->flags & RECEIVING_FILE){
            printf("nothing gib uss missing packets\n");
            indata.seq = 3;
    	    ack_data payload;
            printf("what u want: %u\n",RGB_image.back());
            payload.command = RGB_image.back();
            //indata.msgid = ACK_MSG;
            payload.result = 0;
            payload.progress = 0;
            payload.result_param2 = 0;
            payload.target_system = SYS_ID_DRONE;
            payload.target_component = COMP_ID_CAMERA;
            send_ack(&indata, &payload, shared);
            clock_gettime(CLOCK_MONOTONIC, &ts);
	    last_rgb_pckt = (uint64_t)ts.tv_sec * 1000000000U + (uint64_t)ts.tv_nsec;    
	}

        // Delay required to keep tx from getting cranky
        usleep(43);
	//printf("I'm after exit delay");
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
        total_packets |= indata->payload[0];
        total_packets |= indata->payload[1] << 8;
        prev[1] = total_packets;
        prev[0] = 0xFFFF; // Incase packet index 0 is missing makes sure gets popped onto stack.
        printf("filename: %s, packets: %u\n", &indata->payload[2], prev[1]);
        if(*shared->recv_fd){
            printf("File not closed possible error, closing and opening new file\n");
            close(*shared->recv_fd);
        }
        if(!(*shared->recv_fd = open((char*)&indata->payload[2]
                                      , O_CREAT | O_TRUNC | O_RDWR, S_IRUSR | S_IWUSR))){
            perror("Couldn't open or create file");
        } else {
            printf("fd: %d\n", *shared->recv_fd);
        }

        payload.command = SEND_IMG;
        indata->msgid = ACK_MSG;
        payload.result = 0;
        payload.progress = 0;
        payload.result_param2 = 0;
        payload.target_system = SYS_ID_DRONE;
        payload.target_component = COMP_ID_CAMERA;
        send_ack(indata, &payload, shared);
	shared->flags |= SENT_START_ACK | RECEIVING_FILE;
	data->push_back(p_count - 1); // Make Sure final packet is in the vector
        max_missing_packets = 0;
        packets_received = 0;
        max_missing_packets++;
    } else if(indata->seq == 2){
        if(shared->flags & SENT_START_ACK){
            shared->flags &= ~SENT_START_ACK;
        }
        // This is the final packet
	if(data->front() == p_count){
            data->erase(data->begin());
            max_missing_packets--;
            ssize_t bytes_writen = pwrite(*shared->recv_fd, &indata->payload[2], indata->len - 2, p_count * 253);
            if(bytes_writen < 0){
                perror("failed to write data");
            }
            // edge case gap between 
            // keep track of missing or corrupt packets.
            int16_t p_diff = p_count - prev[0];
            if(p_diff > 1){
                for(uint16_t i = *prev + 1; i < p_count; i++){
                    data->push_back(i);
                    max_missing_packets++;
                    printf("pushing i: %u\n",i);
                }
            }
        prev[0] = p_count;
            packets_received++;
            printf("The Final Packet\n");
	} else {
            printf("vect front: %u p_count: %u", data->front(), p_count);
        }

	// check to see if all of file is receive
        if(!data->size() && packets_received >= total_packets){
             printf("All done\n");
             printf("Total recoverd packets: %u\n", max_missing_packets);
             printf("Percentage missing packets: %lf%\n"
                    , ((double)(max_missing_packets)) / ((double)(total_packets)) * 100.0);
             packets_received = 0;
             total_packets = 0;
             max_missing_packets = 0;
	     shared->flags &= ~RECEIVING_FILE;
             shared->flags |= WAIT_FOR_FINISHED_ACK;
             send_closing_file_ack(shared, indata);
	}

    } else if(indata->seq == 3) {
        // This is if a replacement packet is being sent
	printf("recieving missing packet: %u\n", p_count);
        // keep track of missing or corrupt packets.
        // Handle the edgecase where final packet is sent
        // because of time out problem.
        int16_t p_diff = p_count - prev[0];
        if(!(total_packets - 1 == p_count)){
            if(p_diff > 1){
                printf("EdgeCase Here\n");
                for(uint16_t i = *prev + 1; i < p_count; i++){
                    data->push_back(i);
                    max_missing_packets++;
                    printf("pushing i: %u\n",i);
                }
            }
            prev[0] = p_count;
        }
        
        uint16_t before_size = data->size();
	for(uint16_t i = 0; i < data->size(); i++){
            //printf("data->at: %u\n",data->at(data->size() - i - 1));
            
            if(p_count == data->at(data->size() - i - 1)){
		printf("size: %d\n", data->size());
		uint16_t idx = data->size() - i - 1;
                data->erase(data->end() - 1 - i);
                packets_received++;

                ssize_t bytes_writen = pwrite(*shared->recv_fd
                                              , &indata->payload[2], indata->len - 2, p_count * 253);
                if(bytes_writen < 0){
                    perror("failed to write data");
                }
               
		printf("size after: %d\n", data->size());
	    } 
	}

        if(data->size() == before_size){
            printf("p_count: %d not found: pushing\n", p_count);
            data->push_back((uint16_t)p_count);
            max_missing_packets++;
        }

        printf("packets received: %u\n", packets_received);
	// check to see if all of file is received
        if(!data->size() && packets_received >= total_packets){
             printf("All done\n");
             printf("Total recoverd packets: %u\n", max_missing_packets);
             printf("Percentage missing packets: %lf%\n"
                    , ((double)(max_missing_packets)) / ((double)(total_packets)) * 100.0);
             packets_received = 0;
             total_packets = 0;
             max_missing_packets = 0;
             shared->flags |= WAIT_FOR_FINISHED_ACK;
             send_closing_file_ack(shared, indata);
	     shared->flags &= ~RECEIVING_FILE;
	}

    } else {
        //if(shared->flags & SENT_START_ACK){
	    shared->flags &= ~SENT_START_ACK;
        //}
        // keep track of missing or corrupt packets.
        //printf("Handling packet: %d\n", p_count);
        int16_t p_diff = p_count - prev[0];
        if(p_diff > 1){
            for(uint16_t i = *prev + 1; i < p_count; i++){
                data->push_back(i);
                max_missing_packets++;
                printf("pushing i: %u\n",i);
            }
        }
        prev[0] = p_count;

        ssize_t bytes_writen = pwrite(*shared->recv_fd, &indata->payload[2], 253, p_count * 253);
        if(bytes_writen < 0){
            perror("failed to write data");
        }

        packets_received++;
    }
    
    // send all files received ack when vectorf empty
    /*if(!data->size()){
        payload.command = SEND_IMG;
        indata->msgid = ACK_MSG;
        payload.result = 0;
        payload.progress = 1;
        payload.result_param2 = 0;
        payload.target_system = SYS_ID_DRONE;
        payload.target_component = COMP_ID_CAMERA;
        send_ack(indata, &payload, shared);
	//shared->flags |= SENT_START_ACK;
        shared       
    }*/
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

void send_closing_file_ack(thread_data *shared, mav_data *indata){
    ack_data payload;
   //shared->flags |= WAIT_FOR_FINISHED_ACK;
    indata->seq = 4;
    payload.result = 0;
    payload.progress = 0;
    payload.result_param2 = 0;
    payload.target_system = SYS_ID_DRONE;
    payload.target_component = COMP_ID_CAMERA;
    send_ack(indata, &payload, shared);
    if(*shared->recv_fd){
        if(close(*shared->recv_fd)){
            perror("unable to close file SHIT!!!");
        } else {
            *shared->recv_fd = 0;
        }
    }
}
