//*****************************************************************************
// Program to transmit a string repeatedly on the UART TX interface.
// Connect the oscilloscope to pin 8 to see the waveform.
// A. Varma, May 1, 2017
//*****************************************************************************


#include "telem_uart.h"


//#define BAUDRATE B115200 //B57600 //B115200 // UART speed
/*#define PACKAGE_SIZE 256
#define SYS_ID_DRONE 1
#define COMP_ID_CAMERA 100
#define IMG_SEND 99
#define HEADER_L 10
#define MAGIC 0xFD*/
//#define DELAY 200

//void send_file(char **argv, thread_data *shared, uint16_t comp_id, mav_data *package, uint8_t *buffer);

extern uint32_t errorz = 0;

int main (int argc, char **argv){
    signal(SIGPIPE, SIG_IGN);
    pthread_t heart_beat_thd, recv_thd;
    pthread_mutex_t uart_mut, index_mut;
    pthread_mutex_init(&uart_mut, NULL);
    pthread_mutex_init(&index_mut, NULL);
    thread_data shared;
    shared.uart_mut = &uart_mut;
    shared.index_mut = &index_mut;
    uint8_t buffer[280] = {0};
    mav_data package;
    
    // This defined in host_config.h
    // File configuraions are not shared between
    // Base Station and Drone
    #ifdef TIME_THD
        pthread_t time_thd;
        time_data my_time;
        shared.my_time = &my_time;
        pthread_create(&time_thd, NULL, time_keeper, (void*)&my_time);
    #endif

    buffer[0] = package.magic = MAGIC;
    buffer[1] = package.len = PACKAGE_SIZE - 1;
    buffer[2] = package.incompat_flags = 'b';
    buffer[3] = package.compat_flags = 'c';
    buffer[4] = package.seq = 0;
    buffer[5] = package.sysid = SYS_ID_DRONE;
    buffer[6] = package.compid = COMP_ID_CAMERA;
    buffer[7] = package.msgid = SEND_IMG;
    buffer[8] = 0;
    buffer[9] = 0;
    
    // setting up uart  
    struct termios serial; // Structure to contain UART parameters
  
    printf("Opening %s\n", DEV_ID);
    int fd = open(DEV_ID, O_RDWR | O_NOCTTY | O_NDELAY);
    // O_RDWR indicates the device needs to both written and read.
    // O_NOCTTY states that the device is not used as a console.
    // O_NDELAY indicates that read and writes should be nonblocking.
  
    if (fd == -1){ // Open failed
      perror(DEV_ID);
      return -1;
    }
    shared.fd = &fd;
    // Get UART configuration
    if (tcgetattr(fd, &serial) < 0){
      perror("Getting configuration");
      return -1;
    }
  
    pthread_create(&heart_beat_thd, NULL, heart_beat, (void*)&shared);
    pthread_create(&recv_thd, NULL, uart_recv, (void*)&shared);
    // Set UART parameters in the termios structure
    serial.c_iflag = 0;
    serial.c_oflag = 0;
    serial.c_lflag = 0;
    serial.c_cflag = BAUDRATE | CS8 | CREAD /*| PARENB | PARODD*/;
    // Speed setting + 8-bit data + Enable RX + Enable Parity + Odd Parity
    
    serial.c_cc[VMIN] = 0; // 0 for Nonblocking mode
    serial.c_cc[VTIME] = 0; // 0 for Nonblocking mode
    
    // Set the parameters by writing the configuration
    tcsetattr(fd, TCSANOW, &serial);
    
    #ifdef DRONE
        printf("we drone\n");
        // send_file(argv, &shared, COMP_ID_CAMERA, &package, buffer);
    #endif

    //int i = 0;  
    //uint8_t data[255];
    /*if(p_fd < 1){
        for(; i < PACKAGE_SIZE; i++){
            buffer[10 + i] = package.payload[i] = (i % 94) + 33;
        }
    }*/
  
  
    //int offset = i + 10;
    /*for(int j = 0; j < 13; j++){
        buffer[10 + i++] = package.signiture[j] = j + 48;
    }*/
    //package.payload = data;
  
    
  
    for(;;){
        uart_server(8002, &shared, &package, buffer);
        //sleep(1);
    }
  
    close(fd);
}



void send_file(const char *f_name, thread_data *shared, uint16_t comp_id, mav_data *package, uint8_t *buffer){

    //uint8_t buffer[280] = {0};
    // open file for sending
    int p_fd = 0;
    if((p_fd = open(f_name, O_RDONLY)) < 1){
        perror("input file failed to open.");
        //exit(1);
    } else {
        shared->flags |= WAIT_FOR_FINISHED_ACK;
    }
    // look for size of file
    off_t f_size = lseek(p_fd, 0L, SEEK_END);
    lseek(p_fd, 0L, SEEK_SET);
    char mod_name[100];
    uint8_t o_set = 0;
    
    // Strip of any path information from the file
    for(int i = 0; i < strlen(f_name) + 1; i++){
        if(f_name[i] == '/'){
            o_set = (uint8_t)i;
        } else if((f_name[i] < 45 && f_name[i] > 125)){
            o_set = (uint8_t)i;
        } else if(f_name[i] == 0){
            strcpy(mod_name, &f_name[o_set + 1]);
        }
    }
    printf("mod_name: %s\n", mod_name);
    buffer[10] = 0;
    buffer[10] = 0;
    // find packets needed and prepare packet zero
    // to signal recieving server.
    uint16_t packets = 0;

    // If able to open file
    if(p_fd){
        packets = (f_size / 253) + 1;
        buffer[1] = strlen(mod_name) + 3; // filename plus newline plus 2 bytes
        buffer[10] = packets;
        buffer[11] = packets >> 8;
        strcpy((char*)&buffer[12], mod_name);
        uint16_t checksum = crc_calculate(&buffer[1], HEADER_L - 1);
        crc_accumulate_buffer(&checksum, (const char*)&buffer[10], buffer[1]);
        crc_accumulate(0, &checksum); 
        package->checksum = checksum;
        printf("checksum: 0x%x\n", checksum);
        buffer[10 + buffer[1]] = package->checksum;
        buffer[10 + buffer[1] + 1] = package->checksum >> 8; 
        write_packet(buffer, shared);
    }

    printf("packets to send: %u\n", packets);
    // Now transmit the data until receive finished ACK
    //printf("sizeof package: %u\n", sizeof(package));
    uint16_t packet_cnt = 0;
    uint16_t timeout = 0;
    while ((packet_cnt < packets - 1) || shared->flags & WAIT_FOR_FINISHED_ACK){
        // Wait for first packet acknowlegement
        if(!(shared->flags & RECEIVED_START_ACK)){
            usleep(50000);
            /*if(timeout > 250){
                write_packet(buffer, shared);
                timeout = 0;
            } else {
                timeout += 50;
            }*/
            //printf("waiting for RECEIVED_START_ACK\n");
        } else if(shared->flags & REPLACE_PACKET_FLAG){
            shared->flags &= ~REPLACE_PACKET_FLAG;
            if(!(shared->flags & SENT_FINAL_PACKET)){
                shared->replace_pckt = 0xFFFF;
            }
            //uint16_t index = 0;
            //index |= package->payload[13]; 
            //index |= package->payload[14] << 8;
            timeout = 0;
            // if receive request to replace and last packet has been sent
            if(shared->replace_pckt < 0xFFFF){
                package->payload[0] = shared->replace_pckt;
                package->payload[1] = shared->replace_pckt >> 8;
                printf("index: %u\n", shared->replace_pckt);
                ssize_t bytes_read = pread(p_fd, &package->payload[2], 253, shared->replace_pckt * 253);
    
                if(bytes_read < 0){
                    perror("resend packet: error reading file");
                    package->len = 2;
                    buffer[1] = 2;
                } else {
                    package->len = bytes_read + 2;
                    buffer[1] = package->len;
                }
    
                buffer[4] = 3; // resending seq
                buffer[7] = SEND_IMG;
    
                int i;
                for(i = 0; i < buffer[1]; i++){
                    buffer[10 + i] = package->payload[i];
                }
    
                //printf("pack len: %u\n", package->len);
                uint16_t checksum = crc_calculate(&buffer[1], HEADER_L - 1);
                crc_accumulate_buffer(&checksum, (const char*)package->payload, buffer[1]);
                crc_accumulate(0, &checksum);
                package->checksum = checksum;
                //printf("checksum: 0x%x\n", checksum);
                buffer[10 + i] = package->checksum;
                buffer[10 + i + 1] = package->checksum >> 8;
    
                write_packet(buffer, shared);
                shared->replace_pckt = 0xFFFF;
            }
        } else if(!(shared->flags & SENT_FINAL_PACKET) && shared->flags & HEART_BEAT_PRESENT){
            timeout = 0;
            ssize_t bytes_read = read(p_fd, &package->payload[2], 253);
            if(bytes_read < 0){
                perror("problem reading from file");
                buffer[1] = 2;
                package->len = 2;
            } else {
                buffer[1] = bytes_read + 2;
                package->len = buffer[1];
            }
            //printf("bytes_read: %d\n", bytes_read);
            //printf("packet_cnt: %u\n", packet_cnt);

            // Check to see if last packet
            // Using seq variable to determine
            // whether normal packet 1 or final packet 2
            if(packet_cnt == packets - 1){
                buffer[4] = 2;
                printf("last packet\n");
                shared->flags |= SENT_FINAL_PACKET;               
            } else {
                buffer[4] = 1;
            }

            //printf("packet_cnt: %u\n", packet_cnt);
            package->payload[0] = packet_cnt;
            package->payload[1] = packet_cnt >> 8;
      
            int i;
            for(i = 0; i < buffer[1]; i++){
                buffer[10 + i] = package->payload[i]; 
            }
         
            //printf("pack len: %u\n", package->len);
            uint16_t checksum = crc_calculate(&buffer[1], HEADER_L - 1);
            crc_accumulate_buffer(&checksum, (const char*)package->payload, buffer[1]);
            crc_accumulate(0, &checksum); 
            package->checksum = checksum;
            //printf("checksum: 0x%x\n", checksum);
            buffer[10 + i] = package->checksum;
            buffer[10 + i + 1] = package->checksum >> 8;
    
            write_packet(buffer, shared);
    
            packet_cnt++;

        } else {
            usleep(100000);
            timeout += 100;
            printf("after sleep\n");
            if(timeout > 30000){
                printf("30 seconds of no packet replacement requests exiting\n");
                break;
            }
            //printf("packet_cnt: %u, packets: %u\n", packet_cnt, packets);
        }
        //usleep(DELAY);
    }
    // Clear Ack Flag and Close File
    shared->flags &= ~(RECEIVED_START_ACK | SENT_FINAL_PACKET) ;
    printf("Closing source file.\n");
    //printf("shared->flags: %x ,R_S_A: %x\n", shared->flags, RECEIVED_START_ACK);
    
    if(close(p_fd)){
        perror("jesus this file is uncloseable");
    }
    usleep(250000);
    buffer[4] = 0;
}

// this function handles crc_checks returns 0 for fail 1 for pass
uint8_t crc_check(mav_data *indata, uint8_t *inbuffer, uint8_t extra_crc){

    uint16_t checksum = crc_calculate(&inbuffer[1], HEADER_L - 1);
    crc_accumulate_buffer(&checksum, (const char*)&inbuffer[10], inbuffer[1]);
    crc_accumulate(extra_crc, &checksum);

    //printf("checksum: 0x%x indata: %x\n", checksum, indata->checksum);
    if(indata->checksum == checksum){
        //printf("Passed CRC check\n");
        return 1;
    } else {
        printf("Failed CRC check\n");
        errorz++;
    }

    return 0;
}
