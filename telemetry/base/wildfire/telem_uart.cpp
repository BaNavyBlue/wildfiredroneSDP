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

//void send_image(char **argv, thread_data *shared, uint16_t comp_id, mav_data *package, uint8_t *buffer);

extern uint32_t errors = 0;

int main (int argc, char **argv){
    pthread_t heart_beat_thd, recv_thd;
    pthread_mutex_t uart_mut;
    pthread_mutex_init(&uart_mut, NULL);
    thread_data shared;
    shared.uart_mut = &uart_mut;
    uint8_t buffer[280] = {0};
    mav_data package;
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

    //send_image(argv, &shared, COMP_ID_CAMERA, &package, buffer);

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
        sleep(1);
    }
  
    close(fd);
}



void send_image(char **argv, thread_data *shared, uint16_t comp_id, mav_data *package, uint8_t *buffer){

    //uint8_t buffer[280] = {0};
    int p_fd = 0;
    if((p_fd = open(argv[1], O_RDONLY)) < 1){
        perror("input file failed to open.");
        //exit(1);
    }
  
    off_t f_size = lseek(p_fd, 0L, SEEK_END);
    lseek(p_fd, 0L, SEEK_SET);
  

    uint16_t packets = 0;
    if(p_fd){
        packets = f_size / 253;
    }
    printf("packets to send: %u\n", packets);
    // Now transmit the data in an infinite loop
    //printf("sizeof package: %u\n", sizeof(package));
    uint16_t packet_cnt = 0;
    while (packet_cnt < packets + 1){
        ssize_t bytes_read = read(p_fd, &package->payload[2], 253);
        //printf("bytes_read: %d\n", bytes_read);
        printf("packet_cnt: %u\n", packet_cnt);
        package->payload[0] = packet_cnt;
        package->payload[1] = packet_cnt >> 8;
  
        int i;
        for(i = 0; i < bytes_read + 2; i++){
            buffer[10 + i] = package->payload[i]; 
        }
     
        buffer[1] = bytes_read + 2;
        package->len = buffer[1];
        printf("pack len: %u\n", package->len);
        uint16_t checksum = crc_calculate(&buffer[1], HEADER_L - 1);
        crc_accumulate_buffer(&checksum, (const char*)package->payload, bytes_read + 2);
        crc_accumulate(0, &checksum); 
        package->checksum = checksum;
        printf("checksum: 0x%x\n", checksum);
        buffer[10 + i] = package->checksum;
        buffer[10 + i + 1] = package->checksum >> 8;
        write_packet(buffer, shared);

        packet_cnt++;
        //usleep(DELAY);
    }
    close(p_fd);
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
        errors++;
    }

    return 0;
}
