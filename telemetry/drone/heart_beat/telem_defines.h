#ifndef TELEM_DEFINES_H
#define TELEM_DEFINES_H

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <mavlink.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <stdint.h>
#include <checksum.h>
#include <pthread.h>

#define BUFFER_SIZE 64
#define ZERO 0           // Guess
#define BYTES_P_TRANS 1  // Currently byte by byte
#define DELAY 160       // Tx Delay
#define MAX_PACKET 280 // Max Mavlink Packet
#define BASE_SIZE 12 // Min package size with no payload
#define MAGIC 0xFD
#define HEADER_L 10


enum MSG_ID{
    HEARTBEAT,
    ACK_MSG = 77,
    SEND_IMG = 99,
    RAD_MSG = 109
};

enum EXT_CRC{
    SEND_IMG_CRC = 0,
    H_BEAT_CRC = 50,
    ACK_CRC = 143
};

typedef struct {
    uint8_t magic; // Packet Start
    uint8_t len; // Payload length
    uint8_t incompat_flags; // Incompatibility Flags
    uint8_t compat_flags;  // Compatibility Flags
    uint8_t seq; // Packet sequence number 0-255 , also used for start stop and resent signaling:
    uint8_t sysid; // System ID (Sender) 1-255
    uint8_t compid; // component ID (Sender) 1_255
    uint32_t msgid:24; // Message ID (low, middle, high, bytes)
    uint8_t payload[255]; // PTR to message data 255 bytes at a time max
    uint16_t checksum;
    uint8_t signiture[13];
}mav_data;

// Thread Flags
#define RECEIVED_START_ACK 0x0001 // Ready to start sending file payload
#define SENT_START_ACK 0x0002 // Waiting for payload packets to start
#define WAIT_FOR_FINISHED_ACK 0x0004 // Will not end sending function until ACK
#define REPLACE_PACKET_FLAG 0x0008 // Send Replacement packet
#define SENT_FINAL_PACKET 0x0010 // We sent final packet in file.
#define RECIEVED_FINAL_PACKET 0x0020 // Makesure the final packet has been recieved

typedef struct {
    pthread_mutex_t *uart_mut;
    int *fd;
    uint16_t flags = 0;
}thread_data;

#include "heart_beat.h"
#include "telem_uart.h"
#include "uart_recv.h"
#include "write_helper.h"
#include "host_config.h"

#endif
