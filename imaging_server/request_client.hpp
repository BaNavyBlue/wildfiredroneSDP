
#ifndef REQUEST_CLIENT_HPP
#define REQUEST_CLIENT_HPP

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>  // For error messages
#include <signal.h>

#define LWIR    0
#define RGB_IMG 1
#define IMGS    2
#define WAY_PNT 3

#define CAPTURE 0
#define SEND    1

#define ACCEPT  0
#define DENY    1
#define COLLECT 2


typedef struct Gps{
    double lat;
    double lon;
    double height;
}Gps;

typedef struct req_msg{
    uint8_t file_type;
    uint8_t action;
    uint8_t ascii_msg[100];
    Gps gps;
}req_msg;

typedef struct srv_resp{
    uint8_t response;
    uint8_t ascii_msg[100];
    Gps gps;
    pthread_mutex_t *mutex;
}srv_resp;

void request_client(int *sockfd, const char *hostname, const unsigned int port);
int make_request(const char *hostname, int port_num, req_msg *my_msg);

#endif
