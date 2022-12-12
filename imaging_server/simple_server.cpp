#include "simple_server.hpp"
#include "request_client.hpp"

// This is really just a template to base other simple servers off of
// a stripped down version of my HTTP server from CSE130

int simple_server(int port, srv_resp *message_passing){

    // Set up socket address struct
    struct sockaddr_in addr;

    // The message buffer
    unsigned char buffer[96];

    sprintf((char*)buffer, "Starting %s at %d using port %d\n", "nothing", 0
            , port);
    write(1, buffer, strlen((char*)buffer));

    //struct hostent *hent = gethostbyname(argv[1]);
    struct sockaddr_storage serverStorage;
    socklen_t addr_size;

    addr.sin_addr.s_addr = inet_addr("127.0.0.1"); // looking for local host
    addr.sin_port = htons(port);
    addr.sin_family = AF_INET;
    printf("Port: %u %u\n", addr.sin_port, ntohs(addr.sin_port));

    // Creating a Socket
    int srv_fd = socket(AF_INET, SOCK_STREAM, 0);

    //Socket Setup for Server
    int enable = 1;
    setsockopt(srv_fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
    bind(srv_fd, (struct sockaddr *)&addr, sizeof(addr));

    if(!listen(srv_fd, 3)){
        printf("listening\n");
    } else {
        printf("after listen\n");
    }
    printf("after listen\n");

    while(1){
        // This counting semaphore will block if all threads
        // are active
        //sem_wait(&S);
        req_msg the_msg;
        srv_resp the_resp;
        printf("Before accept\n");

        addr_size = sizeof(serverStorage);
        int sockfd = accept(srv_fd, (struct sockaddr*)&serverStorage, &addr_size);
        printf("after accept\n");
        /* Code bellow should be changed for specific server being used */
        ssize_t n = recv(sockfd, (void*)&the_msg, sizeof(req_msg), 0);
        printf("we recv: %u %u %s\n", the_msg.file_type, the_msg.action, (char *)the_msg.ascii_msg);
        if(n < 0){
            fprintf(stderr, "socket error what do now?\n");
        }
        
        /* Pass File Name By Pointer and unlock Mutex via server parameter */
        strcpy((char*)message_passing->ascii_msg, (char*)the_msg.ascii_msg);
        message_passing->gps = the_msg.gps;
        //message_passing->gps->lon = the_msg.gps.lon;
        //message_passing->gps->height = the_msg.gps.height;
        pthread_mutex_unlock(message_passing->mutex);

        if(the_msg.file_type == LWIR){
            //printf("You did it LWIR\n");
            the_resp.response = ACCEPT;
            n = send(sockfd, (void*)&the_resp, sizeof(srv_resp), 0);
            //sprintf((char *)the_msg.ascii_msg,"../imaging_server/cupOsorrow.png");
            if(!make_request("host", 8002, &the_msg)){
                perror("server not available");
            }
            n = recv(sockfd, (void*)&the_resp, sizeof(srv_resp), 0);
            //sprintf((char *)the_msg.ascii_msg,"../imaging_server/cupOsorrow.png");
            if(n < 0){
                perror("socket_error");
            } else {
                printf("finished sending image\n");
            }
        }

        close(sockfd);
    }
    return 0;
}
