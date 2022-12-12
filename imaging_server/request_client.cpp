#include "request_client.hpp"
#include <iostream>


void request_client(int *sockfd, const char *hostname, const unsigned int port){
     // your client implementation goes here :)
    std::cout << "Greetings Program: " << hostname << " port: " << port << std::endl;
    //strcpy(thisHost, hostname);
    // Create the socket
    *sockfd = socket(PF_INET, SOCK_STREAM, 0);
    if (*sockfd < 0) perror("open");
    //struct hostent *server = gethostbyname(hostname);
    //if (server == NULL) exit(-1);

    struct sockaddr_in serv_addr;
    // Fill with zeros?
    bzero((char *) &serv_addr, sizeof(serv_addr));
    // Set Addres Format
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr("127.0.0.1"); // looking for local host
    //printf("server: %x\n", ntohl(serv_addr.sin_addr.s_addr));
    // Not sure why this mem copy needs to happen but it does.
    //memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);
    serv_addr.sin_port = htons(port);
    //memset(serv_addr.sin_zero, '\0', sizeof(serv_addr.sin_zero));
    //printf("server: %x\n", ntohl(serv_addr.sin_addr.s_addr));  
    // connect to sock
    if (connect(*sockfd,(struct sockaddr *) &serv_addr, sizeof(sockaddr)) < 0){
	std::cout << "Error connect" << std::endl;
        //exit(-1);
    }
    
    // A cute little socket for you
   
}


int make_request(const char *hostname, int port_num, req_msg *my_msg){
    int sock_fd =0;
    request_client(&sock_fd, hostname, port_num);
    if(sock_fd < 0){
        perror("socket error");
        return 1;
    }
    //printf("sock: %d\n", sock_fd);

    //req_msg my_msg;
    //my_msg.file_type = req_type;
    //my_msg.action = CAPTURE;
    //sprintf((char *)my_msg->ascii_msg, "this is just a test");
    //printf("%s\n", (char *)my_msg->ascii_msg);
    srv_resp srv_msg;

    ssize_t bytes_trans = write(sock_fd, my_msg, sizeof(req_msg));
    //printf("get past here?\n");
    if(bytes_trans < 0){
        perror("WTF");
        return 1;
    }
    
    //printf("bytes: %ld\n", bytes_trans);

    bytes_trans = read(sock_fd, &srv_msg, sizeof(srv_resp));
    if(bytes_trans < 0){
        perror("sever sucks");
        return 1;
    }

    //printf("bytes: %ld\n", bytes_trans);
    close(sock_fd);
    return 0;
}
