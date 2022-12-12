


#include "request_client.hpp"

int main(int argc, char** argv){

    req_msg my_msg;
    my_msg.file_type = LWIR;
    my_msg.action = CAPTURE;
    my_msg.gps.lat = 1.1;
    my_msg.gps.lon = 2.2;
    my_msg.gps.height = 3.3;
    sprintf((char *)my_msg.ascii_msg, "ImageCapture_0");
    char srvr_msg[100];
    int counter = 1;
    while(1){
        if(make_request(argv[1], atoi(argv[2]), &my_msg)){
            perror("God is dead");
            return 1;
        } else {
            printf("%s\n", (char *)my_msg.ascii_msg);
        }
        counter++;
        sprintf((char *)my_msg.ascii_msg, "%s_%d", "ImageCapture", counter);
            sleep(20);
    }


    return 0;
}
