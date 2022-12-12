//#include <gst/gst.h>
//#include <stdio.h>
/*
// What level of sadness do you want ot engage in.
#define LWIR_SAD "v4l2src device=/dev/video0 video/x-raw, format=(string)GRAY16_LE!, width=(int)160, height=(int)120 ! videoconvert! pngenc! filesink"
#define RGB_SAD  "v4l2src num-buffers=1 ! jpegenc ! filesink location=./test.jpg"*/

#include "gst_capture.h"

int main(int argc, char **argv)
{
    req_msg data;
    //char lwir_command[200];
    sprintf((char*)data.ascii_msg, "testfile");
    printf("command: %s\n", data.ascii_msg);
    gst_capture(&data);
    //gst_capture(argc, argv, RGB_SAD); 
    return 0;
}

