//gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw, format=GRAY16_LE ! videoscale ! video/x-raw,width=160, height=120! videoconvert ! pngenc ! filesink location="adumbbar.png"

#ifndef GST_CAPTURE_HPP
#define GST_CAPTURE_HPP

#include <gst/gst.h>
#include <stdio.h>
#include "request_client.hpp"

// What level of sadness do you want ot engage in.
//#define LWIR_SAD "v4l2src device=/dev/video1 ! video/x-raw,format=GRAY16_LE ! videoscale ! video/x-raw,width=160,height=120!  videoconvert !  pngenc ! multifilesink location=%d.png"
#define LWIR_SAD "v4l2src device=/dev/video0 ! video/x-raw,format=GRAY16_LE ! videoscale ! video/x-raw,width=160,height=120 ! videoconvert ! pngenc ! multifilesink location="
//#define LWIR_SAD "v4l2src device=/dev/video0 ! video/x-raw, format=(string)GRAY16_LE!, width=(int)160, height=(int)120 ! videoconvert! pngenc! filesink location=./bacon.png"
//#define RGB_SAD  "v4l2src device=/dev/video2 ! num-buffers=1 ! jpegenc ! filesink location=./test.jpg"
#define RGB_SAD  "v4l2src device=/dev/video12 ! num-buffers=1 ! jpegenc ! filesink location=./test.jpg"


int gst_capture(req_msg *data);



#endif
