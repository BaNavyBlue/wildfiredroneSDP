/*#include <gst/gst.h>
#include <stdio.h>

// What level of sadness do you want ot engage in.
#define LWIR_SAD "v4l2src device=/dev/video0 video/x-raw, format=(string)GRAY16_LE!, width=(int)160, height=(int)120 ! videoconvert! pngenc! filesink"
#define RGB_SAD  "v4l2src num-buffers=1 ! jpegenc ! filesink location=./test.jpg"*/

#include "gst_capture.h"
#define TIMEOUT 2500000000

int gst_capture(req_msg *data)
{
    GstElement *pipeline;//Users/janellechen/Desktop/sad.c
    GstBus *bus;
    GstMessage *msg;
    const gchar *nano_str;
    guint major, minor, micro, nano;
    char file_name[200];
    sprintf(file_name, "LWIR%s.png", (char*)data->ascii_msg);
    char camera[200];
    sprintf(camera, "%s%s", LWIR_SAD, file_name);    
    printf("lat: %lf, lon: %lf, height: %lf\n", data->gps.lat, data->gps.lon, data->gps.height);
    printf(camera);
    /* Initialize GStreamer */
    gst_init (NULL, NULL);
    gst_version (&major, &minor, &micro, &nano);
    printf ("\nGStreamer Version: %d.%d.%d\n",major, minor, micro);

    /* Build the pipeline */
    // This function parses command line like string to launch
    // gstreamer pipeline
    pipeline = gst_parse_launch(camera, NULL); 
    printf("After Get Pipeline\n");
    /* Start playing */
    gst_element_set_state (pipeline, GST_STATE_PLAYING);
    printf("After GST_STATE_PLAYING\n");
    /* Wait until error or EOS */
    bus = gst_element_get_bus (pipeline);
    // Time out set to 2.5sec
    msg = gst_bus_timed_pop_filtered (bus, TIMEOUT,
                                      (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    //msg = gst_bus_timed_pop_filtered (bus, GST_CLOCK_TIME_NONE,
    //                                  (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

    /* Free resources */
    if (msg != NULL){
        gst_message_unref (msg);
    }

    gst_object_unref (bus);
    gst_element_set_state (pipeline, GST_STATE_NULL);
    gst_object_unref (pipeline);

    // Check to see if file exists
    int fd = 0;

    if((fd = open(file_name, O_RDWR)) < 0){
        printf("%s: not created\n", file_name);
    } else {

        // look for size of file
        off_t f_size = lseek(fd, 0L, SEEK_END);
        printf("file size: %ld\n", f_size);
        lseek(fd, 0L, SEEK_SET);
        // Append GPS data to end of image file.
        ssize_t bytes_written = pwrite(fd, &data->gps, sizeof(Gps), f_size);
        f_size = lseek(fd, 0L, SEEK_END);
        printf("new file size: %ld\n", f_size);
        close(fd);
        char cwd[100];
        if (getcwd(cwd, sizeof(cwd)) != NULL) {
           //printf("Current working dir: %s\n", cwd);
        } else {
            perror("getcwd() error");
            //return 1;
        }
        printf("cwd: %s\n",cwd);
        sprintf((char *)data->ascii_msg, "%s/%s", cwd ,file_name);
        // send thermal image to telemetry server.
        make_request("localhost", 8002, data);    
    }
    return 0;
}

