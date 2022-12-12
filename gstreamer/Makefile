#make file - gstreamer
LIBS = -pthread -lgstreamer-1.0 -lgobject-2.0 -lglib-2.0
INCL = -I/usr/include/gstreamer-1.0 -I/usr/include/glib-2.0 \
       -I/usr/lib/arm-linux-gnueabihf/glib-2.0/include \
       -I ~/GIT/wildfire/imaging_server
SRC  = sad.c gst_capture.c ~/GIT/wildfire/imaging_server/request_client.cpp
TARGET = sadness
all:	${TARGET}

${TARGET} : ${SRC}
	g++ ${INCL} -Wall ${SRC} ${LIBS} -o ${TARGET}
