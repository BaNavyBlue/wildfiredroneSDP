############################################
# Antones Amazing Makefile
# ##########################################

WFLAGS     = -Wall -Wextra
GPP        = g++ ${WFLAGS}
INCL       = -I ~/GIT/c_library_v2 -I ~/GIT/c_library_v2/ardupilotmega -I ../imaging_server
EXECFILE   = test_3dr
LIBS       = -lpthread
SRC        = telem_uart.cpp write_helper.cpp heart_beat.cpp uart_recv.cpp send_ack.cpp time_keeper.cpp uart_server.cpp 

all : ${EXECFILE}

${EXECFILE} : ${SRC}
	${GPP} ${INCL} ${WFLAGS} ${SRC} ${LIBS} -o ${EXECFILE}
