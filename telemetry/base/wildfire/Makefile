############################################
# Antones Amazing Makefile
# ##########################################

WFLAGS     = -Wall -Wextra
GPP        = g++ ${WFLAGS}
INCL       = -I ~/workspace/GIT/c_library_v2 -I ../ardupilotmega
EXECFILE   = test_3dr
LIBS       = -lpthread
SRC        = telem_uart.cpp write_helper.cpp heart_beat.cpp uart_recv.cpp send_ack.cpp

all : ${EXECFILE}

${EXECFILE} : ${SRC}
	${GPP} ${INCL} ${WFLAGS} ${SRC} ${LIBS} -o ${EXECFILE}
