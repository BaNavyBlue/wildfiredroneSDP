############################################
# Antones Amazing Makefile
# ##########################################

WFLAGS     = -Wall -Wextra
GPP        = g++ ${WFLAGS}
INCL       = -I ~/GIT/c_library_v2 -I ~/GIT/c_library_v2/ardupilotmega
EXECFILE   = packer
LIBS       = -lpthread
SRC        = packwaypoints.cpp 

all : ${EXECFILE}

${EXECFILE} : ${SRC}
	${GPP} ${WFLAGS} ${SRC} -o ${EXECFILE}
