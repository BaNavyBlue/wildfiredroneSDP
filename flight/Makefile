############################################
# Antones Amazing Makefile
# ##########################################

WFLAGS     = -Wall -Wextra
GPP        = g++ ${WFLAGS}
INCL       = -I ~/GIT/Onboard-SDK/osdk-core/api/inc \
             -I ~/GIT/Onboard-SDK/osdk-core/protocol/inc \
             -I ~/GIT/Onboard-SDK/osdk-core/hal/inc \
             -I ~/GIT/Onboard-SDK/osdk-core/utility/inc \
             -I ~/GIT/Onboard-SDK/osdk-core/platform/linux/inc \
             -I ~/GIT/Onboard-SDK/osdk-core/modules/inc/payload \
             -I ~/GIT/Onboard-SDK/osdk-core/modules/inc/flight \
             -I ~/GIT/Onboard-SDK/sample/platform/linux/common \
             -I ~/GIT/Onboard-SDK/osdk-core/platform/default/inc #-I ~/GIT/c_library_v2/ardupilotmega
SRC_PATHS  =  $(wildcard ~/GIT/Onboard-SDK/osdk-core/api/src/*.cpp) \
              $(wildcard ~/GIT/Onboard-SDK/osdk-core/protocol/src/*.cpp) \
              $(wildcard ~/GIT/Onboard-SDK/osdk-core/hal/src/*.cpp) \
              $(wildcard ~/GIT/Onboard-SDK/osdk-core/utility/src/*.cpp) \
              $(wildcard ~/GIT/Onboard-SDK/osdk-core/platform/linux/src/*.cpp) \
              $(wildcard ~/GIT/Onboard-SDK/osdk-core/modules/src/*.cpp) \
              $(wildcard ~/GIT/Onboard-SDK/osdk-core/modules/src/flight/*.cpp) \
              $(wildcard ~/GIT/Onboard-SDK/osdk-core/modules/src/payload/*.cpp) \
              $(wildcard ~/GIT/Onboard-SDK/osdk-core/platform/default/src/*.cpp) \
              $(wildcard ~/GIT/Onboard-SDK/sample/platform/linux/common/*.cpp)
EXECFILE   = test350
LIBS       = -lpthread
SRC        = main.cpp flight_control_sample.cpp telem_thread.cpp ../waypoint/mission_sample.cpp ../imaging_server/request_client.cpp ${SRC_PATHS}
OBJDIR     = obj
OBJ        = $(notdir $(SRC:.cpp=.o))
OBJECTS    = $(addprefix obj/, $(OBJ))

# objects= $(addprefix $(OBJDIR)/, $(OBJECTS))
all: ${EXECFILE}

${EXECFILE}: $(SRC)
	${GPP} ${INCL} $(SRC) ${LIBS} -o ${EXECFILE}

#$(OBJECTS): $(OBJDIR)/%.o : ${SRC} #%.cpp
#	${GPP} ${INCL} -c $< -o $@
