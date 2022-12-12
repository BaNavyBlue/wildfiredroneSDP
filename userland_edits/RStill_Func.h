#ifndef RSTILL_FUNC_H
#define RSTILL_FUNC_H
/*
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <memory.h>
#include <unistd.h>
#include <errno.h>
#include <sysexits.h>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/mmal_parameters_camera.h"

#include "RaspiCommonSettings.h"
#include "RaspiCamControl.h"
#include "RaspiPreview.h"
#include "RaspiCLI.h"
#include "RaspiTex.h"
#include "RaspiHelpers.h"

// TODO
//#include "libgps_loader.h"

#include "RaspiGPS.h"

#include <semaphore.h>
#include <math.h>
#include <pthread.h>
#include <time.h>
*/
#include <pthread.h>
#include <sys/types.h>
#include <stdint.h>
#include "request_client.hpp"

typedef struct rgb_data{
    uint8_t *file_name;
    pthread_mutex_t *next_still;
}rgb_data;

void* still_collect(void *indata);

#endif
