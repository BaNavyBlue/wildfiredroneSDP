// The duke will die before these eyes, then he'll know...

#ifndef TELEM_THREAD_HPP
#define TELEM_THREAD_HPP
#include "flight_control_sample.hpp"
#include <assert.h>
#include <time.h>

typedef struct thread_dat{
    char* file_n_info;
    DJI::OSDK::Vehicle* vehicle;
    struct tm *tm;
    double t_ms = 0.0;
}thread_dat;


void* telem_thread(void* data);

#endif // TELEM_THREAD_HPP
