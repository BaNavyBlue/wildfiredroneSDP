// insanely complex code
// time_keeper thread
// Antone A. Bajor

#include "time_keeper.h"


void* time_keeper(void* data){
    time_data *my_time = (time_data*)data;
    my_time->t_ms = 0.0;
    time_t t = time(NULL);
    struct tm *tm = localtime(&t);
    printf("started at %d:%02d:%02d\n", tm->tm_hour, tm->tm_min, tm->tm_sec);

    while(1){
        sprintf(my_time->clock, "%d:%02d:%02d", tm->tm_hour, tm->tm_min, tm->tm_sec);
        usleep(33333);
        my_time->t_ms += 33.333;
        t = time(NULL);
        tm = localtime(&t);
    } 

}
