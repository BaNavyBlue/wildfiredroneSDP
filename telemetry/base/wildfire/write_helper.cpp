#include "write_helper.h"

void write_packet(uint8_t *buffer, thread_data *shared){

        ssize_t writes = 0;
        ssize_t wcount = 0;
        pthread_mutex_lock(shared->uart_mut);
        while(writes < buffer[1] + BASE_SIZE){
            wcount = write(*shared->fd, &buffer[writes], /*BASE_SIZE + buffer[1] +*/ 1 /*- writes*/);
            /*if(wcount == 0){
                printf("nothing written\n");
            }*/
            if(wcount < 0){
                perror("uart error");
                usleep(DELAY);
            } else {
                writes += wcount;
                //printf("wcount: %d writes: %d\n", wcount, writes);
                usleep(DELAY);

            }
            //tcdrain(*shared->fd);
        }
        pthread_mutex_unlock(shared->uart_mut);

      if (wcount < 0){
          // The negative return value from the write function generally indicates an error,
          // but we need to exclude the value EAGAIN which simply asks us to try again
          // because the transmit buffer is currently full.
          if (errno != EAGAIN){
              // This is a fatal error, so can't continue
              perror("Write");
              //return -1;
          }
      } else {
          //printf("wcount: %d\n", wcount);
      }

}
