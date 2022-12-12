#include "write_helper.h"

static uint16_t sent_bytes = 0;

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
                sent_bytes += wcount;
                //printf("wcount: %d writes: %d\n", wcount, writes);
                //#ifdef BASE
                    usleep(DELAY);
                //#endif

            }
            #ifdef DRONE
                if(sent_bytes > 2000){
                    tcdrain(*shared->fd);
                    sent_bytes = 0;
                } else {
                    usleep(1);
                }
            #endif

            #ifdef BASE
                usleep(DELAY);
            #endif
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
