/*
 * Antone Bajor: abajor
 *
 * Originally for CSE103 Miller, E.
 * Being repurposed for Senior Design Project.
 *
 * Assignment2: Multithreaded HTTP Server
 * 
 *
 * https://www.geeksforgeeks.org/
 * socket-programming-in-cc-handling-multiple-clients-on-server-without-multi-threading/
 * Advanced Programming in the UNIX Environment
 *
 * This program is a basic HTTP server which is capable of PUTing data onto a server
 * and GETing data from the server.
 *
 * This server will spawn a thread pool of worker threads specifically for handling client
 * input.  If the worker thread has no work it will sleep.
 *
 * The default amount of threads is 4, but the user can use the -N <thread number> option
 * to tell the server to spawn N threads.  It will accept up to a 32bit integer but the 
 * operating system will likely limit the total number of threads.
 *
 * This server also has a logging feature if the user uses the -l <filename> option.
 * The logging function logs valid GET and PUT commands along with the size of the data
 * in bytes and the data gets printed in hexadecimal.
 *
 * Any errors will also be logged without any potential erroneous data.
 * if all of the worker threads are busy the distpatch or main thread will sleep until
 * at least one worker thread is done.
 *
 */
#include "../userland_edits/RStill_Func.h"
#include "simple_server.hpp"
#include "request_client.hpp" 

void scan_opts (int argc, char** argv);

//uint32_t thread_count = 2; // Default Thread pool size
//uint8_t thread_flags = 0;
//int log_fd = 0;
int arg_ind = 0;

int main(int argc, char **argv){
    //int err;
    //off_t log_offset = 0;

    signal(SIGPIPE, SIG_IGN);

    if(argc < 3){
        fprintf(stderr, "Usage: %s <$HOSTNAME> <port>\n"
                , argv[0]);
	exit(1);
    }


    // getopt helper
    //scan_opts(argc, argv);

    // Counting Semaphore
    //sem_t S;
    //sem_init(&S, 0, thread_count);

    // How I leared to stop worrying and love malloc
    /*thread_dat *data = (thread_dat*)malloc(sizeof(thread_dat) * thread_count);
    pthread_t *thread_pool = (pthread_t*)malloc(sizeof(pthread_t) * thread_count);
    pthread_mutex_t *work_enable = (pthread_mutex_t*)malloc(sizeof(pthread_mutex_t) * thread_count);
    pthread_mutex_t crit_reg, crit_reg2, crit_recv, crit_send;
    pthread_mutex_init(&crit_reg, NULL);
    pthread_mutex_init(&crit_reg2, NULL);
    pthread_mutex_init(&crit_recv, NULL);
    pthread_mutex_init(&crit_send, NULL);*/

    // Setting up my RStill_Func() Thread

    srv_resp thd_data; // For passing data between image capture thread
    

    printf("Before Pthreads\n");
    pthread_t pi_cam_thrd;
    pthread_mutex_t pi_lock;
    pthread_mutex_init(&pi_lock, NULL);
    pthread_mutex_lock(&pi_lock);
    rgb_data rgb_thd_data;
    rgb_thd_data.gps = &thd_data.gps;
    //rgb_thd_data.gps.lon = &thd_data.gps.lon;
    //rgb_thd_data.gps.height = &thd_data.gps.height;
    rgb_thd_data.next_still = &pi_lock;
    thd_data.mutex = &pi_lock;
    rgb_thd_data.file_name = thd_data.ascii_msg;
    int err = pthread_create(&pi_cam_thrd, NULL, still_collect, (void*)&rgb_thd_data);
    printf("After Pthreads\n");
    
    /*********************************************************************************/
    /*        This is where I set up the listening server                            */
    /*********************************************************************************/


    simple_server(atoi(argv[2]), &thd_data);

    // Set up socket address struct
    //struct sockaddr_in addr;

    //uint32_t i;

    /*for(i = 0; i < thread_count; i++){
        data[i].sockfd = 0; 
	data[i].thread_flag = &thread_flags; // Inteded for interthread signaling using masking
	pthread_mutex_init(&work_enable[i], NULL);
	data[i].work_enable = &work_enable[i];   // Each thread gets it's own mutex to sleep it
	pthread_mutex_lock(data[i].work_enable); //This locks worker threads out of inner loop
	data[i].crit_reg = &crit_reg; 
	data[i].crit_reg2 = &crit_reg2;
	data[i].crit_recv = &crit_recv; // not in use
        data[i].crit_send = &crit_send; // not in use
        data[i].thread_num = i; // sequential thread identification
	data[i].S = &S; // sharing a counting semaphore to sleep dispatcher
        data[i].active = 1; // unused
	data[i].addr = &addr; // shares the sockets info amongst all threads
	//data[i].log_fd = log_fd; // shares log file file descriptor amongst all threads.
	//data[i].log_offset = &log_offset; // shares logfile offset amongst all threads.
	// Roll your own thread pool adventure
        err = pthread_create(&thread_pool[i], NULL, client_handler, (void*)(data + i));
    }*/

    // The message buffer
    /*unsigned char buffer[96];

    sprintf((char*)buffer, "Starting %s at %s using port %s\n", argv[0], argv[1]
            , argv[2]);
    write(1, buffer, strlen((char*)buffer));

    //struct hostent *hent = gethostbyname(argv[1]);
    struct sockaddr_storage serverStorage;
    socklen_t addr_size;*/

    /*if(!hent){
        fprintf(stderr, "failed to get host\n");
        exit(1);
    }*/

    // need to investigate why this mem copy is nescessary.
    //memcpy(&addr.sin_addr.s_addr, hent->h_addr, hent->h_length);
    /*addr.sin_addr.s_addr = inet_addr("127.0.0.1"); // looking for local host
    addr.sin_port = htons(atoi(argv[2]));
    addr.sin_family = AF_INET;
    printf("Port: %u %u\n", addr.sin_port, ntohs(addr.sin_port));

    // Creating a Socket
    int srv_fd = socket(AF_INET, SOCK_STREAM, 0);

    //Socket Setup for Server
    int enable = 1;
    setsockopt(srv_fd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
    bind(srv_fd, (struct sockaddr *)&addr, sizeof(addr));
    printf("Before listen\n");
    if(!listen(srv_fd, 3)){
        printf("listening\n");
    } else {
        printf("after listen\n");
    }
    printf("after listen\n");
    while(1){
        // This counting semaphore will block if all threads
	// are active
        //sem_wait(&S);
        file_msg the_msg;
        srv_resp the_resp;
        printf("Before accept\n");
    
        addr_size = sizeof(serverStorage);
        int sockfd = accept(srv_fd, (struct sockaddr*)&serverStorage, &addr_size);
        printf("after accept\n");
        ssize_t n = recv(sockfd, (void*)&the_msg, sizeof(file_msg), 0);
        printf("we recv: %u %u %s\n", the_msg.file_type, the_msg.action, (char *)the_msg.ascii_msg);
        if(n < 0){
            fprintf(stderr, "socket error what do now?\n");
        }

        if(the_msg.file_type == LWIR){
            printf("You did it LWIR\n");
            the_resp.response = ACCEPT;
            n = send(sockfd, (void*)&the_resp, sizeof(srv_resp), 0);    
        }
   
        close(sockfd);
    }
        // Unfortunately the code currently will never arrive here
	// until I implement a way to exit the server, probaly with a 
	// get statement looking for an esc key press.
	// It would be a good idea to use a barrier prevent the server from exiting
	// before the worker threads can finish.
        //if(log_fd > 0){
        //    close(log_fd);
        //}
        //free(data);
        //free(thread_pool);
        //free(work_enable);*/
        return 0;
}

// This is the function that handles parsing input arguments
// it is also responsible for opening the log file.
void scan_opts (int argc, char** argv){

    opterr = 0;
    int tcnt;

    for(;;) {
        int opt = getopt (argc, argv, "N: l:");
        if (opt == EOF) break;
        switch (opt) {
	    case 'N': //printf("case N: %s\n", optarg); 
                tcnt = atoi(optarg);
		//thread_count = (uint32_t)tcnt;
                break;
	    case 'l': //printf("case l: %s\n", optarg);
                //thread_flags ^= LOGGING;
                //log_fd = open(optarg, O_WRONLY|O_TRUNC|O_CREAT, S_IRUSR | S_IWUSR | S_IROTH);
		//if(log_fd < 0){
                //    fprintf(stderr, "failed to create log file: %s\n may the lord have mercy on your soul\n"
		//            , optarg);
		//    exit(1);
		//}
                break;
	    default: printf("default case optarg: %s optind: %d\n", optarg, optind); break;
        }
    }

    arg_ind = optind;
    printf("optind: %d, argv[optind]: %s\n", optind, argv[optind]);
}

