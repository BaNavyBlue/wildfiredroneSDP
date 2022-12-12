// give me spice!

#include "telem_thread.hpp"

void* telem_thread(void* data){
    //std::cout << "The Thread Has been created!!" << std::endl;
    printf("The Thread Has Been Created!!\n");
    thread_dat* mydata = (thread_dat*)data;
    // File Descriptor
    // int vel_fd = 0;
    // int acc_fd = 0;
    //int quat_fd = 0;
    int time_fd = 0;
    int gps_fd = 0;
    char filename[100];
    char str_buf[300];
    
    /*
    sprintf(filename, "data/vel_%s.csv", mydata->file_n_info); 
    if(!(vel_fd = open(filename, O_CREAT | O_TRUNC | O_RDWR, S_IRUSR | S_IWUSR))){
        perror("Couldn't open or create file");
    } else {
        printf("%s created.\n", filename);
    }
    sprintf(str_buf, "!%s you dingus\n", filename);
    ssize_t bytes_writen = write(vel_fd, str_buf, strlen(str_buf));
    if(bytes_writen < 0){
        perror("failed to write to file");
    }

    sprintf(filename, "data/acc_%s.csv", mydata->file_n_info); 
    if(!(acc_fd = open(filename, O_CREAT | O_TRUNC | O_RDWR, S_IRUSR | S_IWUSR))){
        perror("Couldn't open or create file");
    } else {
        printf("%s created.\n", filename);
    }
    sprintf(str_buf, "!%s you dingus\n", filename);
    bytes_writen = write(acc_fd, str_buf, strlen(str_buf));
    if(bytes_writen < 0){
        perror("failed to write to file");
    }*/

    /*sprintf(filename, "data/quat_%s.csv", mydata->file_n_info); 
    if(!(quat_fd = open(filename, O_CREAT | O_TRUNC | O_RDWR, S_IRUSR | S_IWUSR))){
        perror("Couldn't open or create file");
    } else {
        printf("%s created.\n", filename);
    }
    sprintf(str_buf, "!%s you dingus\n", filename);
    bytes_writen = write(quat_fd, str_buf, strlen(str_buf));
    if(bytes_writen < 0){
        perror("failed to write to file");
    }*/

    sprintf(filename, "data/gps_%s.csv", mydata->file_n_info); 
    if(!(gps_fd = open(filename, O_CREAT | O_TRUNC | O_RDWR, S_IRUSR | S_IWUSR))){
        perror("Couldn't open or create file");
    } else {
        printf("%s created.\n", filename);
    }
    sprintf(str_buf, "!%s you dingus\n", filename);
    ssize_t bytes_writen = write(gps_fd, str_buf, strlen(str_buf));
    if(bytes_writen < 0){
        perror("failed to write to file");
    }

    sprintf(filename, "data/time_%s.csv", mydata->file_n_info); 
    if(!(time_fd = open(filename, O_CREAT | O_TRUNC | O_RDWR, S_IRUSR | S_IWUSR))){
        perror("Couldn't open or create file");
    } else {
        printf("%s created.\n", filename);
    }
    sprintf(str_buf, "!%s you dingus\n", filename);
    bytes_writen = write(time_fd, str_buf, strlen(str_buf));
    if(bytes_writen < 0){
        perror("failed to write to file");
    }
    //sprintf(str_buf, "Sys_clk, t_elapsed(ms), vx(m/s), vy(m/s), vz(m/s), ax, ay, az, roll, pitch, yaw, thr, lat, lon, height, q.w, q.x, q.y. q.z, bat\n");
    
    sprintf(str_buf, "Sys_clk, t_elapsed(ms), bat%\n");
    bytes_writen = write(time_fd, str_buf, strlen(str_buf));
    if(bytes_writen < 0){
        perror("failed to write to file");
    }

    /*sprintf(str_buf, "vel.x(m/s), vel.y(m/s), vel.z(m/s)\n");
    bytes_writen = write(vel_fd, str_buf, strlen(str_buf));
    if(bytes_writen < 0){
        perror("failed to write to file");
    }*/

    /*sprintf(str_buf, "acc.x, acc.y, acc.z\n");
    bytes_writen = write(acc_fd, str_buf, strlen(str_buf));
    if(bytes_writen < 0){
        perror("failed to write to file");
    }*/

    sprintf(str_buf, "LAT, LON, Altitude(m), Height(m)\n");
    bytes_writen = write(gps_fd, str_buf, strlen(str_buf));
    if(bytes_writen < 0){
        perror("failed to write to file");
    }

    /*sprintf(str_buf, "quat.w, quat.x, quat.y, quat.z\n");
    bytes_writen = write(quat_fd, str_buf, strlen(str_buf));
    if(bytes_writen < 0){
        perror("failed to write to file");
    }*/

    const int TIMEOUT = 20;

    // Re-set Broadcast frequencies to their default values
    ACK::ErrorCode ack = mydata->vehicle->broadcast->setBroadcastFreqDefaults(TIMEOUT);
    time_t t = time(NULL);
    struct tm *tm = localtime(&t);
    printf("%d-%02d-%02d\n", tm->tm_hour, tm->tm_min, tm->tm_sec);
    //Telemetry::Vector3f acc = mydata->vehicle->broadcast->getAcceleration();
    Telemetry::Battery batt = mydata->vehicle->broadcast->getBatteryInfo();
    //Telemetry::Vector3f vel = mydata->vehicle->broadcast->getVelocity();
    //Telemetry::TimeStamp timex = mydata->vehicle->broadcast->getTimeStamp();
    //Telemetry::Quaternion quat = mydata->vehicle->broadcast->getQuaternion();
    Telemetry::GlobalPosition gps = mydata->vehicle->broadcast->getGlobalPosition();
    //printf("acc - x: %f y: %f z: %f\n", acc.x, acc.y, acc.z);
    //printf("vel - x: %f y: %f z: %f\n", vel.x, vel.y, vel.z);
    printf("capacity: %u, voltage: %d, current: %d, percent: %u%\n", batt.capacity, batt.voltage, batt.current,
           batt.percentage);
    //printf("time_ms: %u, time_ns: %u\n", timex.time_ms, timex.time_ns);
    //printf("quaternion - w: %f, x: %f, y: %f, z: %f\n", quat.q0, quat.q1, quat.q2, quat.q3);
    printf("LAT: %lf, LON: %lf, alt: %f, height: %f\n", gps.latitude, gps.longitude, gps.altitude, gps.height);
    //timex = mydata->vehicle->broadcast->getTimeStamp();
    //printf("time_ms: %u, time_ns: %u\n", timex.time_ms, timex.time_ns);
    double t_ms = 0.0;
    while(1){

        //acc = mydata->vehicle->broadcast->getAcceleration();
        batt = mydata->vehicle->broadcast->getBatteryInfo();
        //vel = mydata->vehicle->broadcast->getVelocity();
        //timex = mydata->vehicle->broadcast->getTimeStamp();
        //quat = mydata->vehicle->broadcast->getQuaternion();
        gps = mydata->vehicle->broadcast->getGlobalPosition();
        t = time(NULL);
        tm = localtime(&t);
       
        sprintf(str_buf, "%2d:%02d:%02d, %lf, %u\n", tm->tm_hour, tm->tm_min, tm->tm_sec,
                t_ms, batt.percentage);
        bytes_writen = write(time_fd, str_buf, strlen(str_buf));
        if(bytes_writen < 0){
            perror("failed to write to file");
        }

        /*sprintf(str_buf, "%f, %f, %f\n", vel.x, vel.y, vel.z);
        bytes_writen = write(vel_fd, str_buf, strlen(str_buf));
        if(bytes_writen < 0){
            perror("failed to write to file");
        }*/

        /*sprintf(str_buf, "%f, %f, %f\n", acc.x, acc.y, acc.z);
        bytes_writen = write(acc_fd, str_buf, strlen(str_buf));
        if(bytes_writen < 0){
            perror("failed to write to file");
        }*/

        sprintf(str_buf, "%.9lf, %.9lf, %f, %f\n", gps.latitude, gps.longitude, gps.altitude,
                gps.height);
        bytes_writen = write(gps_fd, str_buf, strlen(str_buf));
        if(bytes_writen < 0){
            perror("failed to write to file");
        }

        /*sprintf(str_buf, "%f, %f, %f, %f\n", quat.q0, quat.q1, quat.q2, quat.q3);
        bytes_writen = write(quat_fd, str_buf, strlen(str_buf));
        if(bytes_writen < 0){
            perror("failed to write to file");
        }*/
        usleep(33333);
        t_ms += 33.333;
    }
}
