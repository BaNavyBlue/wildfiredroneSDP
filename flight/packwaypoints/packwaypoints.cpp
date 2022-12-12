// Wrote this code in one shot, but will it compile?

#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <stdint.h>
#include <inttypes.h>
#include <unistd.h>
#include <fcntl.h>

typedef double float64_t;

int main(int argc, char** argv){
    /* OG */
    /*std::vector<float64_t> latitude{36.995471, 36.995469, 36.995436, 36.995444,
                                    36.994989, 36.995018, 36.995040, 36.995037,
                                    36.995031, 36.994525, 36.994520, 36.994516,
                                    36.994526, 36.994549, 36.994560, 36.996095,
                                    36.995471};

    std::vector<float64_t> longitude{-122.031533, -122.032399, -122.033324, -122.034252,
                                     -122.034274, -122.033382, -122.032496, -122.031606,
                                     -122.030719, -122.030700, -122.031590, -122.032483,
                                     -122.033381, -122.034240, -122.035117, -122.035119,
                                     -122.031533};

    std::vector<float64_t> altitude{10.0, 11.0, 38.0, 35.0,
                                    38.0, 6.0, 11.0, 10.0,
                                    10.0, 10.0, 10.0, 10.0,
                                    5.0, 28.0, 38.0, 38.0,
                                    10.0};*/

    std::vector<float64_t> latitude{36.995138, 36.995142, 36.991532, 36.994104,
                                    36.991532, 36.995142, 36.992641, 36.995142,
                                    36.995138};

    std::vector<float64_t> longitude{-122.031275, -122.035758, -122.035758, -122.032609,
                                     -122.035758, -122.035758, -122.032598, -122.035758,
                                     -122.031275};

    std::vector<float64_t> altitude{10.0, 13.0, 66.0, 9.0,
                                    66.0, 13.0, 38.0, 13.0,
                                    10.0};


    int fd = open(argv[1], O_CREAT | O_TRUNC | O_RDWR, S_IRUSR | S_IWUSR);
    if(fd < 0){
        perror("couldn't open file");
        exit(1);
    }

    uint16_t points = latitude.size();
    
    ssize_t bytes_written = write(fd, &points, sizeof(uint16_t));
    if(bytes_written < 0){
        perror("write error");
    }    

    for(uint32_t i = 0; i < points; i++){
        bytes_written = write(fd, &latitude[i], sizeof(float64_t));
        if(bytes_written < 0){
            perror("write error");
        }
        bytes_written = write(fd, &longitude[i], sizeof(float64_t));
        if(bytes_written < 0){
            perror("write error");
        }
        bytes_written = write(fd, &altitude[i], sizeof(float64_t));
        if(bytes_written < 0){
            perror("write error");
        }
    }

    if(close(fd) < 0){
        perror("file close error. how is this even possible?");
        exit(1);
    }

    if(!(fd = open(argv[1], O_RDWR))){
        perror("file open error");
        printf("exiting!\n");
        return 1;
    }

    float64_t lat = 0.0;
    float64_t lon = 0.0;
    float64_t alt = 0.0;
    points = 0;

    ssize_t bytes_read = read(fd, &points, sizeof(uint16_t));
    if(bytes_read < 0){
        perror("read error");
    }

    printf("points: %u\n", points);

    for(uint16_t i = 0; i < points; i++){
        bytes_read = read(fd, &lat, sizeof(float64_t));
        if(bytes_read < 0){
            perror("read error");
        }

        bytes_read = read(fd, &lon, sizeof(float64_t));
        if(bytes_read < 0){
            perror("read error");
        }

        bytes_read = read(fd, &alt, sizeof(float64_t));
        if(bytes_read < 0){
            perror("read error");
        }
        printf("lat: %lf, lon: %lf, alt: %lf\n",lat , lon, alt);
    }

    if(close(fd) < 0){
        perror("what how is this possible file won't close?");
        exit(1);
    }

    return 0;
}
