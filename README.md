# wildfiredroneSDP
Senior Design Project Code used for automating drone flight to GPS way points and capturing images.

This repository needs to be cleaned up.  Code was developed for interfacing with the DJI Matrice 100 developer drone over uart, using a raspberry pi.


# flight directory:
    contains modified DJI OSDK flight routine sample
    also contains pack waypoints code for ingesting list of gps waypoints (as doubles) and dumping them into a binary file that is read by the flight           routine before starting.
# gstreamer directory:
    contains some gstreamer test code
#  
# imaging_server directory:
    contains a TCP/IP client server method of passing commands between different procedures.
# modified_dji directory:
    contains modified DJI::OSDK flight control code to suit our application.
# telemetry directory:
    contains code for using a modified version of the MAVLink protocol for transfering data over a UART radio.
    seperate versions of the code exist for the Drone and Base Station
# userland_edits directory:
    contain modified Raspberry Pi userland still capture code for the Pi camera module.
