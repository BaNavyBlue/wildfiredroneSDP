/*! @file flight-control/main.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  main for Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @Copyright (c) 2016-2017 DJI
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */


/*TODO:flight_control_sample will by replace by flight_sample in the future*/
#include <iomanip>
#include "flight_control_sample.hpp"
#include "telem_thread.hpp"
#include "../imaging_server/request_client.hpp"
//#include "flight_sample.hpp"
#define COLLECT_FLIGHT_DATA 1
//#define FLIGHT_DATA "naked_burn.pnt"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int main(int argc, char** argv) {
  signal(SIGPIPE, SIG_IGN);
  time_t t = time(NULL);
  struct tm *tm = localtime(&t);

  char file_d_and_time[100];
  assert(strftime(file_d_and_time, sizeof(file_d_and_time), "%c", tm));

  //sprintf(file_d_and_time, "%s_%s", __TIME__, __DATE__);
  
  for(int i = 0; i < 100; i++){
      if(file_d_and_time[i] == '\0'){
          break;
      } else if(file_d_and_time[i] == ':'){
          file_d_and_time[i] = '-';
      } else if(file_d_and_time[i] == ' '){
          file_d_and_time[i] = '_';
      }
  }
  


  std::cout << "File Header: " << file_d_and_time << std::endl;
  
  // std::cout << "Time: " << __TIME__ << " Date: " << __DATE__ << std::endl;
  // Initialize variables
  int functionTimeout = 1;

  // Setup OSDK.
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle* vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL) {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // For collecting time coded flight data
  #ifdef COLLECT_FLIGHT_DATA
      pthread_t flight_data;
      thread_dat mydata;
      mydata.vehicle = vehicle;
      mydata.file_n_info = file_d_and_time;
      pthread_create(&flight_data, NULL, telem_thread, (void*)&mydata);
  #endif

  // Obtain Control Authority
  vehicle->obtainCtrlAuthority(functionTimeout);

  // Display interactive prompt
  std::cout
      << "| Available commands:                                            |"
      << std::endl;
  std::cout
      << "| [a] Monitored Takeoff + Landing                                |"
      << std::endl;
  std::cout
      << "| [b] Monitored Takeoff + Position Control + Landing             |"
      << std::endl;
  std::cout << "| [c] Monitored Takeoff + Position Control + Force Landing "
               "Avoid Ground  |"
            << std::endl;
  std::cout << "| [d] Monitored Takeoff + Position Control + Some type of landing Landing\n"
               "Avoid Ground  |"
            << std::endl;
  std::cout << "| [e] Monitored Takeoff + Velocity Control + Some type of landing Landing\n"
               "Avoid Ground  |"
            << std::endl;
  std::cout << "| [f] Test small velocity manuvers + Velocity Control + Some type of landing Landing\n"
               "Avoid Ground  |"
            << std::endl;
  std::cout << "PLIBT!" << std::endl;
  char inputChar;
  std::cin >> inputChar;

  switch (inputChar) {
    case 'a':
      monitoredTakeoff(vehicle);
      monitoredLanding(vehicle);
      break;
    case 'b':
      monitoredTakeoff(vehicle);
      moveByPositionOffset(vehicle, 0, 6, 6, 30, 0);
      moveByPositionOffset(vehicle, 6, 0, -3, -30, 0);
      moveByPositionOffset(vehicle, -6, -6, 0, 0, 0);
      monitoredLanding(vehicle);
      break;

    /*! @NOTE: case 'c' only support for m210 V2*/
    case 'c':
      /*! Turn off rtk switch */
      ErrorCode::ErrorCodeType ret;
      ret = vehicle->flightController->setRtkEnableSync(
          FlightController::RtkEnabled::RTK_DISABLE, 1);
      if (ret != ErrorCode::SysCommonErr::Success) {
        DSTATUS("Turn off rtk switch failed, ErrorCode is:%8x", ret);
      } else {
        DSTATUS("Turn off rtk switch successfully");
      }
      
      /*!  Take off */
      //monitoredTakeoff(vehicle);

      /*! Move to higher altitude */
      //moveByPositionOffset(vehicle, 0, 0, 30, 0);

      /*! Move a short distance*/
      //moveByPositionOffset(vehicle, 10, 0, 0, -30);

      /*! Set aircraft current position as new home location */
      //setNewHomeLocation(vehicle);

      /*! Set new go home altitude */
      //setGoHomeAltitude(vehicle, 50);

      /*! Move to another position */
      //moveByPositionOffset(vehicle, 40, 0, 0, 0);

      /*! go home and  confirm landing */
      //goHomeAndConfirmLanding(vehicle, 1);
      break;
    case 'd': {
      /*
      // Prep the bull

      // How hight to start.
      float32_t start_alt = 76.2; // Equal to 250'
      // Create Start Waypoint
      WayPointSettings start_wp;
      ::OSDK::WayPointSettings
      // Global position retrieved via subscription
      Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition;
      // Global position retrieved via broadcast
      Telemetry::GlobalPosition broadcastGPosition;
      Telemetry::Vector3f localOffset;

      // Take off first
      monitoredTakeoff(vehicle);
      broadcastGPosition = vehicle->broadcast->getGlobalPosition();
      float64_t zoff = broadcastGPosition.altitude;
      std::cout << "Height: " << broadcastGPosition.height << std::endl;
      std::cout << "zoff: " << zoff << std::endl;
      if(zoff < 0){
          zoff = -1 * zoff;
      } else {
          zoff = 0;
      }

      //start_alt += zoff;

      moveByPositionOffset(vehicle, 0, 0, 72.6, 0, zoff); 
      // Get current gps coord
      if (!vehicle->isM100() && !vehicle->isLegacyM600())
      {
        subscribeGPosition = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
        start_wp.latitude  = subscribeGPosition.latitude;
        start_wp.longitude = subscribeGPosition.longitude;
        start_wp.altitude  = start_alt;
        printf("Waypoint created at (LLA): %f \t%f \t%f\n",
               subscribeGPosition.latitude, subscribeGPosition.longitude,
               start_alt);
      }
      else
      {
        broadcastGPosition = vehicle->broadcast->getGlobalPosition();
        start_wp.latitude  = broadcastGPosition.latitude;
        start_wp.longitude = broadcastGPosition.longitude;
        start_wp.altitude  = start_alt;
        printf("Waypoint created at (LLA): %f \t%f \t%f\n",
               broadcastGPosition.latitude, broadcastGPosition.longitude,
               start_alt);
      }

      std::vector<float64_t> latitude{36.995471, 36.995469, 36.995436, 36.995444,
                                      36.994989, 36.995018, 36.995040, 36.995037,
                                      36.995031, 36.994525, 36.994520, 36.994516,
                                      36.994526, 36.994549, 36.994560, 36.996095,
                                      36.995471};

     std::vector<float64_t> longitude{-122.031533, -122.032399, -122.033324, -122.034252,
                                      -122.034274, -122.033382, -122.032496, -122.031606,
                                      -122.030719, -122.030700, -122.031590, -122.032483,
                                      -122.033381, -122.034240, -122.035117, -122.035119,
                                      -122.031533};

     std::vector<float64_t> altitude{86.2 ,87.2, 114.2, 111.2,
                                     114.2, 82.2, 87.2, 86.2,
                                     86.2, 86.2, 86.2, 86.2,
                                     81.2, 104.2, 114.2, 114.2,
                                     86.2};
      
      if (!vehicle->isM100() && !vehicle->isLegacyM600())
      {
         subscribeGPosition = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
         start_wp.latitude  = latitude.at(0) * M_PI / 180;  //subscribeGPosition.latitude;
         start_wp.longitude = longitude.at(0) * M_PI / 180; //subscribeGPosition.longitude;
         start_wp.altitude  = start_alt;
         printf("Waypoint created at (LLA): %f \t%f \t%f\n",
                subscribeGPosition.latitude, subscribeGPosition.longitude,
                start_alt);
      }
      else
      {
         broadcastGPosition = vehicle->broadcast->getGlobalPosition();
         start_wp.latitude  = broadcastGPosition.latitude;
         start_wp.longitude = broadcastGPosition.longitude;
         start_wp.altitude  = start_alt;
         printf("Waypoint created at (LLA): %f \t%f \t%f\n",
                broadcastGPosition.latitude, broadcastGPosition.longitude,
                start_alt);
      }

     
     
      std::vector<DJI::OSDK::WayPointSettings> my_points = create_waypoints(&start_wp
                  , &longitude, &latitude, &altitude);

      // localOffsetFromGpsOffset(vehicle, localOffset, &my_points(1) ,);
      
      // Sort of there, 10 out of maybe?
      for(int i = 0; i < my_points.size() - 1; i++){
          Telemetry::GlobalPosition target;
          Telemetry::GlobalPosition origin;
          Telemetry::GlobalPosition now = vehicle->broadcast->getGlobalPosition();
          std::cout << "Where Are We Now:\nLAT: " << now.latitude * 180 / M_PI << " LON: " <<
                       now.longitude * 180 / M_PI << "\nALT: " << now.altitude << " Height: " <<
                       now.height << std::endl;

          target.latitude = my_points[i + 1].latitude;
          target.longitude = my_points[i + 1].longitude;
          target.height = my_points[i + 1].altitude;

          origin.latitude = my_points[i].latitude;
          origin.longitude = my_points[i].longitude;
          origin.height = my_points[i].altitude;



          

          localOffsetFromGpsOffset(vehicle, localOffset, &target, &origin);
          std::cout << "Where we want to go:\n";
          std::cout << "LAT: " << my_points.at(i + 1).latitude  * 180 / M_PI << " LON: " << 
                       my_points.at(i + 1).longitude * 180 / M_PI << " Height: " << my_points.at(i + 1).altitude
                    <<  std::endl;
          std::cout << "Where we started:\n";
          std::cout << "LAT: " << my_points.at(i).latitude  * 180 / M_PI << " LON: " << 
                       my_points.at(i).longitude * 180 / M_PI << " Height: " << my_points.at(i).altitude <<
                       std::endl;
          std::cout << "xOff: " << localOffset.x << "m yOff: " << localOffset.y << "m zOff: " <<
                       localOffset.z << "m" << std::endl;


          moveByPositionOffset(vehicle, localOffset.x, localOffset.y, localOffset.z, 0, zoff);
      }

      //moveByPositionOffset(vehicle, 0, 6, 6, 30);
      //moveByPositionOffset(vehicle, 6, 0, -3, -30);
      //moveByPositionOffset(vehicle, -6, -6, 0, 0);
      monitoredLanding(vehicle);
      break;*/
    }
    case 'e': {
      // Prep the bull

      // How hight to start.
      const float32_t start_height = 76.2; // Equal to 250'
      // Create Start Waypoint
      MyWayPointSettings start_wp;
      MySetWaypointDefaults(&start_wp);

      // Global position retrieved via subscription
      Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition;
      // Global position retrieved via broadcast
      Telemetry::GlobalPosition broadcastGPosition;
      Telemetry::Vector3f localOffset;

      // Take off first
      monitoredTakeoff(vehicle);
      broadcastGPosition = vehicle->broadcast->getGlobalPosition();
      float64_t zoff = broadcastGPosition.altitude;
      std::cout << "Height: " << broadcastGPosition.height << std::endl;
      std::cout << "zoff: " << zoff << std::endl;
      if(broadcastGPosition.height < 0.008){
          std::cout << "something wrong not in the air!!!\nexiting..." << std::endl;
          return 1;
      }
      if(zoff < 0){
          zoff = -1 * zoff;
      } else {
          zoff = 0;
      }

      //start_alt += zoff;

      /* hopefully this loop will make sure we make it there */
      float32_t zdist = start_height;
      while(broadcastGPosition.height < 72.6){
          std::cout << "zdist: " << zdist << " height: " << broadcastGPosition.height << std::endl;
          moveByVelocityControl(vehicle, 0, 0, zdist, 0, zoff);
          broadcastGPosition = vehicle->broadcast->getGlobalPosition();
          zdist = start_height - broadcastGPosition.height;
      }
      std::cout << "I have escaped! at\nzdist: " << zdist << " height: " << broadcastGPosition.height << std::endl;

      // Get current gps coord
      if (!vehicle->isM100() && !vehicle->isLegacyM600())
      {
        subscribeGPosition = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
        start_wp.latitude  = subscribeGPosition.latitude;
        start_wp.longitude = subscribeGPosition.longitude;
        start_wp.altitude  = start_height;
        printf("Waypoint created at (LLA): %f \t%f \t%f\n",
               subscribeGPosition.latitude, subscribeGPosition.longitude,
               start_height);
      }
      else
      {
        broadcastGPosition = vehicle->broadcast->getGlobalPosition();
        start_wp.latitude  = broadcastGPosition.latitude;
        start_wp.longitude = broadcastGPosition.longitude;
        start_wp.altitude  = start_height;
        printf("Waypoint created at (LLA): %f \t%f \t%f\n",
               broadcastGPosition.latitude, broadcastGPosition.longitude,
               start_height);
      }

    
    int fd = 0;
    if(!(fd = open(argv[2], O_RDWR))){
        perror("file open error");
        printf("exiting!\n");
        return 1;
    }

    float64_t lat = 0.0;
    float64_t lon = 0.0;
    float64_t alt = 0.0;
    uint16_t points = 0;

    std::vector<float64_t> latitude;
    std::vector<float64_t> longitude;
    std::vector<float64_t> altitude;

    ssize_t bytes_read = read(fd, &points, sizeof(uint16_t));
    if(bytes_read < 0){
        perror("read error");
    }

    float op_height = 106.8; // 76.2m ~= 350'
    //float op_height = 76.2; // 76.2m ~= 250'
    float64_t alt_off = 0.0;

    printf("points: %u\n", points);

    for(uint16_t i = 0; i < points; i++){
        bytes_read = read(fd, &lat, sizeof(float64_t));
        if(bytes_read < 0){
            perror("read error");
        }
        latitude.push_back(lat);

        bytes_read = read(fd, &lon, sizeof(float64_t));
        if(bytes_read < 0){
            perror("read error");
        }
        longitude.push_back(lon);

        bytes_read = read(fd, &alt, sizeof(float64_t));
        if(bytes_read < 0){
            perror("read error");
        }
        if(!i){
            alt_off = alt;
        }
        altitude.push_back(alt - alt_off + op_height);
        printf("lat: %lf, lon: %lf, alt: %lf\n",lat , lon, alt);
    }

    if(close(fd) < 0){
        perror("what how is this possible file won't close?");
        exit(1);
    }

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

     std::vector<float64_t> altitude{86.2 ,87.2, 114.2, 111.2,
                                     114.2, 82.2, 87.2, 86.2,
                                     86.2, 86.2, 86.2, 86.2,
                                     81.2, 104.2, 114.2, 114.2,
                                     86.2};*/
      
      if (!vehicle->isM100() && !vehicle->isLegacyM600())
      {
         subscribeGPosition = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
         start_wp.latitude  = latitude.at(0) * M_PI / 180;  //subscribeGPosition.latitude;
         start_wp.longitude = longitude.at(0) * M_PI / 180; //subscribeGPosition.longitude;
         start_wp.altitude  = start_height;
         printf("Waypoint created at (LLA): %f \t%f \t%f\n",
                subscribeGPosition.latitude, subscribeGPosition.longitude,
                start_height);
      }
      else
      {
         broadcastGPosition = vehicle->broadcast->getGlobalPosition();
         start_wp.latitude  = broadcastGPosition.latitude;
         start_wp.longitude = broadcastGPosition.longitude;
         start_wp.altitude  = start_height;
         printf("Waypoint created at (LLA): %f \t%f \t%f\n",
                broadcastGPosition.latitude, broadcastGPosition.longitude,
                start_height);
      }

     
     
      std::vector<DJI::OSDK::MyWayPointSettings> my_points = create_waypoints(&start_wp
                  , &longitude, &latitude, &altitude);

      // localOffsetFromGpsOffset(vehicle, localOffset, &my_points(1) ,);
            
      // Sort of there, yes out of maybe?
      req_msg the_msg;
      the_msg.file_type = LWIR;
      the_msg.action = CAPTURE;
      uint16_t image_count = 0;
      //sprintf((char *)the_msg.ascii_msg, "pee pee poo poo");
      
      for(int i = 0; i < my_points.size() /*- 1*/; i++){
          Telemetry::GlobalPosition target;
          Telemetry::GlobalPosition origin;
          broadcastGPosition = vehicle->broadcast->getGlobalPosition();
          std::cout << "Where Are We Now:\nLAT: " << broadcastGPosition.latitude * 180 / M_PI << " LON: " <<
                       broadcastGPosition.longitude * 180 / M_PI << "\nALT: " << broadcastGPosition.altitude << " Height: " <<
                       broadcastGPosition.height << std::endl;

          target.latitude = my_points[i].latitude;
          target.longitude = my_points[i].longitude;
          target.height = my_points[i].altitude;

          /*origin.latitude = my_points[i].latitude;
          origin.longitude = my_points[i].longitude;
          origin.height = my_points[i].altitude;*/



          


          localOffsetFromGpsOffset(vehicle, localOffset, &target, &broadcastGPosition);
          float32_t dist_mag = sqrt(localOffset.x*localOffset.x + localOffset.y*localOffset.y);
          while(dist_mag > 1.0 || std::abs(localOffset.z) > 1.0){
              std::cout << "Where we want to go:\n";
              std::cout << "LAT: " << std::setprecision(9) << my_points.at(i).latitude  * 180 / M_PI << " LON: " << 
                           std::setprecision(9) <<   my_points.at(i).longitude * 180 / M_PI << " Height: " << 
                           std::setprecision(9) << my_points.at(i).altitude
                        << std::endl;
              std::cout << "Where we are:\n";
              std::cout << "LAT: " << std::setprecision(9) << broadcastGPosition.latitude  * 180 / M_PI << " LON: " << 
                           std::setprecision(9) << broadcastGPosition.longitude * 180 / M_PI << " Height: " << 
                           std::setprecision(9) << broadcastGPosition.height <<
                           std::endl;
              std::cout << "xOff: " << localOffset.x << "m yOff: " << localOffset.y << "m zOff: " <<
                           localOffset.z << "m" << std::endl;
              moveByVelocityControl(vehicle, localOffset.x, localOffset.y, localOffset.z, 0, zoff);
              // are we close to where we expect to be?
              broadcastGPosition = vehicle->broadcast->getGlobalPosition();
              localOffsetFromGpsOffset(vehicle, localOffset, &target, &broadcastGPosition);
              dist_mag = sqrt(localOffset.x*localOffset.x + localOffset.y*localOffset.y);
          }
          std::cout << "We should have arived at desired destination" << std::endl;
          if(my_points.at(i).point_type == 1){
              the_msg.gps.lat = broadcastGPosition.latitude;
              the_msg.gps.lon = broadcastGPosition.longitude;
              the_msg.gps.height = broadcastGPosition.height;
              sprintf((char*)the_msg.ascii_msg, "%s_%u", file_d_and_time, image_count);
              // talk to image server and sleep
              make_request("flight_server", 8001, &the_msg);
              image_count++;
              sleep(3);
          }
      }

      //moveByPositionOffset(vehicle, 0, 6, 6, 30);
      //moveByPositionOffset(vehicle, 6, 0, -3, -30);
      //moveByPositionOffset(vehicle, -6, -6, 0, 0);
      monitoredLanding(vehicle);
      break;
    }
    case 'f': {
      /*
      // Prep the bull

      // How hight to start.
      float32_t start_alt = 76.2; // Equal to 250'
      // Create Start Waypoint
      WayPointSettings start_wp;
      setWaypointDefaults(&start_wp);

      // Global position retrieved via subscription
      Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition;
      // Global position retrieved via broadcast
      Telemetry::GlobalPosition broadcastGPosition;
      Telemetry::Vector3f localOffset;

      // Take off first
      monitoredTakeoff(vehicle);
      broadcastGPosition = vehicle->broadcast->getGlobalPosition();
      float64_t zoff = broadcastGPosition.altitude;
      std::cout << "Height: " << broadcastGPosition.height << std::endl;
      std::cout << "zoff: " << zoff << std::endl;
      if(zoff < 0){
          zoff = -1 * zoff;
      } else {
          zoff = 0;
      }

      //start_alt += zoff;

      moveByVelocityControl(vehicle, 0, 0, 72.6, 0, zoff); 
      // Get current gps coord
      if (!vehicle->isM100() && !vehicle->isLegacyM600())
      {
        subscribeGPosition = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
        start_wp.latitude  = subscribeGPosition.latitude;
        start_wp.longitude = subscribeGPosition.longitude;
        start_wp.altitude  = start_alt;
        printf("Waypoint created at (LLA): %f \t%f \t%f\n",
               subscribeGPosition.latitude, subscribeGPosition.longitude,
               start_alt);
      }
      else
      {
        broadcastGPosition = vehicle->broadcast->getGlobalPosition();
        start_wp.latitude  = broadcastGPosition.latitude;
        start_wp.longitude = broadcastGPosition.longitude;
        start_wp.altitude  = start_alt;
        printf("Waypoint created at (LLA): %f \t%f \t%f\n",
               broadcastGPosition.latitude, broadcastGPosition.longitude,
               start_alt);
      }

      std::vector<float64_t> latitude{36.995471, 36.995469, 36.995436, 36.995444,
                                      36.994989, 36.995018, 36.995040, 36.995037,
                                      36.995031, 36.994525, 36.994520, 36.994516,
                                      36.994526, 36.994549, 36.994560, 36.996095,
                                      36.995471};

     std::vector<float64_t> longitude{-122.031533, -122.032399, -122.033324, -122.034252,
                                      -122.034274, -122.033382, -122.032496, -122.031606,
                                      -122.030719, -122.030700, -122.031590, -122.032483,
                                      -122.033381, -122.034240, -122.035117, -122.035119,
                                      -122.031533};

     std::vector<float64_t> altitude{86.2 ,87.2, 114.2, 111.2,
                                     114.2, 82.2, 87.2, 86.2,
                                     86.2, 86.2, 86.2, 86.2,
                                     81.2, 104.2, 114.2, 114.2,
                                     86.2};
      
      if (!vehicle->isM100() && !vehicle->isLegacyM600())
      {
         subscribeGPosition = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
         start_wp.latitude  = latitude.at(0) * M_PI / 180;  //subscribeGPosition.latitude;
         start_wp.longitude = longitude.at(0) * M_PI / 180; //subscribeGPosition.longitude;
         start_wp.altitude  = start_alt;
         printf("Waypoint created at (LLA): %f \t%f \t%f\n",
                subscribeGPosition.latitude, subscribeGPosition.longitude,
                start_alt);
      }
      else
      {
         broadcastGPosition = vehicle->broadcast->getGlobalPosition();
         start_wp.latitude  = broadcastGPosition.latitude;
         start_wp.longitude = broadcastGPosition.longitude;
         start_wp.altitude  = start_alt;
         printf("Waypoint created at (LLA): %f \t%f \t%f\n",
                broadcastGPosition.latitude, broadcastGPosition.longitude,
                start_alt);
      }

     
     
      std::vector<DJI::OSDK::WayPointSettings> my_points = create_waypoints(&start_wp
                  , &longitude, &latitude, &altitude);

      // localOffsetFromGpsOffset(vehicle, localOffset, &my_points(1) ,);
      
      // Sort of there, 10 out of maybe?
      for(int i = 0; i < my_points.size() - 1; i++){
          Telemetry::GlobalPosition target;
          Telemetry::GlobalPosition origin;
          Telemetry::GlobalPosition now = vehicle->broadcast->getGlobalPosition();
          std::cout << "Where Are We Now:\nLAT: " << now.latitude * 180 / M_PI << " LON: " <<
                       now.longitude * 180 / M_PI << "\nALT: " << now.altitude << " Height: " <<
                       now.height << std::endl;

          target.latitude = my_points[i + 1].latitude;
          target.longitude = my_points[i + 1].longitude;
          target.height = my_points[i + 1].altitude;

          origin.latitude = my_points[i].latitude;
          origin.longitude = my_points[i].longitude;
          origin.height = my_points[i].altitude;



          

          localOffsetFromGpsOffset(vehicle, localOffset, &target, &origin);
          std::cout << "Where we want to go:\n";
          std::cout << "LAT: " << my_points.at(i + 1).latitude  * 180 / M_PI << " LON: " << 
                       my_points.at(i + 1).longitude * 180 / M_PI << " Height: " << my_points.at(i + 1).altitude
                    <<  std::endl;
          std::cout << "Where we started:\n";
          std::cout << "LAT: " << my_points.at(i).latitude  * 180 / M_PI << " LON: " << 
                       my_points.at(i).longitude * 180 / M_PI << " Height: " << my_points.at(i).altitude <<
                       std::endl;
          std::cout << "xOff: " << localOffset.x << "m yOff: " << localOffset.y << "m zOff: " <<
                       localOffset.z << "m" << std::endl;


          moveByVelocityControl(vehicle, localOffset.x, localOffset.y, localOffset.z, 0, zoff);
      }

      //moveByPositionOffset(vehicle, 0, 6, 6, 30);
      //moveByPositionOffset(vehicle, 6, 0, -3, -30);
      //moveByPositionOffset(vehicle, -6, -6, 0, 0);
      monitoredLanding(vehicle);
      break;*/
    }
    default:
      break;
  }

  return 0;
}
//moveByVelocityControl
