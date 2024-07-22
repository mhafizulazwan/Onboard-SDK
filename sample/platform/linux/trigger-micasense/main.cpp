/*! @file missions/waypoint_v2_main.cpp
 *  @version 4.0.0
 *  @date Mar 07 2019
 *
 *  @brief
 *  main for Waypoint Missions V2 API usage in a Linux environment.
 *  Shows example usage of the Waypoint Missions through
 *  the Mission Manager API.
 *
 *  @Copyright (c) 2019 DJI
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

#include "prsb_algae_mission.hpp"
#include "micasense.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int
main(int argc, char** argv)
{
  /*! Initialize variables*/
  int timeout = 1;
  
  /*! Setup OSDK.*/
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  /*! Obtain Control Authority*/
  vehicle->control->obtainCtrlAuthority(timeout);

  auto *prsb = new PrsbAlgaeMission(vehicle);

  /*! Setup subscription*/
  if (!prsb->setUpSubscription(timeout))
  {
    DERROR("Failed to set up subscription!");
    return -1;
  }
  else
  {
    DSTATUS("Set up subscription successfully!");
  }
  sleep(timeout);

  /*! Define waypoints.*/
  std::vector<WaypointV2> GPosition;
  GPosition = prsb->getGPosition();
  
  // Create a vector to store waypoints in.
  std::vector<WaypointV2> waypointList;
  WaypointV2 waypoint0;
  WaypointV2 waypoint1;
  WaypointV2 waypoint2;
  WaypointV2 waypoint3;
  WaypointV2 waypoint4;
  WaypointV2 waypointTest;

  // Define waypoint0.
  waypoint0.latitude  = GPosition[0].latitude; 
  waypoint0.longitude = GPosition[0].longitude; 
  waypoint0.relativeHeight = 15;
  prsb->setWaypointV2Defaults(waypoint0);
  waypointList.push_back(waypoint0);
  
  DSTATUS("Start point latitude:%f",GPosition[0].latitude);
  DSTATUS("Start point longitude:%f",GPosition[0].longitude);
  
   // Define waypoint1.
  waypoint1.latitude = 0.051875; // get waypoint latitude behind tank;
  waypoint1.longitude = 1.77572; // get waypoint longitude behind tank;
  waypoint1.relativeHeight = 15;
  prsb->setWaypointV2Defaults(waypoint1);
  waypointList.push_back(waypoint1);

  // Define waypoint2.
  waypoint2.latitude = 0.051875;
  waypoint2.longitude = 1.77572;
  waypoint2.relativeHeight = 15;
  prsb->setWaypointV2Defaults(waypoint2);
  waypointList.push_back(waypoint2);

  DSTATUS("Tank point latitude:%f",waypoint2.latitude);
  DSTATUS("Tank point longitude:%f",waypoint2.longitude);

  // Define waypoint3.
  waypoint3.latitude = 0.051875;
  waypoint3.longitude = 1.77572;
  waypoint3.relativeHeight = 12;
  prsb->setWaypointV2Defaults(waypoint3);
  waypointList.push_back(waypoint3);

  // Define waypoint4.
  waypoint4.latitude = 0.051875;
  waypoint4.longitude = 1.77572;
  waypoint4.relativeHeight = 15;
  prsb->setWaypointV2Defaults(waypoint4);
  waypointList.push_back(waypoint4);

  // Define waypointTest
  // float32_t radius = 6;
  // float32_t X = radius * cos(0);
  // float32_t Y = radius * sin(0);

  // convert Cartesian to GPS coordinates.
  // waypointTest.latitude = X/EARTH_RADIUS + waypoint0.latitude;
  // waypointTest.longitude = Y/(EARTH_RADIUS * cos(waypoint0.latitude)) + waypoint0.longitude;
  // waypointTest.relativeHeight = 15;
  // prsb->setWaypointV2Defaults(waypointTest);
  // waypointList.push_back(waypointTest);
  
  /*Initialize the mission */ 
  DJIWaypointV2MissionFinishedAction finishedAction;
  finishedAction = DJIWaypointV2MissionFinishedGoHome;
  prsb->initMissionSetting(timeout,waypointList,finishedAction);
  
  /*! run a new WaypointV2 mission prsb*/
  ErrorCode::ErrorCodeType ret;
  ret = prsb->runPrsbAlgaeMission();
  if(ret == ErrorCode::SysCommonErr::Success)
  {
    // micasense.captureAndSyncImages();
    // sleep(timeout);
    // DSTATUS("Mission completed successfully!");
  }

  // waypointList.clear();

  /*! Define waypoints.*/
  // waypointList.push_back(waypoint4);
  // waypointList.push_back(waypoint2);

  // finishedAction = DJIWaypointV2MissionFinishedGoHome;
  // prsb->initMissionSetting(timeout,waypointList,finishedAction);
  
  // /*! run a new WaypointV2 mission prsb*/
  // ret = prsb->runPrsbAlgaeMission();
  // if(ret == ErrorCode::SysCommonErr::Success)
  //   micasense.captureAndSyncImages();
  // sleep(timeout);

  delete(prsb);

  // Mission will continue when we exit here
  return 0;
}