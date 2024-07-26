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
  WaypointV2 waypointTest;

  // Define waypoint0.
  waypoint0.latitude  = GPosition[0].latitude; 
  waypoint0.longitude = GPosition[0].longitude; 
  waypoint0.relativeHeight = 15;
  waypoint0.config.useLocalCruiseVel = 0;
  waypoint0.autoFlightSpeed = 1;
  waypoint0.headingMode = DJIWaypointV2HeadingWaypointCustom;
  waypoint0.heading = -67; // -180 to 180: 0 True North (-67 facing tank)
  waypoint0.turnMode = DJIWaypointV2TurnModeCounterClockwise;
  prsb->setWaypointV2Defaults(waypoint0);
  waypointList.push_back(waypoint0);
  
  DSTATUS("Start point latitude:%f",GPosition[0].latitude);
  DSTATUS("Start point longitude:%f",GPosition[0].longitude);
  
  // Define waypoint1.
  waypoint1.latitude = 0.0518756;
  waypoint1.longitude = 1.77572;
  waypoint1.relativeHeight = 15;
  waypoint1.config.useLocalCruiseVel = 0;
  waypoint1.autoFlightSpeed = 1.5;
  waypoint1.headingMode = DJIWaypointV2HeadingWaypointCustom;
  waypoint1.heading = -155; // -180 to 180: 0 True North (-155 align with tank)
  waypoint1.turnMode = DJIWaypointV2TurnModeCounterClockwise;
  prsb->setWaypointV2Defaults(waypoint1);
  waypointList.push_back(waypoint1);

  DSTATUS("Tank point latitude:%f",waypoint1.latitude);
  DSTATUS("Tank point longitude:%f",waypoint1.longitude);

  // Define waypoint2.
  waypoint2.latitude = 0.0518756;
  waypoint2.longitude = 1.77572;
  waypoint2.relativeHeight = 12;
  waypoint2.config.useLocalCruiseVel = 0;
  waypoint2.autoFlightSpeed = 0.8;
  waypoint2.headingMode = DJIWaypointV2HeadingWaypointCustom;
  waypoint2.heading = -155; // -180 to 180: 0 True North (-155 align with tank)
  waypoint2.turnMode = DJIWaypointV2TurnModeCounterClockwise;
  prsb->setWaypointV2Defaults(waypoint2);
  waypointList.push_back(waypoint2);

  // Define waypoint3.
  waypoint3.latitude = 0.0518756;
  waypoint3.longitude = 1.77572;
  waypoint3.relativeHeight = 15;
  waypoint3.config.useLocalCruiseVel = 0;
  waypoint3.autoFlightSpeed = 1;
  waypoint3.headingMode = DJIWaypointV2HeadingWaypointCustom;
  waypoint3.heading = 114; // -180 to 180: 0 True North (114 facing home)
  waypoint3.turnMode = DJIWaypointV2TurnModeCounterClockwise;
  prsb->setWaypointV2Defaults(waypoint3);
  waypointList.push_back(waypoint3);

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
  // waypointList.push_back(waypoint3);
  // waypointList.push_back(waypoint1);

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