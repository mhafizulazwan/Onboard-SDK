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
  WaypointV2 startPoint;
  WaypointV2 endPoint0;
  WaypointV2 endPoint1;
  WaypointV2 endPoint2;

  // Define startpoint.
  startPoint.latitude  = GPosition[0].latitude; 
  startPoint.longitude = GPosition[0].longitude; 
  startPoint.relativeHeight = 20;
  prsb->setWaypointV2Defaults(startPoint);
  waypointList.push_back(startPoint);
  
  DSTATUS("Start point latitude:%f",GPosition[0].latitude);
  DSTATUS("Start point longitude:%f",GPosition[0].longitude);

  // Define Cartesian coordinates.
  // float32_t radius = 6;
  // float32_t X = radius * cos(0);
  // float32_t Y = radius * sin(0);

  // Define endpoint: convert Cartesian to GPS coordinates.
  // endPoint0.latitude = X/EARTH_RADIUS + startPoint.latitude;
  // endPoint0.longitude = Y/(EARTH_RADIUS * cos(startPoint.latitude)) + startPoint.longitude;
  // endPoint0.relativeHeight = 15;
  // prsb->setWaypointV2Defaults(endPoint0);
  // waypointList.push_back(endPoint0);
  
  endPoint1.latitude = 0.051875;
  endPoint1.longitude = 1.77572;
  endPoint1.relativeHeight = 20;
  prsb->setWaypointV2Defaults(endPoint1);
  waypointList.push_back(endPoint1);

  DSTATUS("End point latitude:%f",endPoint1.latitude);
  DSTATUS("End point longitude:%f",endPoint1.longitude);

  endPoint2.latitude = 0.051875;
  endPoint2.longitude = 1.77572;
  endPoint2.relativeHeight = 12;
  prsb->setWaypointV2Defaults(endPoint2);
  waypointList.push_back(endPoint2);
  
  /*Initialize the mission */ 
  DJIWaypointV2MissionFinishedAction finishedAction;
  finishedAction = DJIWaypointV2MissionFinishedGoHome;
  prsb->initMissionSetting(timeout,waypointList,finishedAction);
  
  /*! Setup Micasense*/
  // ImageCapture micasense;

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
  // waypointList.push_back(endPoint2);
  // waypointList.push_back(endPoint1);

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