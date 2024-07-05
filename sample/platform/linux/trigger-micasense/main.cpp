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

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int
main(int argc, char** argv)
{
  /*! Initialize variables*/

  int functionTimeout = 1;
  /*! Setup OSDK.*/
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle*   vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }
  int responseTimeout = 1;

  /*! Obtain Control Authority*/
  vehicle->control->obtainCtrlAuthority(functionTimeout);

  
  auto *prsb = new PrsbAlgaeMission(vehicle);

  /*! Setup Subscription*/
  int timeout = 3;
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

  // Get GPS position.
  std::vector<WaypointV2> GPosition;
  GPosition = prsb->getGPosition();
  
  // Create a vector to store waypoints in.
  std::vector<WaypointV2> waypointList;
  WaypointV2 startPoint;
  WaypointV2 endPoint;

  // Define startpoint.
  startPoint.latitude  = GPosition[0].latitude; 
  startPoint.longitude = GPosition[0].longitude; 
  startPoint.relativeHeight = 15;
  prsb->setWaypointV2Defaults(startPoint);
  waypointList.push_back(startPoint);
  
  DSTATUS("Latitude start:%f",GPosition[0].latitude);
  DSTATUS("Longitude start:%f",GPosition[0].longitude);

  // Define Cartesian coordinates.
  float32_t radius = 6;
  float32_t X = radius * cos(0);
  float32_t Y = radius * sin(0);

  // Define endpoint: convert Cartesian to GPS coordinates.
  endPoint.latitude = X/EARTH_RADIUS + startPoint.latitude;
  endPoint.longitude = Y/(EARTH_RADIUS * cos(startPoint.latitude)) + startPoint.longitude;
  endPoint.relativeHeight = startPoint.relativeHeight;
  prsb->setWaypointV2Defaults(endPoint);
  waypointList.push_back(endPoint);

  DSTATUS("Latitude end:%f",endPoint.latitude);
  DSTATUS("Longitude end:%f",endPoint.longitude);
  
  /*Let's define what the drone will do once finished the action 
    options: DJIWaypointV2MissionFinishedGoHome, DJIWaypointV2MissionFinishedNoAction */ 
  DJIWaypointV2MissionFinishedAction finishedAction = DJIWaypointV2MissionFinishedGoHome;
  
  /*! run a new WaypointV2 mission prsb*/
  prsb->runPrsbAlgaeMission(waypointList, finishedAction);

  delete(prsb);

  // Mission will continue when we exit here
  return 0;
}