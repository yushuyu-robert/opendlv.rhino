/**
 * Copyright (C) 2016 Chalmers Revere
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <iostream>

#include <opendavinci/odcore/base/Lock.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/data/TimeStamp.h>

#include <opendavinci/odcore/io/tcp/TCPFactory.h>

#include <odvdrhino/GeneratedHeaders_ODVDRhino.h>
#include <odvdautomotivedata/GeneratedHeaders_ODVDAutomotiveData.h>

#include "Cloud.h"

namespace opendlv {
namespace proxy {
namespace rhino {

Cloud::Cloud(const int &argc, char **argv)
  : TimeTriggeredConferenceClientModule(argc, argv, "proxy-rhino-cloud")
  , m_mutex()
  , m_outbound()
  , m_hostname()
  , m_port()
{
    m_outbound["19-1"] = "57.70916";
    m_outbound["19-2"] = "11.94781";
    
    m_outbound["191-1"] = "0.0";
    m_outbound["191-2"] = "0.0";
    m_outbound["191-3"] = "0.0";
    
    m_outbound["195-1"] = "0.0";
    m_outbound["195-2"] = "0.0";
    m_outbound["195-3"] = "0.0";
    
    m_outbound["196-1"] = "0.0";
    
    m_outbound["197-1"] = "0.0";
    m_outbound["197-2"] = "0.0";
    m_outbound["197-3"] = "0.0";
    
    m_outbound["198-1"] = "0.0";
    m_outbound["198-2"] = "0.0";
    m_outbound["198-3"] = "0.0";
    m_outbound["198-4"] = "0.0";
    m_outbound["198-5"] = "0.0";
    m_outbound["198-6"] = "0.0";
    
    m_outbound["199-1"] = "0.0";
    m_outbound["199-2"] = "0.0";
    
    m_outbound["200-1"] = "0.0";
    m_outbound["200-2"] = "0.0";
    m_outbound["200-3"] = "0.0";
}

Cloud::~Cloud() 
{
}

void Cloud::nextContainer(odcore::data::Container &a_container)
{
  odcore::base::Lock l(m_mutex);

  int32_t id = a_container.getDataType();
  std::string idStr = std::to_string(id);

  if (id == geodetic::WGS84::ID()) {
    auto wgs84 = a_container.getData<geodetic::WGS84>();
    m_outbound[idStr + "-1"] = std::to_string(wgs84.getLatitude()); 
    m_outbound[idStr + "-2"] = std::to_string(wgs84.getLongitude()); 
  } else if (id == opendlv::proxy::rhino::ManualControl::ID()) {
    auto manualControl = 
      a_container.getData<opendlv::proxy::rhino::ManualControl>();
    m_outbound[idStr + "-1"] = 
      std::to_string(manualControl.getAccelerationPedalPosition()); 
    m_outbound[idStr + "-2"] = 
      std::to_string(manualControl.getBrakePedalPosition()); 
    m_outbound[idStr + "-3"] = 
      std::to_string(manualControl.getTorsionBarTorque()); 
  } else if (id == opendlv::proxy::rhino::Axles::ID()) {
    auto axles = a_container.getData<opendlv::proxy::rhino::Axles>();
    m_outbound[idStr + "-1"] = std::to_string(axles.getLoadAxle11()); 
    m_outbound[idStr + "-2"] = std::to_string(axles.getLoadAxle12()); 
    m_outbound[idStr + "-3"] = std::to_string(axles.getLoadAxle13()); 
  } else if (id == opendlv::proxy::rhino::Propulsion::ID()) {
    auto propulsion = a_container.getData<opendlv::proxy::rhino::Propulsion>();
    m_outbound[idStr + "-1"] = std::to_string(
        propulsion.getPropulsionShaftVehicleSpeed()); 
  } else if (id == opendlv::proxy::rhino::VehicleState::ID()) {
    auto vehicleState = 
      a_container.getData<opendlv::proxy::rhino::VehicleState>();
    m_outbound[idStr + "-1"] = std::to_string(vehicleState.getAccelerationX()); 
    m_outbound[idStr + "-2"] = std::to_string(vehicleState.getAccelerationY()); 
    m_outbound[idStr + "-3"] = std::to_string(vehicleState.getYawRate()); 
  } else if (id == opendlv::proxy::rhino::Wheels::ID()) {
    auto wheels = a_container.getData<opendlv::proxy::rhino::Wheels>();
    m_outbound[idStr + "-1"] = std::to_string(wheels.getSpeedWheel111()); 
    m_outbound[idStr + "-2"] = std::to_string(wheels.getSpeedWheel112()); 
    m_outbound[idStr + "-3"] = std::to_string(wheels.getSpeedWheel121()); 
    m_outbound[idStr + "-4"] = std::to_string(wheels.getSpeedWheel122()); 
    m_outbound[idStr + "-5"] = std::to_string(wheels.getSpeedWheel131()); 
    m_outbound[idStr + "-6"] = std::to_string(wheels.getSpeedWheel132()); 
  } else if (id == opendlv::proxy::rhino::Steering::ID()) {
    auto steering = a_container.getData<opendlv::proxy::rhino::Steering>();
    m_outbound[idStr + "-1"] = std::to_string(steering.getRoadWheelAngle()); 
    m_outbound[idStr + "-2"] = std::to_string(steering.getSteeringWheelAngle()); 
  } else if (id == opendlv::proxy::rhino::Driveline::ID()) {
    auto driveline = a_container.getData<opendlv::proxy::rhino::Driveline>();
    m_outbound[idStr + "-1"] = std::to_string(driveline.getEngineTorque()); 
    m_outbound[idStr + "-2"] = std::to_string(driveline.getEngineSpeed()); 
    m_outbound[idStr + "-3"] = std::to_string(driveline.getCurrentGear()); 
  }
}

void Cloud::setUp() 
{
  m_hostname = getKeyValueConfiguration().getValue<std::string>(
      "proxy-rhino-cloud.hostname");
  m_port = getKeyValueConfiguration().getValue<uint32_t>(
      "proxy-rhino-cloud.port");
}

void Cloud::tearDown() 
{
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Cloud::body()
{
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() 
      == odcore::data::dmcp::ModuleStateMessage::RUNNING) {

    odcore::base::Lock l(m_mutex);

    auto connection = odcore::io::tcp::TCPFactory::createTCPConnectionTo(
        m_hostname, m_port);
    connection->setRaw(true);

    bool first = true;
    std::string getString = "";
    for (auto const &entry : m_outbound) {
      if (!first) {
        getString += "&";
      }
      getString += entry.first + "=" + entry.second;
      first = false;
    }

    std::string http = "GET /cloud/upload.php?" + getString + " HTTP/1.1\nhost: opendlv.org\n\n";

    connection->send(http);

    std::cout << "Sent: " << http << std::endl;
  }

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

}
}
} 
