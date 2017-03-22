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

#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/data/TimeStamp.h>

#include <opendavinci/odcore/io/tcp/TCPFactory.h>

#include <odvdrhino/GeneratedHeaders_ODVDRhino.h>

#include "Cloud.h"

namespace opendlv {
namespace proxy {
namespace rhino {

Cloud::Cloud(const int &argc, char **argv)
  : TimeTriggeredConferenceClientModule(argc, argv, "proxy-rhino-cloud")
  , m_outbound()
  , m_connection()
{
}

Cloud::~Cloud() 
{
}

void Cloud::nextContainer(odcore::data::Container &a_container)
{
  int32_t id = a_container.getDataType();
  std::string idStr = std::to_string(id);

  if (id == opendlv::proxy::rhino::ManualControl::ID()) {
    auto manualControl = 
      a_container.getData<opendlv::proxy::rhino::ManualControl>();
    m_outbound[idStr + "-1"] = 
      std::to_string(manualControl.getAccelerationPedalPosition()); 
    m_outbound[idStr + "-2"] = 
      std::to_string(manualControl.getBrakePedalPosition()); 
    m_outbound[idStr + "-3"] = 
      std::to_string(manualControl.getTorsionBarTorque()); 
  }
}

void Cloud::setUp() 
{
  std::string const receiver = "91.106.193.119";
  uint32_t const port = 80;

  std::cout << "Connecting to " << receiver << ":" << port << std::endl;

  m_connection = odcore::io::tcp::TCPFactory::createTCPConnectionTo(receiver, 
      port);
  m_connection->setRaw(true);

  updateValues("200-3=54");
}

void Cloud::tearDown() 
{
}

void Cloud::updateValues(std::string const &a_values)
{
  m_connection->send("GET /cloud/upload.php?" + a_values + " HTTP/1.1\nhost: opendlv.org\n\n");

  std::cout << "Sent..." << std::endl;
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Cloud::body()
{
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() 
      == odcore::data::dmcp::ModuleStateMessage::RUNNING) {




  }

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

}
}
} 
