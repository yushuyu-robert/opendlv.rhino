/**
 * Copyright (C) 2017 Ola Benderius
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

#ifndef SIM_RHINO_VEHICLE_H
#define SIM_RHINO_VEHICLE_H

#include <memory>
#include <string>

#include <odvdopendlvstandardmessageset/GeneratedHeaders_ODVDOpenDLVStandardMessageSet.h>
#include <odvdrhino/GeneratedHeaders_ODVDRhino.h>

namespace opendlv {
namespace sim {
namespace rhino {

//class Body;
//class Powertrain;
//class Wheels;
class dynamics;

/**
 * Simulated Rhino.
 */
class Vehicle {
  public:
    Vehicle();
    Vehicle(Vehicle const &) = delete;
    Vehicle &operator=(Vehicle const &) = delete;
    virtual ~Vehicle();
    opendlv::proxy::rhino::Axles GetAxles() const;
    opendlv::proxy::rhino::Driveline GetDriveline() const;
    opendlv::proxy::GroundSpeedReading GetGroundSpeedReading() const;
    //opendlv::coord::KinematicState GetKinematicState() const;
    opendlv::proxy::rhino::ManualControl GetManualControl() const;
    opendlv::proxy::rhino::Propulsion GetPropulsion() const;
    opendlv::proxy::rhino::Steering GetSteering() const;
    opendlv::proxy::rhino::VehicleState GetVehicleState() const;
    opendlv::proxy::rhino::Wheels GetWheels() const;
    void SetAccelerationRequest(opendlv::proxy::rhino::AccelerationRequest);
    void SetBrakeRequest(opendlv::proxy::rhino::BrakeRequest);
    void SetSteeringRequest(opendlv::proxy::rhino::SteeringRequest);
    void Update(double);

  private:
   // std::unique_ptr<Body> m_body;
   // std::unique_ptr<Powertrain> m_powertrain;
   // std::unique_ptr<Wheels> m_wheels;
    std::unique_ptr<dynamics> m_dynamics;
};

}
}
}

#endif
