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

#include "Body.h"
#include "Driveline.h"
#include "Vehicle.h"
#include "Wheels.h"

namespace opendlv {
namespace sim {
namespace rhino {

Vehicle::Vehicle()
  : m_body(new Body)
  , m_driveline(new Driveline)
  , m_wheels(new Wheels)
{
}

Vehicle::~Vehicle()
{
}

opendlv::proxy::rhino::Axles Vehicle::GetAxles() const
{
  double loadAxle11 = 0.0;
  double loadAxle12 = 0.0;
  double loadAxle13 = 0.0;
  odcore::data::TimeStamp fromSensor;

  opendlv::proxy::rhino::Axles axles(loadAxle11, loadAxle12, loadAxle13,
      fromSensor);
  return axles;
}

opendlv::proxy::rhino::Driveline Vehicle::GetDriveline() const
{
	float engineTorque = 0.0f;
	float engineSpeed = 0.0f;
	int8_t currentGear = 0;

  opendlv::proxy::rhino::Driveline driveline(engineTorque, engineSpeed,
      currentGear);
  return driveline;
}

opendlv::proxy::GroundSpeedReading Vehicle::GetGroundSpeedReading() const
{
  double groundSpeed = 0.0;

  opendlv::proxy::GroundSpeedReading groundSpeedReading(groundSpeed);
  return groundSpeedReading;
}

opendlv::coord::KinematicState Vehicle::GetKinematicState() const
{
  double const vx = 0.0;
  double const vy = 0.0;
  double const vz = 0.0;
  double const rollRate = 0.0;
  double const pitchRate = 0.0;
  double const yawRate = 0.0;
 
  opendlv::coord::KinematicState const kinematicState(vx, vy, vz, rollRate,
      pitchRate, yawRate);
  return kinematicState;
}

opendlv::proxy::rhino::ManualControl Vehicle::GetManualControl() const
{
  double accelerationPedalPosition = 0.0;
  double brakePedalPosition = 0.0;
  double torsionBarTorque = 0.0;
  odcore::data::TimeStamp fromSensor;

  opendlv::proxy::rhino::ManualControl manualControl(accelerationPedalPosition,
      brakePedalPosition, torsionBarTorque, fromSensor);
  return manualControl;
}

opendlv::proxy::rhino::Propulsion Vehicle::GetPropulsion() const
{
  double propulsionShaftVehicleSpeed = 0.0;
  odcore::data::TimeStamp fromSensor;

  opendlv::proxy::rhino::Propulsion propulsion(propulsionShaftVehicleSpeed,
      fromSensor);
  return propulsion;
}

opendlv::proxy::rhino::Steering Vehicle::GetSteering() const
{
  double roadWheelAngle = 0.0;
  double steeringWheelAngle = 0.0;
  odcore::data::TimeStamp fromSensor;

  opendlv::proxy::rhino::Steering steering(roadWheelAngle, steeringWheelAngle,
      fromSensor);
  return steering;
}

opendlv::proxy::rhino::VehicleState Vehicle::GetVehicleState() const
{
  double accelerationX = 0.0;
  double accelerationY = 0.0;
  double yawRate = 0.0;
  odcore::data::TimeStamp fromSensor;

  opendlv::proxy::rhino::VehicleState vehicleState(accelerationX, accelerationY,
      yawRate, fromSensor);
  return vehicleState;
}

opendlv::proxy::rhino::Wheels Vehicle::GetWheels() const
{
  double speedWheel111 = 0.0;
  double speedWheel112 = 0.0;
  double speedWheel121 = 0.0;
  double speedWheel122 = 0.0;
  double speedWheel131 = 0.0;
  double speedWheel132 = 0.0;
  odcore::data::TimeStamp fromSensor;

  opendlv::proxy::rhino::Wheels wheels(speedWheel111, speedWheel112,
      speedWheel121, speedWheel122, speedWheel131, speedWheel132, fromSensor);
  return wheels;
}

void Vehicle::SetAccelerationRequest(
    opendlv::proxy::rhino::AccelerationRequest a_accelerationRequest)
{
  bool enableRequest = a_accelerationRequest.getEnableRequest();
  if (enableRequest) {
    double accelerationPedalPosition =
      a_accelerationRequest.getAccelerationPedalPosition();

    (void) accelerationPedalPosition; // Since not used.. Remove later.
  }
}

void Vehicle::SetBrakeRequest(
    opendlv::proxy::rhino::BrakeRequest a_brakeRequest)
{
  bool enableRequest = a_brakeRequest.getEnableRequest();
  if (enableRequest) {
    double deceleration = a_brakeRequest.getDeceleration();

    (void) deceleration; // Since not used.. Remove later.
  }
}

void Vehicle::SetSteeringRequest(
    opendlv::proxy::rhino::SteeringRequest a_steeringRequest)
{
  bool enableRequest = a_steeringRequest.getEnableRequest();
  if (enableRequest) {
    double roadWheelAngle = a_steeringRequest.getRoadWheelAngle();
    double deltaTorque = a_steeringRequest.getDeltaTorque();

    (void) roadWheelAngle; // Since not used.. Remove later.
    (void) deltaTorque; // Since not used.. Remove later.
  }
}

void Vehicle::Update(double a_deltaTime)
{
  (void) a_deltaTime; // Since not used.. Remove later.

  // m_body->Update(a_deltaTime);
  // m_driveline->Update(a_deltaTime);
  // m_wheels->Update(a_deltaTime);
}

}
}
}
