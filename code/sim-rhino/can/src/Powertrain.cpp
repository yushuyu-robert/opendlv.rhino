/**
 * Copyright (C) 2017 Chalmers Revere
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

#include <algorithm>
#include <iostream>
#include <utility>
#include <vector>

#include "Powertrain.h"

namespace opendlv {
namespace sim {
namespace rhino {

Powertrain::Powertrain():
  m_acceleratorPedalPosition(0.0),
  m_engineSpeed(0.0),
  m_engineTorque(0.0)
{
}

Powertrain::~Powertrain()
{
}

double Powertrain::CalcEngineMaxTorque() const
{
  std::vector<std::pair<double, double>> const torqueLookupTable = {
    std::pair<double, double>(0.0, 0.0),
    std::pair<double, double>(62.8318, 1660.0),
    std::pair<double, double>(73.3038, 1880.0),
    std::pair<double, double>(83.775, 2240.0),
    std::pair<double, double>(94.247, 2900.0),
    std::pair<double, double>(104.719, 3550.0),
    std::pair<double, double>(115.191, 3550.0),
    std::pair<double, double>(125.663, 3550.0),
    std::pair<double, double>(136.135, 3550.0),
    std::pair<double, double>(146.607, 3550.0),
    std::pair<double, double>(157.079, 3470.0),
    std::pair<double, double>(167.551, 3310.0),
    std::pair<double, double>(178.023, 3120.0),
    std::pair<double, double>(188.495, 2880.0),
    std::pair<double, double>(198.967, 2660.0 ),
    std::pair<double, double>(209.439, 1680.0),
    std::pair<double, double>(219.911, 0.0)
  };

  if (m_engineSpeed < torqueLookupTable[0].first) {
    return 0.0;
  }

  if (m_engineSpeed > torqueLookupTable[torqueLookupTable.size() - 1].first) {
    return 0.0;
  }

  for (uint32_t i = 0; i < torqueLookupTable.size() - 2; i++) {
    double const x1 = torqueLookupTable[i].first;
    double const x2 = torqueLookupTable[i+1].first;

    if (m_engineSpeed >= x1 && m_engineSpeed < x2) {
      double const r = (m_engineSpeed - x1) / (x2 - x1);

      double const y1 = torqueLookupTable[i].second;
      double const y2 = torqueLookupTable[i+1].second;

      double const maxTorque = y1 + r * (y2 - y1);
      return maxTorque;
    }
  }

  std::cerr << "Lookup failed. This should never happen." << std::endl;
  return 0.0;
}

double Powertrain::GetAcceleratorPedalPosition() const
{
  return m_acceleratorPedalPosition;
}
    
double Powertrain::GetEngineSpeed() const
{
  return m_engineSpeed;
}

double Powertrain::GetEngineTorque() const
{
  return m_engineTorque;
}

int32_t Powertrain::GetGear() const
{
  int32_t const gear = 6;
  return gear;
}

void Powertrain::Update(double a_deltaTime, double a_longitudinalAcceleration)
{
  double const engineIdleSpeed = 600.0 * 3.14 / 30.0;
  double const accelerationUpperLimit = 1.35;
  double const accelerationLowerLimit = 1.1;
  double const eFactor = 0.5;
  double const torqueFilterCoefficient = 0.5;

  m_engineSpeed = engineIdleSpeed;

  double const engineMaxTorque = CalcEngineMaxTorque();

  double const throttle = std::max(
      std::min(m_acceleratorPedalPosition / 100.0, 1.0), 0.0);

  double const rawRequestedTorque = throttle * engineMaxTorque;
  double accelerationLimitedTorque;
  if (a_longitudinalAcceleration < accelerationLowerLimit) {
    accelerationLimitedTorque = rawRequestedTorque;
  } else if (a_longitudinalAcceleration > accelerationUpperLimit) {
    accelerationLimitedTorque = eFactor * rawRequestedTorque;
  } else {
    double const r = (a_longitudinalAcceleration - accelerationLowerLimit) /
      (accelerationUpperLimit - accelerationLowerLimit);
    double eFactorInterpolated = r * eFactor;
    accelerationLimitedTorque = eFactorInterpolated * rawRequestedTorque;
  }

  double const engineTorqueRate = torqueFilterCoefficient *
    (accelerationLimitedTorque - m_engineTorque);
  m_engineTorque = m_engineTorque + engineTorqueRate * a_deltaTime;
}

void Powertrain::SetAcceleratorPedalPosition(double a_acceleratorPedalPosition)
{
  m_acceleratorPedalPosition = a_acceleratorPedalPosition;
}

}
}
} 
