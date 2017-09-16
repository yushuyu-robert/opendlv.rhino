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

#include "Powertrain.h"

namespace opendlv {
namespace sim {
namespace rhino {

Powertrain::Powertrain():
  m_acceleratorPedalPosition(0.0),
  m_engineSpeed(0.0),
  m_engineTorque(0.0),
  m_gear(0)
{
}

Powertrain::~Powertrain()
{
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
  return m_gear;
}

void Powertrain::Update(double a_deltaTime)
{
  (void) a_deltaTime;
}

}
}
} 
