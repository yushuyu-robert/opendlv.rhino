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

#include "Body.h"

namespace opendlv {
namespace sim {
namespace rhino {

Body::Body():
  m_lateralAcceleration(0.0),
  m_lateralVelocity(0.0),
  m_longitudinalAcceleration(0.0),
  m_longitudinalVelocity(0.0),
  m_yawAcceleration(0.0),
  m_yawVelocity(0.0)
{
}

Body::~Body()
{
}

double Body::GetLateralAcceleration() const
{
  return m_lateralAcceleration;
}

double Body::GetLateralVelocity() const
{
  return m_lateralVelocity;
}

double Body::GetLongitudinalAcceleration() const
{
  return m_longitudinalAcceleration;
}

double Body::GetLongitudinalVelocity() const
{
  return m_longitudinalVelocity;
}

double Body::GetYawAcceleration() const
{
  return m_yawAcceleration;
}

double Body::GetYawVelocity() const
{
  return m_yawVelocity;
}

void Body::Update(double a_deltaTime)
{
  (void) a_deltaTime;
}

}
}
}
