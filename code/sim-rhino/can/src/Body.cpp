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
#include <iostream>

using namespace std;

namespace opendlv {
namespace sim {
namespace rhino {

Body::Body()
{
}

Body::~Body()
{
}

// Example
//double Body::GetX() const
//{
//  return m_x;
//}

void Body::Update(float a_deltaTime)
{
  cout << "Hello" << endl;
  (void) a_deltaTime;

  // Remove later.
 // Update the state of body...

  // Example
//   m_x = m_x + xdot * a_deltaTime;
//   m_u = m_u + udot * a_deltaTime;
}

//I tried with another function

void Body::Test()
{
  cout << "Hello1" << endl; 
}

}
}
}
