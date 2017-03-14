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

#ifndef SIM_RHINO_BODY_H
#define SIM_RHINO_BODY_H

#include <memory>
#include <string>

namespace opendlv {
namespace sim {
namespace rhino {

/**
 * Simulated Rhino body.
 */
class Body {
   public:
    Body();
    Body(Body const &) = delete;
    Body &operator=(Body const &) = delete;
    virtual ~Body();
// Example
//    double GetX() const;
    void Update(float);
    void Test();
   private:
// Example
//    double m_x;
//    double m_u;

//Parameters for Rhino
// m_cg_to_firstaxle(1.68f);
// m_cg_to_secondaxle(1.715f);
// m_cornering_stiffness_front(278631.0f);
// m_cornering_stiffness_rear(155628.0);
// m_inertia(41340.0f);
// m_mass(9840.0f);
// m_steering_ratio(15.6f);
//
// float const a = m_cg_to_firstaxle;
// float const b = m_cg_to_secondaxle;
// float const c_f = m_cornering_stiffness_front;
// float const c_r = m_cornering_stiffness_rear;
//
// float psi = m_heading;
// float x = m_lonigtudinal_position;
// float y = m_lateral_position;
// float vx = longitudinal_velocity;
// float vy = lateral velocity;
// float r = yaw_rate;
// float delta = a_swa/m_steering_ratio;
//
// double motion_angle_front = (-a*r - vy)/vx;
// double motion_angle_rear = (b*r - vy)/vx;
//
// double rdot  = (-b*c_r*motion_angle_rear + a*c_f*motion_angle_front + a*c_f*delta)/m_inertia;
//
// double vydot = (c_r*motion_angle_rear + c_f*motion_angle_front + c_f*delta)/m_mass - u*r;

};

}
}
}

#endif
