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
    double GetLateralAcceleration() const;
    double GetLateralVelocity() const;
    double GetLongitudinalAcceleration() const;
    double GetLongitudinalVelocity() const;
    double GetYawAcceleration() const;
    double GetYawVelocity() const;
    void Update(double);

   private:
    double m_lateralAcceleration;
    double m_lateralVelocity;
    double m_longitudinalAcceleration;
    double m_longitudinalVelocity;
    double m_yawAcceleration;
    double m_yawVelocity;
};

}
}
}

#endif
