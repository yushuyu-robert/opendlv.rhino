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

#ifndef SIM_RHINO_POWERTRAIN_H
#define SIM_RHINO_POWERTRAIN_H

#include <memory>
#include <string>

namespace opendlv {
namespace sim {
namespace rhino {

/**
 * Simulated Rhino powertrain.
 */
class Powertrain {
  public:
    Powertrain();
    Powertrain(Powertrain const &) = delete;
    Powertrain &operator=(Powertrain const &) = delete;
    virtual ~Powertrain();
    double CalcEngineMaxTorque() const;
    double GetAcceleratorPedalPosition() const;
    double GetEngineSpeed() const;
    double GetEngineTorque() const;
    int32_t GetGear() const;
    void Update(double, double);
    void SetAcceleratorPedalPosition(double);

  private:
    double m_acceleratorPedalPosition;
    double m_engineSpeed;
    double m_engineTorque;
};

}
}
}

#endif
