/*
 * dynamics_vehicle.h
 *
 *  Created on: Jan 22, 2018
 *      Author: yushu
 */

#ifndef DYNAMICS_VEHICLE_H_
#define DYNAMICS_VEHICLE_H_

//other header files

#include <memory>
#include <string>


namespace opendlv {
namespace sim {
namespace rhino {

class dynamics_vehicle{
public:
	dynamics_vehicle();
	virtual ~dynamics_vehicle();
 

private:
	double T_emax; 

};

}
}
}

#endif /* DYNAMICS_VEHICLE_H_ */
