#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <istream>
#include <gmapping_fix/sensor/sensor_base/sensor.h>

namespace GMapping_Fix {

class Configuration{
	public:
		virtual ~Configuration();
		virtual SensorMap computeSensorMap() const=0;
};

};
#endif

