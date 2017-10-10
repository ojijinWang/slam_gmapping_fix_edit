#include <gmapping_fix/sensor/sensor_base/sensor.h>

namespace GMapping_Fix{

Sensor::Sensor(const std::string& name){
	m_name=name;
}

Sensor::~Sensor(){
}

};// end namespace
