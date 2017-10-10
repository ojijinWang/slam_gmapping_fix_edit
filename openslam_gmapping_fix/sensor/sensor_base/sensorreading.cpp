#include <gmapping_fix/sensor/sensor_base/sensorreading.h>

namespace GMapping_Fix{

SensorReading::SensorReading(const Sensor* s, double t){
	m_sensor=s;
	m_time=t;
}


SensorReading::~SensorReading(){
}

};

