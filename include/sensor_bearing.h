#ifndef SENSORBREARING_H
#define SENSORBEARING_H

#include <sensor.h>

/* 
Assumption here is that we are solving for only one extra robot 
in the environment otherwise we would need to chnage our update menthod 
somehow incorporating the measurements from all the robots
*/

namespace arpro
{
class BearingSensor : public Sensor {
	public :
	BearingSensor ( Robot & _robot , double _x , double _y , double _theta ) :
		Sensor ( _robot , _x , _y , _theta ) // call the Sensor constructor
			{ // the RangeSensor constructor does nothing more }
			}
		void checkTwist(Twist &_t) {
			// chnage in twist to follow the other robot based on s_ computed
			_t.w = _t.w - (gain_* s_);
		}

	void update(const Pose &_p){
		// look for first other robot
		double bearingAngle;
		for ( auto other : envir_->robots_ )
			if ( other != robot_ )
			{
				// compute angle between sensor and detected robot
				Pose robotPose = other->pose();
				bearingAngle = atan2(robotPose.y - _p.y, robotPose.x - _p.x) - _p.theta;
				break ;
			}

	    if (abs(bearingAngle) > 2*M_PI )
	        bearingAngle = fmod(bearingAngle, 2*M_PI);
	    if(bearingAngle > M_PI)
	        bearingAngle =- 2*M_PI + bearingAngle;
	    else if (bearingAngle < -M_PI)
	        bearingAngle = 2*M_PI + bearingAngle;
		s_= bearingAngle;
	}

protected:
	double gain_ = 0.5;
};

}

#endif