#ifndef SENSORRANGE_H
#define SENSORRANGE_H

#include <sensor.h>
#include <string>
#include <envir.h>
#include <robot.h>
#include <float.h>

// class RangeSensor;
namespace arpro
{
class RangeSensor : public Sensor {
	public :
	RangeSensor ( Robot & _robot , double _x , double _y , double _theta ) :
		Sensor ( _robot , _x , _y , _theta ) // call the Sensor constructor
			{ // the RangeSensor constructor does nothing more }
			}

		void checkTwist(Twist &_t) {
			// keeping the twist velocity in check to avoid collision
			if(_t.vx > gain_*(s_ - safe_wall_distance_))
				_t.vx = gain_*(s_ - safe_wall_distance_);

		}

	void update(const Pose &_p){
		Pose p1 , p2 ;
		double true_distance = FLT_MAX;
		for ( int i =0; i < envir_-> walls . size ();++ i )
		{
			p1 = envir_->walls [ i ];
			p2 = envir_->walls [( i +1)% envir_->walls.size ()];
			// do whatever you want to do with points p1 and p2
			double distance_num = p1.x*(p2.y - _p.y) - p2.x*(p1.y - _p.y) + _p.x*(p1.y - p2.y);
			double distance_den = sin(_p.theta)*(p1.x - p2.x) - cos(_p.theta)*(p1.y - p2.y);

			double current_wall_distance;
			if(distance_den == 0) continue;
			else current_wall_distance = distance_num/distance_den;

			if(current_wall_distance < 0) continue;
			else if (current_wall_distance < true_distance) true_distance = current_wall_distance;
		}
		if(true_distance < FLT_MAX) s_ = true_distance;
	}
// 
protected:
	double safe_wall_distance_ = 0.1; 
	double gain_ = 0.1;

};
}

#endif