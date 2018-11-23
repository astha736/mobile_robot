

#include <iostream>
#include <math.h>
#include <robot.h>
#include <sensor.h>
// #include <sensor_range.h>

using namespace arpro;
using namespace std;

Environment* Sensor::envir_ = NULL;

/* 

Using pointer of the object created in memory "this" 
as it helps to avoid mistakes of accidentally using variables 
that have similar names in the code. 

Though originally specified functions are left untouched 

*/

Robot::Robot(string _name, double _x, double _y, double _theta)
{    
    pose_.x = _x;
    pose_.y = _y;
    pose_.theta = _theta;

    name_ = _name;

    // init position history
    x_history_.push_back(_x);
    y_history_.push_back(_y);

    // default sampling time: 1/100 s
    dt_ = .01;
}

void Robot::initWheels(double b, double r, double wmax)
{
    //defining an initWheel method
    baseDistance_ = b;
    radius_ = r;
    velocityMax_ = wmax;
    wheelsInit_ = true;

}

void Robot::moveXYT(double _vx, double _vy, double _omega)
{
    // update position
    pose_.x += _vx*dt_;
    pose_.y += _vy*dt_;
    pose_.theta += _omega*dt_;

    // store position history
    x_history_.push_back(pose_.x);
    y_history_.push_back(pose_.y);
}



void Robot::rotateWheels(double _left, double _right)
{
    // defining an initWheel method

    if(!this->wheelsInit_) return;

    double a = max(
        fabs(_left)/this->velocityMax_,
        fabs(_right)/this->velocityMax_);

    if (a < 1) a = 1;

    _left   = _left/a;
    _right  = _right/a;

    double velocity_vec = this->radius_*(_left + _right)/2;
    double omega_vec= this->radius_*(_left - _right)/(2*this->baseDistance_);

    moveXYT(
        velocity_vec*cos(this->pose_.theta),
        velocity_vec*sin(this->pose_.theta),
        omega_vec);

}


// move robot with linear and angular velocities
void Robot::moveVW(double _v, double _omega)
{
    double w_left  = (_v + this->baseDistance_*_omega)/this->radius_;
    double w_right = (_v - this->baseDistance_*_omega)/this->radius_;
    rotateWheels(w_left,w_right);
}

// try to go to a given x-y position
void Robot::goTo(const Pose &_p)
{
    // error in robot frame
    Pose error = _p.transformInverse(pose_);
    // try to do a straight line with sensor constraints
    moveWithSensor(Twist(error.x, error.y, 0));
}


void Robot::moveWithSensor(Twist _twist)
{
    // Answer 3.1 Q3
    for ( auto & sensor : sensors_){
        sensor->updateFromRobotPose(this->pose_);
        sensor->checkRobotTwist(_twist);
    }

    moveVW(_twist.vx,20* _twist.vy +_twist.w);

}


void Robot::printPosition()
{
    cout << "Current position: " << pose_.x << ", " << pose_.y << endl;
}

