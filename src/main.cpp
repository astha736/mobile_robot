// #include <iostream>
#include <math.h>
#include <cmath>

#include <robot.h>
#include <envir.h>
#include <sensor.h>
#include <sensor_range.h> 
#include <sensor_bearing.h>



using namespace std;
using namespace arpro;



int main()
{
    // default environment with moving target
    Environment envir;
    // sensors gets measurements from this environment
    Sensor::setEnvironment(envir);

    // init robot at (0,0,0)
    Robot robot("R2D2", 0,0,0);
    envir.addRobot(robot);
    robot.initWheels(0.3, 0.07,10);
    RangeSensor rangeSensor(robot,0.1,0,0);

    Robot robotSmall("sR2D2",0,0,0);
    robotSmall.initWheels(0.3,0.05,10);
    envir.addRobot(robotSmall);
    BearingSensor bearingSensor(robotSmall,0.1,0,0);


    for(unsigned int i=0;i<10000;++i)
    {
        cout << "---------------------" << endl;

        // update target position
        envir.updateTarget();        

        // try to follow target
        robot.goTo(envir.target);
        robotSmall.moveWithSensor(Twist(0.4,0,0));

    }

    // plot trajectory
    envir.plot();
    
}
