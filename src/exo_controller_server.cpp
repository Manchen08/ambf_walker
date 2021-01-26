 #include "ros/ros.h"
#include "Eigen/Core"
#include "rbdl_server/RBDLServer.h"
#include "boost/shared_ptr.hpp"
#include "controller_modules/ControllerManager.h"
#include "controller_modules/PDController.h"
#include "ambf_walker/DynController.h"
#include "controller_modules/JointControl.h"
#include "controller_modules/ControllerBase.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "sensor_msgs/JointState.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "Controller_Server");
    ros::NodeHandle n;
   

    Eigen::MatrixXd Kp = Eigen::MatrixXd::Ones(7,7);
    Eigen::MatrixXd Kd = Eigen::MatrixXd::Zero(7,7);
    
    Kp(0,0) = 100.0;
    Kp(1,1) = 100.0;
    Kp(2,2) = 100.0;
    Kp(3,3) = 100.0;
    Kp(4,4) = 100.0;
    Kp(5,5) = 100.0;
    Kp(6,6) = 100.0;

    Kd(0,0) = 4.0;
    Kd(1,1) = 4.0;
    Kd(2,2) = 4.0;
    Kd(3,3) = 4.0;
    Kd(4,4) = 4.0;
    Kd(5,5) = 0.40;
    Kd(6,6) = 0.40;

    ControllerManager manager = ControllerManager(&n);
    PDController pd(Kp,Kd);
    boost::shared_ptr<ControllerBase> my_controller(new  DynController("exo", &n, &pd) );

    manager.addController("Dyn", my_controller);

    ros::spin();

    return 0;
}