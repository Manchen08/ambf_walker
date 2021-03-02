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
   
    Eigen::MatrixXd exo_Kp = Eigen::MatrixXd::Ones(7,7);
    Eigen::MatrixXd exo_Kd = Eigen::MatrixXd::Ones(7,7);
    
    
    exo_Kp(0,0) = 1000.0;
    exo_Kp(1,1) = 1000.0;
    exo_Kp(2,2) = 1000.0;
    exo_Kp(3,3) = 1000.0;
    exo_Kp(4,4) = 1000.0;
    exo_Kp(5,5) = 1000.0;
    exo_Kp(6,6) = 1000.0;

    exo_Kd(0,0) = 4;
    exo_Kd(1,1) = 4;
    exo_Kd(2,2) = 4;
    exo_Kd(3,3) = 4;
    exo_Kd(4,4) = 4;
    exo_Kd(5,5) = 4;
    exo_Kd(6,6) = 4;




    
    ControllerManager manager = ControllerManager(&n);
    PDController exo(exo_Kp, exo_Kd);
    boost::shared_ptr<ControllerBase> Dyn_controller(new  DynController("exo", &n, &exo) );



    manager.addController("Dyn", Dyn_controller);

    ros::spin();

    return 0;
}