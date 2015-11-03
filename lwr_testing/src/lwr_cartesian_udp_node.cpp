#include <ros/ros.h>
// PCL specific includes

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <lwr_controllers/PoseRPY.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <kdl/frames_io.hpp>

using namespace std;

/** MAIN NODE **/
int
main(int argc, char** argv) {

        std::cout << "LWR Cartesina UDP Node Started...\n";
        // Initialize ROS
        ros::init(argc, argv, "lwr_cartesian_udp_node");
        ros::NodeHandle nh;

}
