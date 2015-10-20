#include <ros/ros.h>
// PCL specific includes

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <lwr_controllers/PoseRPY.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

using namespace std;


/** MAIN NODE **/
int
main (int argc, char** argv)
{
        // Initialize ROS
        ros::init (argc, argv, "comau_tf_follower");
        ros::NodeHandle nh;


        ros::Publisher pose_publisher = nh.advertise<lwr_controllers::PoseRPY>("/lwr/full_control_simple/command", 1);
        ros::Publisher joints_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/lwr/joint_trajectory_controller/command", 1);

        lwr_controllers::PoseRPY pose;
        trajectory_msgs::JointTrajectory traj;
        trajectory_msgs::JointTrajectoryPoint point;
        traj.points.resize(1);
        traj.joint_names.resize(7);
        traj.joint_names[0] = "lwr_0_joint";
        traj.joint_names[1] = "lwr_1_joint";
        traj.joint_names[2] = "lwr_2_joint";
        traj.joint_names[3] = "lwr_3_joint";
        traj.joint_names[4] = "lwr_4_joint";
        traj.joint_names[5] = "lwr_5_joint";
        traj.joint_names[6] = "lwr_6_joint";



        traj.points[0] = point;
        traj.points[0].time_from_start = ros::Duration(0.1);
        traj.points[0].positions.resize(7);
        for(int i = 0; i < 7; i++) {
                traj.points[0].positions[i] = 0.0f;
        }



        std::cout << traj <<std::endl;
        float x,y,z,roll,pitch,yaw;
        char c;

        int t = 0;
        float f = 0.00002f;
        // Spin
        while( ros::ok()) {
                cout << "Command: ";
                cin >> c;

                if(c=='q') break;

                cout << "Position: ";
                cin >> x;
                cin >> y;
                cin >> z;
                cin >> roll;
                cin >> pitch;
                cin >> yaw;


                //traj.points[0].positions[1] = traj.points[0].positions[1]+0.01;
                pose.id = 0;
                pose.position.x = x;
                pose.position.y = y;
                pose.position.z = z;

                pose.orientation.roll=roll;
                pose.orientation.roll=pitch;
                pose.orientation.yaw=yaw;






                std::cout << pose <<std::endl;
                pose_publisher.publish(pose);
                //joints_publisher.publish(traj);
                ros::spinOnce();

        }

}
