#include <ros/ros.h>
// PCL specific includes

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <lwr_controllers/PoseRPY.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"
#include "pcl_ros/impl/transforms.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <kdl/frames_io.hpp>

using namespace std;

pcl::visualization::PCLVisualizer* viewer;
typedef pcl::PointXYZRGBA PointType;
pcl::PointCloud<PointType>::Ptr cloud_full(new  pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_full_filtered(new  pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud(new  pcl::PointCloud<PointType>);
pcl::VoxelGrid<PointType> sor;
pcl::PassThrough<PointType> pass;
std::string save_folder;
int save_counter =0;
Eigen::Matrix4f T;

void
pose_cb (const lwr_controllers::PoseRPY& pose)
{
        KDL::Frame frame;
        frame.M = KDL::Rotation::RPY(
                pose.orientation.roll,
                pose.orientation.pitch,
                pose.orientation.yaw
                );

        frame.p[0] = pose.position.x;
        frame.p[1] = pose.position.y;
        frame.p[2] = pose.position.z;

        for(int i = 0; i < 3; i++) {
                for(int j =0; j < 3; j++) {
                        T(i,j) = frame.M(i,j);
                }
        }

        T(0,3) = pose.position.x;
        T(1,3) = pose.position.y;
        T(2,3) = pose.position.z;



}
void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

        // Create a container for the data.
        sensor_msgs::PointCloud2 output;


        pcl::PointCloud<PointType>::Ptr cloud_trans(new  pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr cloud_trans_filtered(new  pcl::PointCloud<PointType>);
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*input, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc, *cloud);

        viewer->removeAllPointClouds();
        viewer->addPointCloud(cloud, "scene");
}

unsigned int text_id = 0;
bool capture = false;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
        if (event.getKeySym () == "v" && event.keyDown ())
        {
                if(cloud->points.size()>1000) {
                        //(*cloud_full) += (*cloud_trans);

                        std::ofstream myfile;
                        std::stringstream ss;
                        ss << save_folder << "/"<<save_counter<<".txt";

                        myfile.open (ss.str().c_str());
                        myfile << T;
                        myfile.close();


                        ss.str("");
                        ss << save_folder << "/" << save_counter<<".pcd";
                        pcl::io::savePCDFileASCII (ss.str().c_str(), *cloud);

                        std::cout << "Saved snapshot: "<<save_counter<<std::endl;
                        save_counter++;
                }
        }
}



ros::Publisher pose_publisher;
ros::Publisher joints_publisher;
ros::Subscriber sub;
ros::Subscriber sub_pose;

void updateCommands() {

        lwr_controllers::PoseRPY pose;

        sensor_msgs::JointState joint_msg;
        joint_msg.name.resize(7);
        joint_msg.name[0] = "lwr_0_joint";
        joint_msg.name[1] = "lwr_1_joint";
        joint_msg.name[2] = "lwr_2_joint";
        joint_msg.name[3] = "lwr_3_joint";
        joint_msg.name[4] = "lwr_4_joint";
        joint_msg.name[5] = "lwr_5_joint";
        joint_msg.name[6] = "lwr_6_joint";
        joint_msg.position.resize(7);


        for(int i = 0; i < 7; i++) {
                joint_msg.position[0]=0;
        }

        float x=0.5f,y=0.5f,z=0.5f,roll=0.0f,pitch=0.0f,yaw=0.0f;
        char c;

        int t = 0;
        float f = 0.00002f;

        while(true) {
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
                if(c=='c') {
                        pose.id = 0;
                        pose.position.x = x;
                        pose.position.y = y;
                        pose.position.z = z;

                        pose.orientation.roll=roll;
                        pose.orientation.pitch=pitch;
                        pose.orientation.yaw=yaw;
                        std::cout << pose <<std::endl;
                        pose_publisher.publish(pose);
                }else if(c=='j') {
                        joint_msg.position[0] = x;
                        joint_msg.position[1] = y;
                        joint_msg.position[2] = z;
                        joint_msg.position[3] = roll;
                        joint_msg.position[4] = pitch;
                        joint_msg.position[5] = yaw;
                        joint_msg.position[6] = 0;
                        joints_publisher.publish(joint_msg);
                }
        }
}

/** MAIN NODE **/
int
main (int argc, char** argv)
{
        // Initialize ROS
        ros::init (argc, argv, "comau_tf_follower");
        ros::NodeHandle nh;
        viewer = new pcl::visualization::PCLVisualizer("viewer");
        viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);

        std::stringstream ss;
        ss << "/home/daniele/temp/"<<ros::Time::now();
        save_folder = ss.str();

        T <<
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1;

        boost::filesystem::create_directory(save_folder);



        pose_publisher = nh.advertise<lwr_controllers::PoseRPY>("/lwr/full_control_simple/command", 1);
        joints_publisher = nh.advertise<sensor_msgs::JointState>("/lwr/full_control_simple/command_joints", 1);
        sub = nh.subscribe ("/xtion/xtion/depth/points", 1, cloud_cb);
        sub_pose = nh.subscribe ("/lwr/full_control_simple/current_pose", 1, pose_cb);


        // Spin

        boost::thread updateTFsThread(updateCommands);

        // Spin
        while(nh.ok() && !viewer->wasStopped()) {







                //joints_publisher.publish(traj);
                viewer->spinOnce();
                ros::spinOnce();
        }

        updateTFsThread.join();
}
