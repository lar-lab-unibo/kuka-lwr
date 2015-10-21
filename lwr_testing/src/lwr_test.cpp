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
pcl::PointCloud<PointType>::Ptr cloud_full(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_full_filtered(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
pcl::VoxelGrid<PointType> sor;
pcl::PassThrough<PointType> pass;
std::string save_folder;
int save_counter = 0;
Eigen::Matrix4f T;
Eigen::Matrix4f Robot_Base;
Eigen::Matrix4f EE;
Eigen::Matrix4f Target;

void rotationMatrixT(char axis, float angle, Eigen::Matrix4f& out) {
    if (axis == 'x') {
        out <<
                1, 0, 0, 0,
                0, cos(angle), -sin(angle), 0,
                0, sin(angle), cos(angle), 0,
                0, 0, 0, 1;
    }
    if (axis == 'y') {
        out <<
                cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;
    }
    if (axis == 'z') {
        out <<
                cos(angle), -sin(angle), 0, 0,
                sin(angle), cos(angle), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
    }

}

void rotationMatrixT(float roll, float pitch, float yaw, Eigen::Matrix4f& out) {
    KDL::Rotation rot = KDL::Rotation::RPY(roll, pitch, yaw);
    out << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out(i, j) = rot(i, j);
        }
    }
}

void transformMatrixT(float x, float y, float z, float roll, float pitch, float yaw, Eigen::Matrix4f& out) {
    KDL::Rotation rot = KDL::Rotation::RPY(roll, pitch, yaw);
    out << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            out(i, j) = rot(i, j);
        }
    }
    out(0, 3) = x;
    out(1, 3) = y;
    out(2, 3) = z;
}

Eigen::Matrix4f
invertTransformationMatrix(Eigen::Matrix4f & t) {
    Eigen::Matrix3f R = t.block<3, 3>(0, 0);
    Eigen::Vector3f translation = t.block<3, 1>(0, 3);
    Eigen::MatrixXf last_block = t.block<1, 4>(3, 0);

    Eigen::Matrix3f RT = R.transpose();
    Eigen::MatrixXf Temp(3, 4);
    translation = -RT*translation;
    Temp << RT, translation;
    Eigen::Matrix4f t2;
    t2 << Temp, last_block;

    return t2;
}

void
pose_cb(const lwr_controllers::PoseRPY& pose) {

    KDL::Frame frame;

    frame.M = KDL::Rotation::RPY(
            pose.orientation.roll,
            pose.orientation.pitch,
            pose.orientation.yaw
            );

    frame.p[0] = pose.position.x;
    frame.p[1] = pose.position.y;
    frame.p[2] = pose.position.z;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            T(i, j) = frame.M(i, j);
        }
    }

    T(0, 3) = pose.position.x;
    T(1, 3) = pose.position.y;
    T(2, 3) = pose.position.z;

    T = Robot_Base*T;
    T = T* EE;




}

void
cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {

    // Create a container for the data.
    sensor_msgs::PointCloud2 output;


    pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_trans_filtered(new pcl::PointCloud<PointType>);
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*input, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, *cloud);

    viewer->removeAllPointClouds();
    viewer->addPointCloud(cloud, "scene");
}

unsigned int text_id = 0;
bool capture = false;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
        void* viewer_void) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
    if (event.getKeySym() == "v" && event.keyDown()) {
        if (cloud->points.size() > 1000) {
            //(*cloud_full) += (*cloud_trans);

            std::ofstream myfile;
            std::stringstream ss;
            ss << save_folder << "/" << save_counter << ".txt";

            myfile.open(ss.str().c_str());
            myfile << T;
            myfile.close();


            ss.str("");
            ss << save_folder << "/" << save_counter << ".pcd";
            pcl::io::savePCDFileASCII(ss.str().c_str(), *cloud);

            std::cout << "Saved snapshot: " << save_counter << std::endl;
            save_counter++;
        }
    }
}



ros::Publisher pose_publisher;
ros::Publisher joints_publisher;
ros::Subscriber sub;
ros::Subscriber sub_pose;

lwr_controllers::PoseRPY pose;
sensor_msgs::JointState joint_msg;
char mode = 'j';

void updateCommands() {






    float x = 0.5f, y = 0.5f, z = 0.5f, roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    char c;

    int t = 0;
    float f = 0.00002f;

    while (true) {
        cout << "Command: ";
        cin >> c;
        mode = c;
        if (c == 'q') break;




        //traj.points[0].positions[1] = traj.points[0].positions[1]+0.01;
        if (c == 'c') {
            float distance, azimuth, zenith;
            cin >> distance;
            cin >> azimuth;
            cin >> zenith;

            Eigen::Matrix4f approach;
            Eigen::Matrix4f azi;
            Eigen::Matrix4f zeni;
            Eigen::Matrix4f NewTarget;
            transformMatrixT(0, 0, 0, 0, 0, azimuth, azi);
            transformMatrixT(0, 0, 0, 0, zenith, 0, zeni);
            transformMatrixT(0, 0, distance, 0, M_PI, 0, approach);
            NewTarget = Target;
            NewTarget = NewTarget*azi;
            NewTarget = NewTarget*zeni;
            NewTarget = NewTarget*approach;
            NewTarget = invertTransformationMatrix(Robot_Base) * NewTarget;


            KDL::Rotation r;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    r(i, j) = NewTarget(i, j);
                }
            }
            r.GetRPY(
                    pose.orientation.roll,
                    pose.orientation.pitch,
                    pose.orientation.yaw
                    );



            pose.id = 0;
            pose.position.x = NewTarget(0, 3);
            pose.position.y = NewTarget(1, 3);
            pose.position.z = NewTarget(2, 3);

            std::cout << pose << std::endl;

        } else if (c == 'j') {
            float* j = new float[7];
            //            for (int i = 0; i < 7; i++) {
            cin >> joint_msg.position[0];
            //            }


            //            joint_msg.position[0] = 0;
            joint_msg.position[1] = 1.57;
            joint_msg.position[2] = 0;
            joint_msg.position[3] = 0.9;
            joint_msg.position[4] = 0;
            joint_msg.position[5] = -1.8;
            joint_msg.position[6] = 0;

            //            joints_publisher.publish(joint_msg);
        }
    }
}

/** MAIN NODE **/
int
main(int argc, char** argv) {

    std::cout << "Testing node started...\n";
    // Initialize ROS
    ros::init(argc, argv, "comau_tf_follower");
    ros::NodeHandle nh;
    viewer = new pcl::visualization::PCLVisualizer("viewer");
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &viewer);

    std::stringstream ss;
    ss << "/home/daniele/temp/" << ros::Time::now();
    save_folder = ss.str();

    T <<
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;


    transformMatrixT(0, 0, 2.0f, 0, M_PI, 0, Robot_Base);
    rotationMatrixT(0, 0, -M_PI / 2, EE);
    transformMatrixT(0, 0, 1.0f, 0, 0, 0, Target);


    boost::filesystem::create_directory(save_folder);



    pose_publisher = nh.advertise<lwr_controllers::PoseRPY>("/lwr/full_control_simple/command", 1);
    joints_publisher = nh.advertise<sensor_msgs::JointState>("/lwr/full_control_simple/command_joints", 1);
    sub = nh.subscribe("/xtion/xtion/depth/points", 1, cloud_cb);
    sub_pose = nh.subscribe("/lwr/full_control_simple/current_pose", 1, pose_cb);


    // Spin

    boost::thread updateTFsThread(updateCommands);

    joint_msg.name.resize(7);
    joint_msg.name[0] = "lwr_0_joint";
    joint_msg.name[1] = "lwr_1_joint";
    joint_msg.name[2] = "lwr_2_joint";
    joint_msg.name[3] = "lwr_3_joint";
    joint_msg.name[4] = "lwr_4_joint";
    joint_msg.name[5] = "lwr_5_joint";
    joint_msg.name[6] = "lwr_6_joint";
    joint_msg.position.resize(7);

    joint_msg.position[0] = 0;
    joint_msg.position[1] = 1.57;
    joint_msg.position[2] = 0;
    joint_msg.position[3] = 0.9;
    joint_msg.position[4] = 0;
    joint_msg.position[5] = -1.8;
    joint_msg.position[6] = 0;


    mode='j';

    // Spin
    while (nh.ok() && !viewer->wasStopped()) {







        //joints_publisher.publish(traj);
        if (mode == 'c') {
            pose_publisher.publish(pose);
        } else if (mode == 'j') {
            joints_publisher.publish(joint_msg);
        }
        viewer->spinOnce();
        ros::spinOnce();
    }

    updateTFsThread.join();
}
