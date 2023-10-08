//
// Created by bismarck on 12/11/22.
//
#define USE_CUSTOM_LASER2SCAN

#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

ros::Publisher pub;
bool publish_tf = true;

tf2_ros::Buffer tfBuffer;

void project2plane_callback(const ros::TimerEvent&){    //将3D位置投影到2D地图上用于导航
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped base2map;
    try {
        base2map = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Project2Scan Get TF ERROR!");
        return;
    }

    tf2::Quaternion b2m{base2map.transform.rotation.x, base2map.transform.rotation.y,
                        base2map.transform.rotation.z, base2map.transform.rotation.w};

    double roll = 0, pitch = 0, yaw = 0;
    tf2::Matrix3x3(b2m).getRPY(roll, pitch, yaw);

    if (publish_tf) {
        geometry_msgs::TransformStamped trans = base2map;
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        trans.child_frame_id = "plane_base_link";
        trans.header.stamp = ros::Time::now();
        trans.transform.rotation.x = q.x();
        trans.transform.rotation.y = q.y();
        trans.transform.rotation.z = q.z();
        trans.transform.rotation.w = q.w();
        trans.transform.translation.z = 0;
        br.sendTransform(trans);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "laser_projection");
    ros::NodeHandle pnh("~");
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Timer timer1 = pnh.createTimer(ros::Duration(0.02), project2plane_callback);
    ros::spin();
    return 0;
}
