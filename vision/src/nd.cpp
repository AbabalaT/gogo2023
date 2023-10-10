#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

tf2_ros::Buffer tfBuffer;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh("~");

    rs2::pipeline pipe;
    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    rs2::pipeline_profile profile = pipe.start(cfg);
    rs2::align align_to_color(RS2_STREAM_COLOR);
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::TransformBroadcaster br;
    // ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("image_topic", 1);

    while (ros::ok())
    {
        rs2::frameset frameset = pipe.wait_for_frames();
        auto aligned_frameset = align_to_color.process(frameset); // 实际进行流对齐

        // 基于对齐的混合流获取深度流和彩色流,进而获取流对齐后的深度内参
        rs2::video_frame color_stream = aligned_frameset.get_color_frame();
        rs2::depth_frame aligned_depth_stream = aligned_frameset.get_depth_frame();
        rs2::video_stream_profile depth_stream_profile = aligned_depth_stream.get_profile().as<rs2::video_stream_profile>();
        const auto depth_intrinsics = depth_stream_profile.get_intrinsics(); // 获取对齐后的深度内参
        cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void *)color_stream.get_data(), cv::Mat::AUTO_STEP);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_image).toImageMsg();
        image_pub.publish(msg);
        int xmin = detected_objects[i].rect.x;
        int ymin = detected_objects[i].rect.y;
        int width = detected_objects[i].rect.width;
        int height = detected_objects[i].rect.height;
        Rect rect1(xmin, ymin, width, height); // 左上坐标（x,y）和矩形的长(x)宽(y)
        // std::cout << detected_objects[i].class_id << ' ' << detected_objects[i].prob << std::endl;
        cv::rectangle(osrc, rect1, Scalar(0, 0, 255), 1, LINE_8, 0);

        float pixe_center[2], point_in_color_coordinates[3];
        pixe_center[0] = xmin + 0.5 * width;
        pixe_center[1] = (ymin + 0.5 * height) * 0.75;
        Rect rect2(pixe_center[0], pixe_center[1], 3, 3); // 左上坐标（x,y）和矩形的长(x)宽(y)
        cv::rectangle(color_image, rect2, Scalar(0, 0, 255), 1, LINE_8, 0);
        float pixed_center_depth_value = aligned_depth_stream.get_distance(pixe_center[0], pixe_center[1]);
        rs2_deproject_pixel_to_point(point_in_color_coordinates, &depth_intrinsics, pixe_center, pixed_center_depth_value);
        geometry_msgs::TransformStamped trans;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        trans.child_frame_id = "target";
        trans.header.stamp = ros::Time::now();
        trans.header.frame_id = "d435";
        trans.transform.rotation.x = q.x();
        trans.transform.rotation.y = q.y();
        trans.transform.rotation.z = q.z();
        trans.transform.rotation.w = q.w();
        trans.transform.translation.x = point_in_color_coordinates[2];
        trans.transform.translation.y = -point_in_color_coordinates[0];
        trans.transform.translation.z = -point_in_color_coordinates[1];
        br.sendTransform(trans);
        geometry_msgs::TransformStamped base2map;
        try
        {
            base2map = tfBuffer.lookupTransform("map", "target", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Target Get TF ERROR!");
        }
        std::cout
            << "ID:" << detected_objects[i].class_id
            << " X：" << base2map.transform.translation.x
            << " Y：" << base2map.transform.translation.y
            << std::endl;
        cv::imshow("Image", color_image);
        cv::waitKey(1);
    }
    return 0;
}