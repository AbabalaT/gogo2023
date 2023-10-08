#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <iostream>

int main(int argc, char **argv){
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh("~");

    rs2::pipeline pipe;
    rs2::config cfg;
 
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
 
    rs2::pipeline_profile profile = pipe.start(cfg); 
    rs2::align align_to_color(RS2_STREAM_COLOR);
    

    while(ros::ok()){
        rs2::frameset frameset = pipe.wait_for_frames();
        auto aligned_frameset = align_to_color.process(frameset); // 实际进行流对齐
 
        // 基于对齐的混合流获取深度流和彩色流,进而获取流对齐后的深度内参
        rs2::video_frame color_stream = aligned_frameset.get_color_frame();
        rs2::depth_frame aligned_depth_stream = aligned_frameset.get_depth_frame();
        rs2::video_stream_profile depth_stream_profile = aligned_depth_stream.get_profile().as<rs2::video_stream_profile>();
        const auto depth_intrinsics = depth_stream_profile.get_intrinsics(); //获取对齐后的深度内参
        cv::Mat color_image(cv::Size(640, 480), CV_8UC3, (void*)color_stream.get_data(), cv::Mat::AUTO_STEP);

		float pixe_center[2], point_in_color_coordinates[3];
		pixe_center[0] = 320;
		pixe_center[1] = 240;
 
        // 像素坐标系转换到相机坐标系
        float pixed_center_depth_value = aligned_depth_stream.get_distance(pixe_center[0],pixe_center[1]);
 
        rs2_deproject_pixel_to_point(point_in_color_coordinates, &depth_intrinsics, pixe_center, pixed_center_depth_value);
 
        std::cout
            << "像素中心在在彩色相机坐标系下的X坐标"<< point_in_color_coordinates[0]
            << "Y坐标系" << point_in_color_coordinates[1]
            << "Z坐标" << point_in_color_coordinates[2]
            << std::endl;
        cv::imshow("Image", color_image);
        cv::waitKey(1);
    }
    return 0;
}