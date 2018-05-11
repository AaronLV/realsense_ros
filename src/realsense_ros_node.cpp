/* Author: Yetao,Lyu */
#include <iostream>

// include OpenCV header file
#include <opencv2/opencv.hpp>

// include the librealsense C++ header file
#include <librealsense2/rs.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>

// include Eigen header file
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS image handling
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;
using namespace Eigen;

#define  PI 3.1415926
#define  color_width 640
#define  color_height 480
#define  depth_width 640
#define  depth_height 480
#define  all_fps 30

//for shifting the RGB image in order to better align with depth image
#define  shift_x 2
#define  shift_y 8

ros::Publisher img_pub;
image_transport::Publisher g_pub_c;
ros::Publisher info_pub;
ros::Publisher cloud_pub;

Mat translateImg(Mat &img, int offsetx, int offsety){
    Mat trans_mat = (Mat_<double>(2,3) << 1, 0, offsetx, 0, 1, offsety);
    warpAffine(img,img,trans_mat,img.size());
    return trans_mat;
}

void img_capture(rs2::pipeline pipe, rs2::align align, rs2_stream align_to){
  ros::Time t = ros::Time::now();

  //capture a frame
  rs2::frameset frames = pipe.wait_for_frames();

  //Get processed aligned frame
  auto proccessed = align.proccess(frames);

  // Trying to get both color and aligned depth frames
  rs2::video_frame color_frame = proccessed.first(align_to);
  rs2::depth_frame aligned_depth_frame = proccessed.get_depth_frame();

  // Creating OpenCV Matrix from a color image
  Mat color(Size(color_width, color_height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
  translateImg(color,shift_x, shift_y);

  sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color).toImageMsg();
  img_msg->header.stamp = t;
  img_msg->header.frame_id = "world";
  img_pub.publish(img_msg);
  g_pub_c.publish(img_msg);
  sensor_msgs::CameraInfo camera_info;
  camera_info.width = color_width;
  camera_info.height = color_height;
  camera_info.header.stamp = t;
  camera_info.header.frame_id = "world";

  camera_info.K.at(0) = 616.4887349929622;
  camera_info.K.at(2) = 330.09844701390523;
  camera_info.K.at(4) = 620.3902938047753;
  camera_info.K.at(5) = 242.55281878477678;
  camera_info.K.at(8) = 1;

  camera_info.P.at(0) = camera_info.K.at(0);
  camera_info.P.at(1) = 0;
  camera_info.P.at(2) = camera_info.K.at(2);
  camera_info.P.at(3) = 0;
  camera_info.P.at(4) = 0;
  camera_info.P.at(5) = camera_info.K.at(4);
  camera_info.P.at(6) = camera_info.K.at(5);
  camera_info.P.at(7) = 0;
  camera_info.P.at(8) = 0;
  camera_info.P.at(9) = 0;
  camera_info.P.at(10) = 1;
  camera_info.P.at(11) = 0;

  camera_info.R.at(0) = 1.0;
  camera_info.R.at(1) = 0.0;
  camera_info.R.at(2) = 0.0;
  camera_info.R.at(3) = 0.0;
  camera_info.R.at(4) = 1.0;
  camera_info.R.at(5) = 0.0;
  camera_info.R.at(6) = 0.0;
  camera_info.R.at(7) = 0.0;
  camera_info.R.at(8) = 1.0;

  camera_info.D.push_back(0.1526821970349751);
  camera_info.D.push_back(-0.2646397257024497);
  camera_info.D.push_back(-0.004882202239657907);
  camera_info.D.push_back(0.0016206790069335302);
  camera_info.D.push_back(0.0);
  info_pub.publish(camera_info);

  //string filename_rgb("/home/aaronlv/catkin_ws/src/realsense_ros/rgbd_image.png");
  //imwrite(filename_train, color);

  //// Creating OpenCV Matrix from a depth image
  //Mat depth(Size(depth_width, depth_height), CV_16UC1, (void*)aligned_depth_frame.get_data(), Mat::AUTO_STEP);
  //string filename_depth("/home/aaronlv/catkin_ws/src/realsense_ros/depth_map.png");
  //imwrite(filename_depth, depth*10);

  // Declare pointcloud object, for calculating pointclouds and texture mappings
  rs2::pointcloud pc;
  // We want the points object to be persistent so we can display the last cloud when a frame drops
  rs2::points points;
  // Generate the pointcloud and texture mappings
  points = pc.calculate(aligned_depth_frame);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  const rs2::vertex *vertices = points.get_vertices();              // get vertices

  sensor_msgs::PointCloud2 cloud_msg;
  cloud_msg.header.stamp = t;
  cloud_msg.header.frame_id = "world";
  cloud_msg.width = depth_width;
  cloud_msg.height = depth_height;

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2Fields(4,
                                "x", 1, sensor_msgs::PointField::FLOAT32,
                                "y", 1, sensor_msgs::PointField::FLOAT32,
                                "z", 1, sensor_msgs::PointField::FLOAT32,
                                "rgb", 1, sensor_msgs::PointField::FLOAT32);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  sensor_msgs::PointCloud2Iterator<float>iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float>iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float>iter_z(cloud_msg, "z");

  sensor_msgs::PointCloud2Iterator<uint8_t>iter_r(cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t>iter_g(cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t>iter_b(cloud_msg, "b");

  // Fill the PointCloud2 fields
  for (int i = 0; i < depth_width * depth_height; i++) {
      float x = (vertices + i)->x;
      float y = (vertices + i)->y;
      float z = (vertices + i)->z;

      *iter_x = x;
      *iter_y = y;
      *iter_z = z;

      *iter_r = static_cast<uint8_t>(color.at<Vec3b>(i)[2]);
      *iter_g = static_cast<uint8_t>(color.at<Vec3b>(i)[1]);
      *iter_b = static_cast<uint8_t>(color.at<Vec3b>(i)[0]);

      ++iter_x; ++iter_y; ++iter_z;
      ++iter_r; ++iter_g; ++iter_b;
      // pcl::PointXYZRGB point(color.at<Vec3b>(i)[2],color.at<Vec3b>(i)[1],color.at<Vec3b>(i)[0]);
      // point.x = x;
      // point.y = y;
      // point.z = z;
      // cloud->push_back(point);
  }
  cloud_pub.publish(cloud_msg);
  // pcl::PLYWriter Writer;
  // Writer.write("/home/aaronlv/catkin_ws/src/realsense_ros/train.ply", *cloud);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "realsense_ros");
  ros::NodeHandle n;
  #pragma region realsense initialization
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, color_width, color_height, RS2_FORMAT_BGR8, all_fps);
    cfg.enable_stream(RS2_STREAM_DEPTH, depth_width, depth_height, RS2_FORMAT_Z16, all_fps);
    cfg.enable_stream(RS2_STREAM_INFRARED, depth_width, depth_height, RS2_FORMAT_Y8, all_fps);

    //Instruct pipeline to start streaming with the requested configuration
    pipe.start(cfg);

    //align depth to the color stream
    rs2_stream align_to = RS2_STREAM_COLOR;

    // Create a rs2::align object.
    // rs2::align allows us to perform alignment of depth frames to others frames
    //The "align_to" is the stream type to which we plan to align depth frames.
    rs2::align align(align_to);
  #pragma region realsense initialization

  ROS_INFO("setupPublishers...");
  image_transport::ImageTransport image_transport(n);
  g_pub_c = image_transport.advertise("camera/color/image_raw_c", 1);

  img_pub = n.advertise<sensor_msgs::Image>("camera/color/image_raw", 1);
  info_pub = n.advertise<sensor_msgs::CameraInfo>("camera/color/camera_info", 1);
  cloud_pub = n.advertise<sensor_msgs::PointCloud2>("camera/points", 1);

  ros::Rate loop_rate(10);
  while (n.ok()) {
    img_capture(pipe,align,align_to);
    ros::spinOnce();
    loop_rate.sleep();
  }

}
