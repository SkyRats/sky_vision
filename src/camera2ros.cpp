#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Header.h"
#include "string.h"
#include "ros/console.h"
#include <cv_bridge/cv_bridge.h>
#include <iostream>

using namespace cv;

int main(int argc, char **argv){

  ros::init(argc, argv, "camera");
  ros::NodeHandle nodeHandle("~"); //the "~" makes the handler launch the node as private

  int index = 0; //opencv default camera index
  
  //this param has to be private, as we may launch multiple nodes like this
  ros::param::get("~camera/index", index);

  String topicName = "sky_vision/camera_" + std::to_string(index) + "/image_raw";

  sensor_msgs::Image frameROS; //ros image format
  cv_bridge::CvImage frameBridge; //bridge class
  Mat frameCV; //opencv image format
  std_msgs::Header header; //ros images need a header
  uint32_t counter = 0; //counter used to populate the ros image header

  VideoCapture cap(index);

  while(!cap.isOpened() && ros::ok){
    ROS_ERROR("CAMERA_%d: Wasn't able to open the camera. Trying again...\n", index);
    ros::Duration(1).sleep();
    ros::spinOnce();
  }

  ROS_INFO("CAMERA_%d: Succesfully opened the camera. \n", index);


  ros::Publisher cameraPub = nodeHandle.advertise<sensor_msgs::Image>(topicName, 5);

  while(ros::ok()){

    header.seq = counter;
    header.stamp = ros::Time::now();
    cap.read(frameCV);
    frameBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, frameCV);
    cameraPub.publish(frameBridge);

    counter++;
    ros::spinOnce();
  }

  return 0;
}