/*
 * sensor_processor_node.cpp
 *
 *  Created on: Apr 29, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "height_mapping/SensorProcessor.h"

int main(int argc, char** argv)
{
  std::string node_name{ "sensor_processor" };

  ros::init(argc, argv, node_name);
  ros::NodeHandle nh{ node_name };

  // Read from the parameter server
  std::string point_type_{ nh.param<std::string>("pointType", "color") };
  // Set parameter

  if (point_type_ == "color")
  {
    std::cout << "\033[1;32m[SensorProcessor]: Waiting for RGB pointcloud Processing...\033[0m" << std::endl;

    nh.setParam("processedCloudTopic", "color/points");

    SensorProcessor<pcl::PointXYZRGB> node;
    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
  }
  else if (point_type_ == "laser")
  {
    std::cout << "\033[1;32m[SensorProcessor]: Waiting for Laser pointcloud Processing...\033[0m" << std::endl;

    nh.setParam("processedCloudTopic", "laser/points");

    SensorProcessor<pcl::PointXYZI> node;
    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
  }
  else
  {
    std::cout << "\033[33m[SensorProcessor]: Invalid Point Type!! Supported point types: color, laser\033[0m"
              << std::endl;
    return -1;
  }

  return 0;
}