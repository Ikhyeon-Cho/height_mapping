/*
 * HeightMapLoader.cpp
 *
 *  Created on: Mar 21, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "heightmap_loader/HeightMapLoader.h"

HeightMapLoader::HeightMapLoader() {
  ROS_INFO_STREAM("[1] Reading images from: " << image_path_ + map_name_);

  // Check the image path
  std::filesystem::path path(image_path_ + map_name_);
  if (!std::filesystem::exists(path) || !std::filesystem::is_directory(path)) {
    ROS_ERROR("Could not find the directory. Check the image path.");
    ros::shutdown();
    return;
  }
  ROS_INFO("Found the directory. Looking for .png files..");

  // Iterate through the directory, looking for .png files
  std::vector<std::string> image_paths;
  for (const auto &entry : std::filesystem::directory_iterator(path)) {
    if (entry.path().extension() == ".png") {
      ROS_INFO_STREAM("Found " << entry.path().filename().string());
      image_paths.push_back(entry.path().string());
    }
  }
  if (image_paths.empty()) {
    ROS_ERROR("Could not find any .png files in the directory. Check the image "
              "path.");
    ros::shutdown();
    return;
  }

  // // Read map layer images
  // std::vector<cv::Mat> images;
  // for (const auto& img_path : image_paths)
  // {
  //   cv::Mat image = cv::imread(img_path);
  //   if (image.empty())
  //   {
  //     ROS_ERROR("Could not read the image");
  //     continue;
  //   }
  //   images.push_back(image);
  // }

  std::cout << std::endl;
  ROS_INFO("[2] Found .png files. Reading image metadata..");

  std::map<std::string, YAML::Node> img_yamls;
  for (const auto &img_path : image_paths) {
    // Check the metadata file
    std::filesystem::path yaml_path = img_path;
    yaml_path.replace_extension(".yaml");
    if (!std::filesystem::exists(yaml_path)) {
      ROS_ERROR_STREAM("Could not find " << yaml_path.filename().string());
      continue;
    }

    // Read metadata
    YAML::Node img_yaml = YAML::LoadFile(yaml_path.string());
    if (img_yaml) {
      std::string layer_name = img_yaml["Layer"].as<std::string>();
      img_yamls[layer_name] = img_yaml;
      ROS_INFO_STREAM("Read " << layer_name << ".yaml");
    } else {
      ROS_ERROR("Could not read the metadata file.");
      continue;
    }
  }

  std::cout << std::endl;
  ROS_INFO("[3] Converting images to height map layers..");

  for (const auto &img_path : image_paths) {
    cv::Mat image = cv::imread(img_path);
    if (image.empty()) {
      ROS_ERROR("Could not read the image");
      continue;
    }

    std::filesystem::path yaml_path = img_path;
    yaml_path.replace_extension(".yaml");
    std::string layer_name =
        yaml_path.stem()
            .string(); // Assumes layer name is the stem of the file name

    if (img_yamls.find(layer_name) == img_yamls.end()) {
      ROS_ERROR_STREAM("Metadata for " << layer_name << " not found.");
      continue;
    }

    auto &img_yaml = img_yamls[layer_name];
    double min_val = img_yaml["Min"].as<double>();
    double max_val = img_yaml["Max"].as<double>();
    std::string layer = img_yaml["Layer"].as<std::string>();

    if (!HeightMapConverter::fromGrayImage(image, min_val, max_val, map_,
                                           layer)) {
      ROS_ERROR_STREAM("Could not convert the " << layer << " to height map");
      continue;
      ;
    }
    ROS_INFO_STREAM("Converted " << layer << " layer");
  }

  map_.setPosition(
      grid_map::Position(map_.getLength().x() / 2, map_.getLength().y() / 2));

  std::cout << std::endl;
  ROS_INFO("[4] Done. Start visualization..");
  visualization_timer_.start();
}

void HeightMapLoader::visualizeHeightMap(const ros::WallTimerEvent &event) {
  // Visualize height map
  visualization_msgs::Marker msg_region;
  HeightMapMsgs::toMapRegion(map_, msg_region);
  pub_map_region_.publish(msg_region);

  // Visualize height map as point cloud
  sensor_msgs::PointCloud2 msg_cloud;
  grid_map::GridMapRosConverter::toPointCloud(map_, map_.getHeightLayer(),
                                              msg_cloud);
  pub_heightcloud_.publish(msg_cloud);
}