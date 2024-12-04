/*
 * KITTIMapWriter.h
 *
 *  Created on: Dec 02, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <height_mapping_core/height_mapping_core.h>
#include <string>
#include <vector>

class KITTIMapWriter {
public:
  KITTIMapWriter() = default;

  void setDataPath(const std::string &path);
  bool writeMap(const grid_map::GridMap &map);

private:
  std::string getMapFilename(unsigned int count, bool isInvalid = false) const;
  void writeLayerNames(const std::vector<std::string> &layerNames) const;
  std::pair<std::vector<float>, std::vector<uint8_t>>
  gridMapToTensor(const grid_map::GridMap &map) const;

  std::string dataPath_;
  unsigned int mapCount_{0};
  bool isInitialized_{false};
};
