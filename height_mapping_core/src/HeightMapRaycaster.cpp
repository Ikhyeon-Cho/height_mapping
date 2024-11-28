/*
 * HeightMapRaycaster.cpp
 *
 *  Created on: Nov 28, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/height_correctors/HeightMapRaycaster.h"

namespace height_mapping {

bool HeightMapRaycaster::isStaticAt(const grid_map::HeightMap &map,
                                    const grid_map::Index &index) {
  auto &minHeight = map.getMinHeightMatrix()(index(0), index(1));
  auto &maxHeight = map.getMaxHeightMatrix()(index(0), index(1));
  return maxHeight - minHeight < heightDiffThreshold_;
}
} // namespace height_mapping