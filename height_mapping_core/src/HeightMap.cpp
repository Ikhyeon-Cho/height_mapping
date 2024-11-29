/*
 * HeightMap.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/map/HeightMap.h"

namespace grid_map {
HeightMap::HeightMap() {
  // Add core layers
  add(CoreLayers::ELEVATION);
  add(CoreLayers::ELEVATION_MIN);
  add(CoreLayers::ELEVATION_MAX);
  add(CoreLayers::VARIANCE);

  setFrameId("map");
  setBasicLayers({CoreLayers::ELEVATION, CoreLayers::ELEVATION_MIN,
                  CoreLayers::ELEVATION_MAX});
}

void HeightMap::addLayer(const std::string &layer, float default_val) {
  if (!exists(layer))
    add(layer, default_val);
}

void HeightMap::removeLayer(const std::string &layer) {
  if (exists(layer))
    erase(layer);
}

void HeightMap::addBasicLayer(const std::string &layer) {
  auto basic_layers = getBasicLayers();
  basic_layers.insert(basic_layers.end(), {layer});
  setBasicLayers(basic_layers);
}

bool HeightMap::hasHeightValues() const {
  const auto &mat = getHeightMatrix();
  auto allNaN =
      mat.array().unaryExpr([](float elem) { return std::isnan(elem); }).all();
  return !allNaN;
}

float HeightMap::getMaxHeight() const {
  return HeightMapMath::getMaxVal(*this, CoreLayers::ELEVATION);
}

float HeightMap::getMinHeight() const {
  return HeightMapMath::getMinVal(*this, CoreLayers::ELEVATION);
}
} // namespace grid_map

float HeightMapMath::getMinVal(const grid_map::HeightMap &map,
                               const std::string &layer) {
  const auto &data = map[layer];

  auto fillNaNForFindingMinVal =
      data.array().isNaN().select(std::numeric_limits<double>::max(), data);
  return fillNaNForFindingMinVal.minCoeff();
}

float HeightMapMath::getMaxVal(const grid_map::HeightMap &map,
                               const std::string &layer) {
  const auto &data = map[layer];

  // https://www.geeksforgeeks.org/difference-between-stdnumeric_limitst-min-max-and-lowest-in-cpp/
  auto fillNaNForFindingMaxVal =
      data.array().isNaN().select(std::numeric_limits<double>::lowest(), data);
  return fillNaNForFindingMaxVal.maxCoeff();
}