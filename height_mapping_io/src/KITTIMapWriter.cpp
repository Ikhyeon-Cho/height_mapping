#include "height_mapping_io/KITTIMapWriter.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
void KITTIMapWriter::setDataPath(const std::string &path) {
  dataPath_ = path;
  std::filesystem::create_directories(dataPath_);
  isInitialized_ = true;
}

bool KITTIMapWriter::writeMap(const grid_map::GridMap &map) {
  if (!isInitialized_) {
    throw std::runtime_error("[KITTIMapWriter] Data path is not set");
  }

  auto start = std::chrono::high_resolution_clock::now();

  // Write layer names if this is the first map
  if (mapCount_ == 0) {
    writeLayerNames(map.getLayers());
  }

  // Convert grid map to tensor and invalid mask
  auto [tensor, invalidMask] = gridMapToTensor(map);

  // Write tensor to binary file
  auto dataFilename = getMapFilename(mapCount_);
  std::ofstream dataFile(dataFilename, std::ios::out | std::ios::binary);
  if (!dataFile.is_open()) {
    throw std::runtime_error("Failed to open data file: " + dataFilename);
  }
  dataFile.write(reinterpret_cast<const char *>(tensor.data()),
                 tensor.size() * sizeof(float));
  dataFile.close();

  // Write invalid mask to binary file
  auto invalidFilename = getMapFilename(mapCount_, true);
  std::ofstream invalidFile(invalidFilename, std::ios::out | std::ios::binary);
  if (!invalidFile.is_open()) {
    throw std::runtime_error("Failed to open invalid file: " + invalidFilename);
  }
  invalidFile.write(reinterpret_cast<const char *>(invalidMask.data()),
                    invalidMask.size() * sizeof(uint8_t));
  invalidFile.close();

  auto end = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  std::cout << "[KITTIMapWriter] Write time: " << duration.count() / 1000.0
            << " ms (map size: " << map.getSize()(0) << "x" << map.getSize()(1)
            << ", layers: " << map.getLayers().size() << ")" << std::endl;

  mapCount_++;
  return true;
}

void KITTIMapWriter::writeLayerNames(
    const std::vector<std::string> &layerNames) const {
  auto layerFile = dataPath_ + "/layers.txt";
  std::ofstream file(layerFile);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open layer file: " + layerFile);
  }

  for (const auto &layer : layerNames) {
    file << layer << std::endl;
  }
  file.close();
}

std::pair<std::vector<float>, std::vector<uint8_t>>
KITTIMapWriter::gridMapToTensor(const grid_map::GridMap &map) const {
  const auto &size = map.getSize();
  const auto &layers = map.getLayers();
  const size_t numCells = size(0) * size(1);
  const size_t numLayers = layers.size();

  std::vector<float> tensor(numCells * numLayers);
  std::vector<uint8_t> invalidMask(numCells, 0); // 0: valid, 1: invalid

  size_t cellIdx = 0;
  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    // First check the basic layer (elevation) for invalid mask
    invalidMask[cellIdx] =
        map.isValid(*it, grid_map::HeightMap::CoreLayers::ELEVATION) ? 0 : 1;

    // Store data for each layer
    for (size_t l = 0; l < numLayers; ++l) {
      const auto &layer = layers[l];
      size_t tensorIdx = cellIdx * numLayers + l;

      if (map.isValid(*it, layer)) {
        tensor[tensorIdx] = map.at(layer, *it);
      } else {
        tensor[tensorIdx] = std::numeric_limits<float>::quiet_NaN();
      }
    }

    cellIdx++;
  }

  return {tensor, invalidMask};
}

std::string KITTIMapWriter::getMapFilename(unsigned int count,
                                           bool isInvalid) const {
  std::stringstream ss;
  ss << std::setw(6) << std::setfill('0') << count;
  return dataPath_ + "/" + ss.str() + (isInvalid ? ".invalid" : ".bin");
}
