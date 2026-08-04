#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "lidar_quality_pipeline.hpp"
#include "lidar_rgb_overlay.hpp"

namespace dji_lidar_quality {

enum RgbMaskLabel : std::uint8_t {
    RGB_MASK_UNKNOWN = 0,
    RGB_MASK_TREE = 1,
    RGB_MASK_BUILDING = 2,
    RGB_MASK_GROUND = 3
};

struct SegmentationMask {
    const std::uint8_t *data = nullptr;
    std::size_t sizeBytes = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t strideBytes = 0;
    std::uint64_t timestampNs = 0;
};

struct TreeRoiConfig {
    bool requireTreeLabel = false;
    bool discardUnknown = false;
    std::uint64_t maxTimestampSkewNs = 100000000;
    std::size_t maxOutputPoints = 200000;
};

struct TreeRoiResult {
    std::size_t inputPoints = 0;
    std::size_t treePoints = 0;
    std::size_t buildingPoints = 0;
    std::size_t groundPoints = 0;
    std::size_t unknownPoints = 0;
    std::size_t projectedOutOfFrame = 0;
    std::size_t outputPoints = 0;
    bool calibrationValid = false;
    bool timestampAligned = false;
    bool outputCapped = false;
};

class TreeRoiFilter {
public:
    TreeRoiFilter(const RgbCalibration &calibration,
                  const TreeRoiConfig &config = TreeRoiConfig());

    TreeRoiResult Filter(const std::vector<Point> &input,
                         std::uint64_t lidarTimestampNs,
                         const SegmentationMask &mask,
                         std::vector<Point> *treeRoi) const;

private:
    RgbCalibration calibration_;
    TreeRoiConfig config_;
};

}  // namespace dji_lidar_quality
