#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "lidar_quality_pipeline.hpp"

namespace dji_lidar_quality {

struct RgbCalibration {
    // Row-major rotation and translation: p_rgb = R * p_lidar + t.
    float rotation[9];
    float translation[3];
    float fx;
    float fy;
    float cx;
    float cy;
    std::uint32_t imageWidth;
    std::uint32_t imageHeight;
    std::uint64_t maxTimestampSkewNs = 100000000;  // 100 ms
};

struct RgbImage {
    std::uint8_t *data = nullptr;
    std::size_t sizeBytes = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t strideBytes = 0;
    std::uint8_t channels = 3;  // RGB packed (3) or RGBA (4).
    std::uint64_t timestampNs = 0;
};

struct OverlayResult {
    std::size_t projectedRegions = 0;
    std::size_t rejectedBehindCamera = 0;
    std::size_t rejectedOutOfFrame = 0;
    bool timestampAligned = false;
    bool calibrationValid = false;
};

class RgbOverlay {
public:
    explicit RgbOverlay(const RgbCalibration &calibration);

    OverlayResult DrawLowDensity(const std::vector<Result::LowDensityRegion> &regions,
                                  std::uint64_t lidarTimestampNs,
                                  RgbImage *image,
                                  std::uint8_t alpha = 150,
                                  std::uint32_t radiusPixels = 8) const;

private:
    RgbCalibration calibration_;
};

}  // namespace dji_lidar_quality
