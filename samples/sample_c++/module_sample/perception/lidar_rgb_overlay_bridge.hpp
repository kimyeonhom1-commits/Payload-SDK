#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "lidar_rgb_overlay.hpp"

namespace dji_lidar_quality {

void PublishLowDensityRegions(const std::vector<Result::LowDensityRegion> &regions,
                              std::uint64_t hostTimestampNs);
void SetRgbCalibration(const RgbCalibration &calibration);
bool LoadRgbCalibrationFile(const char *path);
bool HasRgbCalibration();

// Mutates an RGB/RGBA image buffer only when a valid calibration and a recent
// low-density result are available. Returns true when an overlay was drawn.
OverlayResult ApplyLatestOverlay(std::uint8_t *data,
                                 std::size_t sizeBytes,
                                 std::uint32_t width,
                                 std::uint32_t height,
                                 std::uint8_t channels,
                                 std::uint64_t hostTimestampNs);

}  // namespace dji_lidar_quality
