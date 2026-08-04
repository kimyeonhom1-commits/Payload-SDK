#include "lidar_rgb_overlay_bridge.hpp"

#include <algorithm>
#include <fstream>
#include <mutex>

namespace dji_lidar_quality {
namespace {

std::mutex g_mutex;
std::vector<Result::LowDensityRegion> g_regions;
RgbCalibration g_calibration = {};
bool g_calibrationSet = false;
std::uint64_t g_timestampNs = 0;

}  // namespace

void PublishLowDensityRegions(const std::vector<Result::LowDensityRegion> &regions,
                              std::uint64_t hostTimestampNs) {
    std::lock_guard<std::mutex> lock(g_mutex);
    const std::size_t count = std::min<std::size_t>(regions.size(), 128);
    g_regions.assign(regions.begin(), regions.begin() + count);
    g_timestampNs = hostTimestampNs;
}

void SetRgbCalibration(const RgbCalibration &calibration) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_calibration = calibration;
    g_calibrationSet = true;
}

bool LoadRgbCalibrationFile(const char *path) {
    if (!path) return false;
    std::ifstream in(path);
    if (!in) return false;
    RgbCalibration c = {};
    if (!(in >> c.imageWidth >> c.imageHeight >> c.fx >> c.fy >> c.cx >> c.cy)) return false;
    for (int i = 0; i < 9; ++i) if (!(in >> c.rotation[i])) return false;
    for (int i = 0; i < 3; ++i) if (!(in >> c.translation[i])) return false;
    if (in >> c.maxTimestampSkewNs) {
        // Optional final value is nanoseconds. Keep the default if omitted.
    }
    SetRgbCalibration(c);
    return true;
}

bool HasRgbCalibration() {
    std::lock_guard<std::mutex> lock(g_mutex);
    return g_calibrationSet;
}

OverlayResult ApplyLatestOverlay(std::uint8_t *data, std::size_t sizeBytes,
                                 std::uint32_t width, std::uint32_t height,
                                 std::uint8_t channels, std::uint64_t hostTimestampNs) {
    std::vector<Result::LowDensityRegion> regions;
    RgbCalibration calibration = {};
    std::uint64_t lidarTimestamp = 0;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        regions = g_regions;
        calibration = g_calibration;
        lidarTimestamp = g_timestampNs;
    }
    RgbOverlay overlay(calibration);
    RgbImage image;
    image.data = data;
    image.sizeBytes = sizeBytes;
    image.width = width;
    image.height = height;
    image.strideBytes = width * channels;
    image.channels = channels;
    image.timestampNs = hostTimestampNs;
    return overlay.DrawLowDensity(regions, lidarTimestamp, &image, 150, 10);
}

}  // namespace dji_lidar_quality
