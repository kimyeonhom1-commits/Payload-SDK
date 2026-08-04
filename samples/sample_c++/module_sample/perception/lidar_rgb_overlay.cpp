#include "lidar_rgb_overlay.hpp"

#include <algorithm>
#include <cmath>

namespace dji_lidar_quality {
namespace {

static bool ValidCalibration(const RgbCalibration &c) {
    return c.imageWidth > 0 && c.imageHeight > 0 &&
           c.fx > 0.0f && c.fy > 0.0f &&
           std::isfinite(c.fx) && std::isfinite(c.fy) &&
           std::isfinite(c.cx) && std::isfinite(c.cy);
}

static std::uint8_t BlendRed(std::uint8_t old, std::uint8_t alpha) {
    const unsigned inv = 255u - alpha;
    return static_cast<std::uint8_t>((255u * alpha + old * inv) / 255u);
}

}  // namespace

RgbOverlay::RgbOverlay(const RgbCalibration &calibration) : calibration_(calibration) {}

OverlayResult RgbOverlay::DrawLowDensity(
    const std::vector<Result::LowDensityRegion> &regions,
    std::uint64_t lidarTimestampNs,
    RgbImage *image,
    std::uint8_t alpha,
    std::uint32_t radiusPixels) const {
    OverlayResult result;
    result.calibrationValid = ValidCalibration(calibration_);
    if (!result.calibrationValid || !image || !image->data || image->width == 0 || image->height == 0) {
        return result;
    }
    if (image->width != calibration_.imageWidth || image->height != calibration_.imageHeight ||
        (image->channels != 3 && image->channels != 4) ||
        image->strideBytes < image->width * image->channels ||
        image->sizeBytes < static_cast<std::size_t>(image->strideBytes) * image->height) {
        return result;
    }
    const std::uint64_t skew = lidarTimestampNs > image->timestampNs ?
        lidarTimestampNs - image->timestampNs : image->timestampNs - lidarTimestampNs;
    result.timestampAligned = skew <= calibration_.maxTimestampSkewNs;
    if (!result.timestampAligned) return result;

    const std::size_t maxRegions = std::min<std::size_t>(regions.size(), 128);
    for (std::size_t i = 0; i < maxRegions; ++i) {
        const Result::LowDensityRegion &region = regions[i];
        const float px3 = calibration_.rotation[0] * region.x + calibration_.rotation[1] * region.y +
                          calibration_.rotation[2] * region.z + calibration_.translation[0];
        const float py3 = calibration_.rotation[3] * region.x + calibration_.rotation[4] * region.y +
                          calibration_.rotation[5] * region.z + calibration_.translation[1];
        const float pz3 = calibration_.rotation[6] * region.x + calibration_.rotation[7] * region.y +
                          calibration_.rotation[8] * region.z + calibration_.translation[2];
        if (!(pz3 > 0.01f) || !std::isfinite(pz3)) { ++result.rejectedBehindCamera; continue; }
        const int px = static_cast<int>(std::lround(calibration_.fx * px3 / pz3 + calibration_.cx));
        const int py = static_cast<int>(std::lround(calibration_.fy * py3 / pz3 + calibration_.cy));
        const int r = static_cast<int>(std::min<std::uint32_t>(radiusPixels, 32));
        if (px < -r || py < -r || px >= static_cast<int>(image->width) + r ||
            py >= static_cast<int>(image->height) + r) {
            ++result.rejectedOutOfFrame;
            continue;
        }
        const int minY = std::max(0, py - r), maxY = std::min(static_cast<int>(image->height) - 1, py + r);
        const int minX = std::max(0, px - r), maxX = std::min(static_cast<int>(image->width) - 1, px + r);
        for (int y = minY; y <= maxY; ++y) {
            for (int x = minX; x <= maxX; ++x) {
                const int dx = x - px, dy = y - py;
                if (dx * dx + dy * dy > r * r) continue;
                std::uint8_t *pixel = image->data + static_cast<std::size_t>(y) * image->strideBytes +
                                      static_cast<std::size_t>(x) * image->channels;
                // RGB/RGBA input/output. Red is blended; alpha is left unchanged.
                pixel[0] = BlendRed(pixel[0], alpha);
                pixel[1] = static_cast<std::uint8_t>(pixel[1] * (255u - alpha) / 255u);
                pixel[2] = static_cast<std::uint8_t>(pixel[2] * (255u - alpha) / 255u);
            }
        }
        ++result.projectedRegions;
    }
    return result;
}

}  // namespace dji_lidar_quality
