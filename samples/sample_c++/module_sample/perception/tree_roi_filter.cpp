#include "tree_roi_filter.hpp"

#include <algorithm>
#include <cmath>

namespace dji_lidar_quality {
namespace {

static bool ValidCalibration(const RgbCalibration &c) {
    return c.imageWidth > 0 && c.imageHeight > 0 && c.fx > 0.0f && c.fy > 0.0f &&
           std::isfinite(c.fx) && std::isfinite(c.fy) &&
           std::isfinite(c.cx) && std::isfinite(c.cy);
}

static bool ValidMask(const SegmentationMask &m) {
    return m.data && m.width > 0 && m.height > 0 && m.strideBytes >= m.width &&
           m.sizeBytes >= static_cast<std::size_t>(m.strideBytes) * m.height;
}

}  // namespace

TreeRoiFilter::TreeRoiFilter(const RgbCalibration &calibration, const TreeRoiConfig &config)
    : calibration_(calibration), config_(config) {}

TreeRoiResult TreeRoiFilter::Filter(const std::vector<Point> &input,
                                    std::uint64_t lidarTimestampNs,
                                    const SegmentationMask &mask,
                                    std::vector<Point> *treeRoi) const {
    TreeRoiResult result;
    result.inputPoints = input.size();
    if (!treeRoi) return result;
    treeRoi->clear();
    treeRoi->reserve(std::min(input.size(), config_.maxOutputPoints));
    result.calibrationValid = ValidCalibration(calibration_);
    if (!result.calibrationValid || !ValidMask(mask)) return result;
    const std::uint64_t skew = lidarTimestampNs > mask.timestampNs ?
        lidarTimestampNs - mask.timestampNs : mask.timestampNs - lidarTimestampNs;
    result.timestampAligned = skew <= config_.maxTimestampSkewNs;
    if (!result.timestampAligned) return result;

    for (std::size_t i = 0; i < input.size(); ++i) {
        const Point &p = input[i];
        const float x = calibration_.rotation[0] * p.x + calibration_.rotation[1] * p.y +
                        calibration_.rotation[2] * p.z + calibration_.translation[0];
        const float y = calibration_.rotation[3] * p.x + calibration_.rotation[4] * p.y +
                        calibration_.rotation[5] * p.z + calibration_.translation[1];
        const float z = calibration_.rotation[6] * p.x + calibration_.rotation[7] * p.y +
                        calibration_.rotation[8] * p.z + calibration_.translation[2];
        if (!(z > 0.01f) || !std::isfinite(z)) { ++result.projectedOutOfFrame; continue; }
        const int px = static_cast<int>(std::lround(calibration_.fx * x / z + calibration_.cx));
        const int py = static_cast<int>(std::lround(calibration_.fy * y / z + calibration_.cy));
        if (px < 0 || py < 0 || px >= static_cast<int>(mask.width) || py >= static_cast<int>(mask.height)) {
            ++result.projectedOutOfFrame; continue;
        }
        const std::uint8_t label = mask.data[static_cast<std::size_t>(py) * mask.strideBytes + px];
        if (label == RGB_MASK_TREE) {
            ++result.treePoints;
            if (treeRoi->size() < config_.maxOutputPoints) treeRoi->push_back(p);
            else result.outputCapped = true;
        } else if (label == RGB_MASK_BUILDING) {
            ++result.buildingPoints;
        } else if (label == RGB_MASK_GROUND) {
            ++result.groundPoints;
        } else {
            ++result.unknownPoints;
            if (!config_.requireTreeLabel && !config_.discardUnknown) {
                if (treeRoi->size() < config_.maxOutputPoints) treeRoi->push_back(p);
                else result.outputCapped = true;
            }
        }
    }
    result.outputPoints = treeRoi->size();
    return result;
}

}  // namespace dji_lidar_quality
