#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace dji_lidar_quality {

struct Point {
    float x;
    float y;
    float z;
    std::uint8_t intensity;
    std::uint8_t label;
};

struct Config {
    float voxelSizeM = 0.20f;
    float supervoxelSizeM = 0.60f;
    std::size_t maxInputPoints = 200000;
    std::size_t maxActiveVoxels = 50000;
    std::size_t maxSupervoxels = 5000;
    std::size_t maxEdges = 30000;
    std::uint32_t minVoxelPoints = 3;
    std::uint32_t minSupervoxelPoints = 12;
    float minLinearity = 0.55f;
    float maxLinkDistanceM = 1.20f;
    float minAxisAlignment = 0.55f;
    std::uint32_t voxelTtlFrames = 25;
    std::uint32_t statisticsDecayFrames = 5;
    std::size_t maxLowDensityRegions = 128;
};

struct Result {
    struct LowDensityRegion {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float density = 0.0f;
        std::uint32_t pointCount = 0;
    };
    std::uint64_t timestampNs = 0;
    std::uint32_t frameNumber = 0;
    std::size_t inputPoints = 0;
    std::size_t sampledPoints = 0;
    std::size_t activeVoxels = 0;
    std::size_t denseVoxels = 0;
    std::size_t supervoxels = 0;
    std::size_t woodCandidates = 0;
    std::size_t edges = 0;
    std::size_t components = 0;
    std::size_t isolatedNodes = 0;
    float denseVoxelRatio = 0.0f;
    float largestComponentRatio = 0.0f;
    bool inputCapped = false;
    bool voxelCapped = false;
    bool supervoxelCapped = false;
    bool edgeCapped = false;
    std::vector<LowDensityRegion> lowDensityRegions;

    std::string ToJson() const;
};

class Pipeline {
public:
    explicit Pipeline(const Config &config = Config());
    ~Pipeline();

    Pipeline(const Pipeline &) = delete;
    Pipeline &operator=(const Pipeline &) = delete;

    Result Process(const Point *points,
                   std::size_t pointCount,
                   std::uint64_t timestampNs,
                   std::uint32_t frameNumber);
    void Reset();

private:
    struct Impl;
    Impl *impl_;
};

}  // namespace dji_lidar_quality
