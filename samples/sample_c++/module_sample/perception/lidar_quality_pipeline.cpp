#include "lidar_quality_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

namespace dji_lidar_quality {
namespace {

struct Key {
    std::int32_t x;
    std::int32_t y;
    std::int32_t z;
    bool operator==(const Key &other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct KeyHash {
    std::size_t operator()(const Key &key) const {
        std::size_t h = static_cast<std::size_t>(key.x) * 73856093u;
        h ^= static_cast<std::size_t>(key.y) * 19349663u;
        h ^= static_cast<std::size_t>(key.z) * 83492791u;
        return h;
    }
};

struct Accumulator {
    std::uint32_t count = 0;
    std::uint32_t lastFrame = 0;
    double sx = 0.0, sy = 0.0, sz = 0.0;
    double sxx = 0.0, syy = 0.0, szz = 0.0;
    double sxy = 0.0, sxz = 0.0, syz = 0.0;
    double intensity = 0.0;

    void Add(const Point &p, std::uint32_t frame) {
        ++count;
        lastFrame = frame;
        sx += p.x; sy += p.y; sz += p.z;
        sxx += p.x * p.x; syy += p.y * p.y; szz += p.z * p.z;
        sxy += p.x * p.y; sxz += p.x * p.z; syz += p.y * p.z;
        intensity += p.intensity;
    }

    void Merge(const Accumulator &o) {
        count += o.count;
        lastFrame = std::max(lastFrame, o.lastFrame);
        sx += o.sx; sy += o.sy; sz += o.sz;
        sxx += o.sxx; syy += o.syy; szz += o.szz;
        sxy += o.sxy; sxz += o.sxz; syz += o.syz;
        intensity += o.intensity;
    }
};

struct Node {
    Key cell;
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float ax = 0.0f, ay = 0.0f, az = 1.0f;
    float linearity = 0.0f;
    float intensity = 0.0f;
    std::uint32_t count = 0;
};

static Key Quantize(float x, float y, float z, float size) {
    return Key{static_cast<std::int32_t>(std::floor(x / size)),
               static_cast<std::int32_t>(std::floor(y / size)),
               static_cast<std::int32_t>(std::floor(z / size))};
}

static bool Finite(const Point &p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

static Node MakeNode(const Key &key, const Accumulator &a) {
    Node n;
    n.cell = key;
    n.count = a.count;
    if (a.count == 0) return n;
    const double inv = 1.0 / a.count;
    const double mx = a.sx * inv, my = a.sy * inv, mz = a.sz * inv;
    n.x = static_cast<float>(mx); n.y = static_cast<float>(my); n.z = static_cast<float>(mz);
    n.intensity = static_cast<float>(a.intensity * inv);

    const double c00 = std::max(0.0, a.sxx * inv - mx * mx);
    const double c11 = std::max(0.0, a.syy * inv - my * my);
    const double c22 = std::max(0.0, a.szz * inv - mz * mz);
    const double c01 = a.sxy * inv - mx * my;
    const double c02 = a.sxz * inv - mx * mz;
    const double c12 = a.syz * inv - my * mz;

    double vx = 0.577350269, vy = 0.577350269, vz = 0.577350269;
    for (int i = 0; i < 8; ++i) {
        const double nx = c00 * vx + c01 * vy + c02 * vz;
        const double ny = c01 * vx + c11 * vy + c12 * vz;
        const double nz = c02 * vx + c12 * vy + c22 * vz;
        const double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (norm < 1e-12) break;
        vx = nx / norm; vy = ny / norm; vz = nz / norm;
    }
    const double lambda1 = vx * (c00 * vx + c01 * vy + c02 * vz) +
                           vy * (c01 * vx + c11 * vy + c12 * vz) +
                           vz * (c02 * vx + c12 * vy + c22 * vz);
    const double trace = c00 + c11 + c22;
    n.ax = static_cast<float>(vx); n.ay = static_cast<float>(vy); n.az = static_cast<float>(vz);
    n.linearity = trace > 1e-12 ? static_cast<float>(lambda1 / trace) : 0.0f;
    return n;
}

struct DisjointSet {
    std::vector<std::size_t> parent;
    std::vector<std::size_t> size;
    explicit DisjointSet(std::size_t n) : parent(n), size(n, 1) {
        for (std::size_t i = 0; i < n; ++i) parent[i] = i;
    }
    std::size_t Find(std::size_t x) {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    }
    void Unite(std::size_t a, std::size_t b) {
        a = Find(a); b = Find(b);
        if (a == b) return;
        if (size[a] < size[b]) std::swap(a, b);
        parent[b] = a; size[a] += size[b];
    }
};

}  // namespace

struct Pipeline::Impl {
    explicit Impl(const Config &c) : config(c) {}
    Config config;
    std::unordered_map<Key, Accumulator, KeyHash> voxels;
    std::uint32_t lastDecayFrame = 0;
};

Pipeline::Pipeline(const Config &config) : impl_(new Impl(config)) {
    impl_->voxels.reserve(config.maxActiveVoxels);
}

Pipeline::~Pipeline() { delete impl_; }

void Pipeline::Reset() {
    impl_->voxels.clear();
    impl_->lastDecayFrame = 0;
}

Result Pipeline::Process(const Point *points, std::size_t pointCount,
                         std::uint64_t timestampNs, std::uint32_t frameNumber) {
    Result out;
    out.timestampNs = timestampNs;
    out.frameNumber = frameNumber;
    out.inputPoints = pointCount;
    if (!points || pointCount == 0) return out;

    const Config &cfg = impl_->config;
    if (cfg.statisticsDecayFrames > 0 &&
        frameNumber >= impl_->lastDecayFrame + cfg.statisticsDecayFrames) {
        for (typename std::unordered_map<Key, Accumulator, KeyHash>::iterator it = impl_->voxels.begin();
             it != impl_->voxels.end();) {
            Accumulator &a = it->second;
            a.count /= 2;
            a.sx *= 0.5; a.sy *= 0.5; a.sz *= 0.5;
            a.sxx *= 0.5; a.syy *= 0.5; a.szz *= 0.5;
            a.sxy *= 0.5; a.sxz *= 0.5; a.syz *= 0.5;
            a.intensity *= 0.5;
            if (a.count == 0) it = impl_->voxels.erase(it); else ++it;
        }
        impl_->lastDecayFrame = frameNumber;
    }
    const std::size_t stride = std::max<std::size_t>(1, (pointCount + cfg.maxInputPoints - 1) /
                                                        cfg.maxInputPoints);
    out.inputCapped = stride > 1;

    for (std::size_t i = 0; i < pointCount; i += stride) {
        const Point &p = points[i];
        if (!Finite(p) || p.label == 1 || p.label == 3) continue;
        const Key key = Quantize(p.x, p.y, p.z, cfg.voxelSizeM);
        typename std::unordered_map<Key, Accumulator, KeyHash>::iterator it = impl_->voxels.find(key);
        if (it == impl_->voxels.end()) {
            if (impl_->voxels.size() >= cfg.maxActiveVoxels) {
                out.voxelCapped = true;
                continue;
            }
            it = impl_->voxels.insert(std::make_pair(key, Accumulator())).first;
        }
        it->second.Add(p, frameNumber);
        ++out.sampledPoints;
    }

    for (typename std::unordered_map<Key, Accumulator, KeyHash>::iterator it = impl_->voxels.begin();
         it != impl_->voxels.end();) {
        const std::uint32_t age = frameNumber >= it->second.lastFrame ?
                                  frameNumber - it->second.lastFrame : 0;
        if (age > cfg.voxelTtlFrames) it = impl_->voxels.erase(it); else ++it;
    }

    out.activeVoxels = impl_->voxels.size();
    out.lowDensityRegions.reserve(cfg.maxLowDensityRegions);
    for (typename std::unordered_map<Key, Accumulator, KeyHash>::const_iterator it = impl_->voxels.begin();
         it != impl_->voxels.end() && out.lowDensityRegions.size() < cfg.maxLowDensityRegions; ++it) {
        if (it->second.count >= cfg.minVoxelPoints) continue;
        const double inv = it->second.count ? 1.0 / it->second.count : 0.0;
        Result::LowDensityRegion region;
        region.x = static_cast<float>(it->second.sx * inv);
        region.y = static_cast<float>(it->second.sy * inv);
        region.z = static_cast<float>(it->second.sz * inv);
        region.pointCount = it->second.count;
        region.density = static_cast<float>(it->second.count /
                                            (cfg.voxelSizeM * cfg.voxelSizeM * cfg.voxelSizeM));
        out.lowDensityRegions.push_back(region);
    }
    // Keep the lowest-density, spatially separated candidates first so the
    // overlay identifies meaningful regions instead of arbitrary hash entries.
    std::sort(out.lowDensityRegions.begin(), out.lowDensityRegions.end(),
              [](const Result::LowDensityRegion &a, const Result::LowDensityRegion &b) {
                  return a.pointCount < b.pointCount;
              });
    std::vector<Result::LowDensityRegion> separated;
    separated.reserve(out.lowDensityRegions.size());
    const float minCandidateDistance2 = cfg.supervoxelSizeM * cfg.supervoxelSizeM;
    for (std::size_t i = 0; i < out.lowDensityRegions.size(); ++i) {
        bool tooClose = false;
        for (std::size_t j = 0; j < separated.size(); ++j) {
            const float dx = out.lowDensityRegions[i].x - separated[j].x;
            const float dy = out.lowDensityRegions[i].y - separated[j].y;
            const float dz = out.lowDensityRegions[i].z - separated[j].z;
            if (dx * dx + dy * dy + dz * dz < minCandidateDistance2) { tooClose = true; break; }
        }
        if (!tooClose) separated.push_back(out.lowDensityRegions[i]);
    }
    out.lowDensityRegions.swap(separated);
    std::unordered_map<Key, Accumulator, KeyHash> super;
    super.reserve(std::min(cfg.maxSupervoxels, impl_->voxels.size()));
    for (typename std::unordered_map<Key, Accumulator, KeyHash>::const_iterator it = impl_->voxels.begin();
         it != impl_->voxels.end(); ++it) {
        if (it->second.count >= cfg.minVoxelPoints) ++out.denseVoxels;
        const double inv = it->second.count ? 1.0 / it->second.count : 0.0;
        const Key sk = Quantize(static_cast<float>(it->second.sx * inv),
                                static_cast<float>(it->second.sy * inv),
                                static_cast<float>(it->second.sz * inv), cfg.supervoxelSizeM);
        typename std::unordered_map<Key, Accumulator, KeyHash>::iterator sit = super.find(sk);
        if (sit == super.end()) {
            if (super.size() >= cfg.maxSupervoxels) { out.supervoxelCapped = true; continue; }
            sit = super.insert(std::make_pair(sk, Accumulator())).first;
        }
        sit->second.Merge(it->second);
    }
    out.denseVoxelRatio = out.activeVoxels ?
                          static_cast<float>(out.denseVoxels) / out.activeVoxels : 0.0f;

    std::vector<Node> nodes;
    nodes.reserve(super.size());
    std::unordered_map<Key, std::size_t, KeyHash> nodeByCell;
    nodeByCell.reserve(super.size());
    for (typename std::unordered_map<Key, Accumulator, KeyHash>::const_iterator it = super.begin();
         it != super.end(); ++it) {
        if (it->second.count < cfg.minSupervoxelPoints) continue;
        Node node = MakeNode(it->first, it->second);
        if (node.linearity < cfg.minLinearity) continue;
        nodeByCell[node.cell] = nodes.size();
        nodes.push_back(node);
    }
    out.supervoxels = super.size();
    out.woodCandidates = nodes.size();

    DisjointSet dsu(nodes.size());
    std::vector<std::uint16_t> degree(nodes.size(), 0);
    const float maxDist2 = cfg.maxLinkDistanceM * cfg.maxLinkDistanceM;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        const Node &a = nodes[i];
        for (int dz = -1; dz <= 1; ++dz) for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0 && dz == 0) continue;
                const Key nk{a.cell.x + dx, a.cell.y + dy, a.cell.z + dz};
                typename std::unordered_map<Key, std::size_t, KeyHash>::const_iterator jt = nodeByCell.find(nk);
                if (jt == nodeByCell.end() || jt->second <= i) continue;
                const std::size_t j = jt->second;
                const Node &b = nodes[j];
                const float px = a.x - b.x, py = a.y - b.y, pz = a.z - b.z;
                const float dist2 = px * px + py * py + pz * pz;
                const float align = std::fabs(a.ax * b.ax + a.ay * b.ay + a.az * b.az);
                if (dist2 > maxDist2 || align < cfg.minAxisAlignment) continue;
                if (out.edges >= cfg.maxEdges) { out.edgeCapped = true; continue; }
                ++out.edges; ++degree[i]; ++degree[j]; dsu.Unite(i, j);
            }
    }

    std::unordered_map<std::size_t, std::size_t> componentSizes;
    std::size_t largest = 0;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (degree[i] == 0) ++out.isolatedNodes;
        const std::size_t root = dsu.Find(i);
        const std::size_t size = ++componentSizes[root];
        largest = std::max(largest, size);
    }
    out.components = componentSizes.size();
    out.largestComponentRatio = nodes.empty() ? 0.0f : static_cast<float>(largest) / nodes.size();
    return out;
}

std::string Result::ToJson() const {
    std::ostringstream s;
    s << std::fixed << std::setprecision(4)
      << "{\"frame\":" << frameNumber
      << ",\"timestamp_ns\":" << timestampNs
      << ",\"input_points\":" << inputPoints
      << ",\"sampled_points\":" << sampledPoints
      << ",\"active_voxels\":" << activeVoxels
      << ",\"dense_voxels\":" << denseVoxels
      << ",\"dense_voxel_ratio\":" << denseVoxelRatio
      << ",\"supervoxels\":" << supervoxels
      << ",\"wood_candidates\":" << woodCandidates
      << ",\"edges\":" << edges
      << ",\"components\":" << components
      << ",\"isolated_nodes\":" << isolatedNodes
      << ",\"largest_component_ratio\":" << largestComponentRatio
      << ",\"low_density_regions\":[";
    const std::size_t jsonRegionLimit = std::min<std::size_t>(lowDensityRegions.size(), 16);
    for (std::size_t i = 0; i < jsonRegionLimit; ++i) {
        if (i != 0) s << ',';
        const LowDensityRegion &r = lowDensityRegions[i];
        s << "{\"x\":" << r.x << ",\"y\":" << r.y << ",\"z\":" << r.z
          << ",\"density\":" << r.density << ",\"points\":" << r.pointCount << "}";
    }
    s << "]"
      << ",\"capped\":{\"input\":" << (inputCapped ? "true" : "false")
      << ",\"voxel\":" << (voxelCapped ? "true" : "false")
      << ",\"supervoxel\":" << (supervoxelCapped ? "true" : "false")
      << ",\"edge\":" << (edgeCapped ? "true" : "false") << "}}";
    return s.str();
}

}  // namespace dji_lidar_quality
