#pragma once

#include "core/Math.h"

#include <cstdint>
#include <vector>

namespace occlusa {

// Static 3D kd-tree over a point set for nearest-neighbour queries (ICP correspondences).
class KdTree {
public:
    KdTree() = default;
    explicit KdTree(std::vector<glm::vec3> points) { build(std::move(points)); }

    void build(std::vector<glm::vec3> points);
    bool empty() const { return points_.empty(); }
    std::size_t size() const { return points_.size(); }
    const std::vector<glm::vec3>& points() const { return points_; }

    // Index into points() of the nearest point within maxDistance, or -1.
    std::int64_t nearest(const glm::vec3& query, float maxDistance, float* outDistanceSq = nullptr) const;

private:
    struct Node {
        std::uint32_t begin, end; // range into order_
        std::int32_t left = -1, right = -1;
        float split = 0.0f;
        std::uint8_t axis = 0;
        glm::vec3 bmin, bmax;
    };
    std::int32_t buildNode(std::uint32_t begin, std::uint32_t end, int depth);

    std::vector<glm::vec3> points_;
    std::vector<std::uint32_t> order_;
    std::vector<Node> nodes_;
};

} // namespace occlusa
