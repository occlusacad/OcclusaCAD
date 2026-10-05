#include "core/KdTree.h"

#include <algorithm>
#include <limits>

namespace occlusa {

namespace {
constexpr std::uint32_t kLeafSize = 12;

float boxDistanceSq(const glm::vec3& p, const glm::vec3& mn, const glm::vec3& mx)
{
    const glm::vec3 d = glm::max(glm::max(mn - p, glm::vec3(0.0f)), p - mx);
    return glm::dot(d, d);
}
} // namespace

void KdTree::build(std::vector<glm::vec3> points)
{
    points_ = std::move(points);
    order_.resize(points_.size());
    for (std::uint32_t i = 0; i < order_.size(); ++i)
        order_[i] = i;
    nodes_.clear();
    if (!points_.empty()) {
        nodes_.reserve(2 * points_.size() / kLeafSize + 1);
        buildNode(0, static_cast<std::uint32_t>(points_.size()), 0);
    }
}

std::int32_t KdTree::buildNode(std::uint32_t begin, std::uint32_t end, int depth)
{
    Node node;
    node.begin = begin;
    node.end = end;
    node.bmin = glm::vec3(std::numeric_limits<float>::max());
    node.bmax = glm::vec3(std::numeric_limits<float>::lowest());
    for (std::uint32_t i = begin; i < end; ++i) {
        node.bmin = glm::min(node.bmin, points_[order_[i]]);
        node.bmax = glm::max(node.bmax, points_[order_[i]]);
    }
    const auto index = static_cast<std::int32_t>(nodes_.size());
    nodes_.push_back(node);
    if (end - begin <= kLeafSize || depth > 48)
        return index;

    const glm::vec3 ext = node.bmax - node.bmin;
    const int axis = (ext.x >= ext.y && ext.x >= ext.z) ? 0 : (ext.y >= ext.z ? 1 : 2);
    const std::uint32_t mid = begin + (end - begin) / 2;
    std::nth_element(order_.begin() + begin, order_.begin() + mid, order_.begin() + end,
                     [&](std::uint32_t a, std::uint32_t b) { return points_[a][axis] < points_[b][axis]; });
    const float split = points_[order_[mid]][axis];
    const std::int32_t left = buildNode(begin, mid, depth + 1);
    const std::int32_t right = buildNode(mid, end, depth + 1);
    nodes_[index].axis = static_cast<std::uint8_t>(axis);
    nodes_[index].split = split;
    nodes_[index].left = left;
    nodes_[index].right = right;
    return index;
}

std::int64_t KdTree::nearest(const glm::vec3& q, float maxDistance, float* outDistanceSq) const
{
    if (nodes_.empty())
        return -1;
    float best = maxDistance * maxDistance;
    std::int64_t bestIdx = -1;
    std::int32_t stack[128];
    int sp = 0;
    stack[sp++] = 0;
    while (sp > 0) {
        const Node& n = nodes_[stack[--sp]];
        if (boxDistanceSq(q, n.bmin, n.bmax) > best)
            continue;
        if (n.left < 0) {
            for (std::uint32_t i = n.begin; i < n.end; ++i) {
                const glm::vec3 d = points_[order_[i]] - q;
                const float d2 = glm::dot(d, d);
                if (d2 <= best) {
                    best = d2;
                    bestIdx = order_[i];
                }
            }
            continue;
        }
        // Visit the nearer child last so it is popped first.
        const bool goLeft = q[n.axis] < n.split;
        if (sp + 2 > 128)
            continue;
        stack[sp++] = goLeft ? n.right : n.left;
        stack[sp++] = goLeft ? n.left : n.right;
    }
    if (outDistanceSq && bestIdx >= 0)
        *outDistanceSq = best;
    return bestIdx;
}

} // namespace occlusa
