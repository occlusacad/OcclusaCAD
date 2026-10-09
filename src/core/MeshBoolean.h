#pragma once

#include "core/Mesh.h"
#include "core/MeshTopology.h" // isClosedManifold, signedVolume

#include <string>
#include <vector>

namespace occlusa {

// Boolean operations on closed triangle meshes (Manifold). `error` is empty on success.
struct BooleanResult {
    Mesh mesh;
    std::string error;
    bool ok() const { return error.empty(); }
};

// (union of `add`) minus (union of `subtract`). Every input must be a closed, consistently
// oriented manifold.
BooleanResult meshBoolean(const std::vector<const Mesh*>& add, const std::vector<const Mesh*>& subtract = {});

} // namespace occlusa
