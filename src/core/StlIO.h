#pragma once

#include "core/Mesh.h"

#include <filesystem>
#include <span>
#include <stdexcept>
#include <string>

namespace occlusa {

class MeshIOError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Reads binary or ASCII STL. Vertices are welded and normals computed.
Mesh readStl(const std::filesystem::path& path);
Mesh readStlFromMemory(std::span<const unsigned char> bytes);

// Writes binary STL. If `transform` is not identity, vertices are transformed on write.
void writeStlBinary(const std::filesystem::path& path, const Mesh& mesh, const glm::dmat4& transform = glm::dmat4(1.0),
                    const std::string& header = "OcclusaCAD");

} // namespace occlusa
