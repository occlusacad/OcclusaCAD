#include "core/StlIO.h"

#include <bit>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <vector>

namespace occlusa {
namespace {

static_assert(sizeof(float) == 4);

std::uint32_t readU32LE(const unsigned char* p)
{
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) | (static_cast<std::uint32_t>(p[2]) << 16) |
           (static_cast<std::uint32_t>(p[3]) << 24);
}

float readF32LE(const unsigned char* p)
{
    return std::bit_cast<float>(readU32LE(p));
}

void writeU32LE(std::ofstream& out, std::uint32_t v)
{
    const unsigned char b[4] = {static_cast<unsigned char>(v), static_cast<unsigned char>(v >> 8), static_cast<unsigned char>(v >> 16),
                                static_cast<unsigned char>(v >> 24)};
    out.write(reinterpret_cast<const char*>(b), 4);
}

void writeF32LE(std::ofstream& out, float f)
{
    writeU32LE(out, std::bit_cast<std::uint32_t>(f));
}

Mesh parseBinary(std::span<const unsigned char> bytes, std::uint32_t triCount)
{
    Mesh mesh;
    mesh.positions.reserve(static_cast<std::size_t>(triCount) * 3);
    mesh.indices.reserve(static_cast<std::size_t>(triCount) * 3);
    const unsigned char* p = bytes.data() + 84;
    for (std::uint32_t t = 0; t < triCount; ++t, p += 50) {
        for (int v = 0; v < 3; ++v) {
            const unsigned char* q = p + 12 + v * 12;
            glm::vec3 pos(readF32LE(q), readF32LE(q + 4), readF32LE(q + 8));
            if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z))
                throw MeshIOError("STL contains non-finite coordinates");
            mesh.indices.push_back(static_cast<std::uint32_t>(mesh.positions.size()));
            mesh.positions.push_back(pos);
        }
    }
    return mesh;
}

Mesh parseAscii(std::span<const unsigned char> bytes)
{
    Mesh mesh;
    const char* p = reinterpret_cast<const char*>(bytes.data());
    const char* end = p + bytes.size();

    auto skipSpace = [&] {
        while (p < end && std::isspace(static_cast<unsigned char>(*p)))
            ++p;
    };
    auto token = [&]() -> std::string_view {
        skipSpace();
        const char* start = p;
        while (p < end && !std::isspace(static_cast<unsigned char>(*p)))
            ++p;
        return {start, static_cast<std::size_t>(p - start)};
    };
    auto number = [&]() -> float {
        std::string_view tok = token();
        // std::from_chars for float is not available everywhere (older libc++); use strtof on a copy.
        std::string tmp(tok);
        char* stop = nullptr;
        const float v = std::strtof(tmp.c_str(), &stop);
        if (stop == tmp.c_str())
            throw MeshIOError("Malformed ASCII STL: expected number");
        return v;
    };

    int cornersInFacet = 0;
    while (p < end) {
        std::string_view tok = token();
        if (tok.empty())
            break;
        if (tok == "vertex") {
            glm::vec3 v;
            v.x = number();
            v.y = number();
            v.z = number();
            mesh.indices.push_back(static_cast<std::uint32_t>(mesh.positions.size()));
            mesh.positions.push_back(v);
            ++cornersInFacet;
        } else if (tok == "endfacet") {
            if (cornersInFacet != 3)
                throw MeshIOError("Malformed ASCII STL: facet without exactly 3 vertices");
            cornersInFacet = 0;
        }
    }
    return mesh;
}

void finalize(Mesh& mesh)
{
    if (mesh.indices.empty())
        throw MeshIOError("STL contains no triangles");
    // Weld tolerance relative to model size; ~1e-6 of the extent is well below scanner resolution.
    const double extent = mesh.bounds().diagonal();
    mesh.weldVertices(static_cast<float>(std::max(extent * 1e-7, 1e-6)));
    mesh.computeVertexNormals();
}

} // namespace

Mesh readStlFromMemory(std::span<const unsigned char> bytes)
{
    if (bytes.size() < 15)
        throw MeshIOError("File too small to be an STL");

    // Binary STL files may also start with "solid", so the size check is authoritative.
    if (bytes.size() >= 84) {
        const std::uint32_t triCount = readU32LE(bytes.data() + 80);
        const std::uint64_t expected = 84ULL + 50ULL * triCount;
        if (expected == bytes.size() || (triCount > 0 && expected < bytes.size() && bytes.size() - expected < 64)) {
            Mesh mesh = parseBinary(bytes, triCount);
            finalize(mesh);
            return mesh;
        }
    }
    const std::string_view head(reinterpret_cast<const char*>(bytes.data()), std::min<std::size_t>(bytes.size(), 512));
    if (head.find("solid") != std::string_view::npos && head.find("facet") != std::string_view::npos) {
        Mesh mesh = parseAscii(bytes);
        finalize(mesh);
        return mesh;
    }
    if (bytes.size() >= 84) {
        // Some exporters write an incorrect triangle count. Trust the file length instead.
        const std::uint32_t triCount = static_cast<std::uint32_t>((bytes.size() - 84) / 50);
        if (triCount > 0) {
            Mesh mesh = parseBinary(bytes, triCount);
            finalize(mesh);
            return mesh;
        }
    }
    throw MeshIOError("Unrecognized STL format");
}

Mesh readStl(const std::filesystem::path& path)
{
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in)
        throw MeshIOError("Cannot open file: " + path.string());
    const auto size = static_cast<std::size_t>(in.tellg());
    in.seekg(0);
    std::vector<unsigned char> data(size);
    if (size > 0 && !in.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(size)))
        throw MeshIOError("Failed to read file: " + path.string());
    return readStlFromMemory(data);
}

void writeStlBinary(const std::filesystem::path& path, const Mesh& mesh, const glm::dmat4& transform, const std::string& header)
{
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out)
        throw MeshIOError("Cannot write file: " + path.string());
    char head[80] = {};
    std::memcpy(head, header.data(), std::min<std::size_t>(header.size(), 79));
    out.write(head, 80);
    const auto triCount = static_cast<std::uint32_t>(mesh.triangleCount());
    writeU32LE(out, triCount);
    for (std::size_t t = 0; t < triCount; ++t) {
        glm::dvec3 v[3];
        for (int k = 0; k < 3; ++k)
            v[k] = transformPoint(transform, glm::dvec3(mesh.positions[mesh.indices[t * 3 + k]]));
        glm::dvec3 n = glm::cross(v[1] - v[0], v[2] - v[0]);
        const double len = glm::length(n);
        n = len > 0 ? n / len : glm::dvec3(0, 0, 1);
        writeF32LE(out, static_cast<float>(n.x));
        writeF32LE(out, static_cast<float>(n.y));
        writeF32LE(out, static_cast<float>(n.z));
        for (int k = 0; k < 3; ++k) {
            writeF32LE(out, static_cast<float>(v[k].x));
            writeF32LE(out, static_cast<float>(v[k].y));
            writeF32LE(out, static_cast<float>(v[k].z));
        }
        const unsigned char attr[2] = {0, 0};
        out.write(reinterpret_cast<const char*>(attr), 2);
    }
    if (!out)
        throw MeshIOError("Failed while writing: " + path.string());
}

} // namespace occlusa
