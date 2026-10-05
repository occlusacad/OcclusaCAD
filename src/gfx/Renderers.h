#pragma once

#include "core/Mesh.h"
#include "core/Volume.h"
#include "gfx/Camera.h"
#include "gfx/GlObjects.h"

#include <memory>
#include <vector>

namespace occlusa::gfx {

// ---------------------------------------------------------------------------
// GPU resources
// ---------------------------------------------------------------------------

class GpuMesh {
public:
    explicit GpuMesh(const Mesh& mesh);
    ~GpuMesh();
    GpuMesh(const GpuMesh&) = delete;
    GpuMesh& operator=(const GpuMesh&) = delete;

    void draw() const;
    std::size_t triangleCount() const { return indexCount_ / 3; }
    bool hasColors() const { return hasColors_; }

private:
    GLuint vao_ = 0, vbo_ = 0, ibo_ = 0;
    bool hasColors_ = false;
    GLsizei indexCount_ = 0;
};

class GpuVolume {
public:
    explicit GpuVolume(const Volume& volume);
    ~GpuVolume();
    GpuVolume(const GpuVolume&) = delete;
    GpuVolume& operator=(const GpuVolume&) = delete;

    GLuint texture() const { return texture_; }
    const VolumeGeometry& geometry() const { return geometry_; }
    // real value = a * normalizedTexel + b
    glm::vec2 texToReal() const { return texToReal_; }
    float realToTex(double real) const { return static_cast<float>((real - texToReal_.y) / texToReal_.x); }

private:
    GLuint texture_ = 0;
    VolumeGeometry geometry_;
    glm::vec2 texToReal_{1.0f, 0.0f};
};

// ---------------------------------------------------------------------------
// Draw parameters
// ---------------------------------------------------------------------------

struct FrameContext {
    glm::mat4 view{1.0f};
    glm::mat4 projection{1.0f};
    glm::vec3 eye{0.0f};
    glm::vec3 viewDir{0.0f, 0.0f, -1.0f};
    int width = 1, height = 1;
};

struct MeshDraw {
    const GpuMesh* mesh = nullptr;
    glm::mat4 model{1.0f};
    glm::vec3 color{0.85f, 0.85f, 0.8f};
    float opacity = 1.0f;
    bool highlight = false; // e.g. selected / being registered
    bool vertexColors = false; // use the mesh's per-vertex colours
    bool depthBias = false;    // pull towards the camera (overlays on coincident surfaces)
};

enum class VolumeMode { Isosurface = 0, MaximumIntensity = 1, DirectVolume = 2 };

struct VolumeDraw {
    const GpuVolume* volume = nullptr;
    VolumeMode mode = VolumeMode::Isosurface;
    double isoValue = 500.0;        // real units
    double windowCenter = 1000.0;   // real units (MIP / DVR mapping)
    double windowWidth = 3000.0;
    glm::vec3 color{0.93f, 0.89f, 0.80f};
    float opacity = 1.0f;           // DVR opacity scale
    glm::vec3 clipMin{0.0f};        // texture space crop box
    glm::vec3 clipMax{1.0f};
    float stepVoxels = 0.5f;        // sampling distance in voxels (larger while interacting)
};

struct SliceDraw {
    const GpuVolume* volume = nullptr;
    glm::vec3 corners[4];           // world-space quad, counter-clockwise
    double windowCenter = 1000.0;
    double windowWidth = 3000.0;
    bool showThreshold = false;     // tint voxels above isoValue
    double isoValue = 500.0;
    glm::vec3 thresholdColor{0.25f, 0.7f, 1.0f};
};

// ---------------------------------------------------------------------------
// Renderer (owns shaders; one instance per GL context)
// ---------------------------------------------------------------------------

class SceneRenderer {
public:
    SceneRenderer();

    void drawBackground(const glm::vec3& top, const glm::vec3& bottom);
    void drawMeshes(const FrameContext& ctx, const std::vector<MeshDraw>& meshes, bool transparentPass);
    // Requires the target's depth copy to be current (RenderTarget::copyDepth()).
    void drawVolume(const FrameContext& ctx, const VolumeDraw& vol, GLuint depthTexture);
    void drawSlice(const FrameContext& ctx, const SliceDraw& slice);

    // Full 3D frame: background, opaque meshes, volume, transparent meshes, resolve.
    void render3D(RenderTarget& target, const FrameContext& ctx, const std::vector<MeshDraw>& meshes, const VolumeDraw* volume,
                  const glm::vec3& bgTop, const glm::vec3& bgBottom);
    void renderSlice(RenderTarget& target, const FrameContext& ctx, const SliceDraw& slice, const glm::vec3& background);

private:
    ShaderProgram background_;
    ShaderProgram mesh_;
    ShaderProgram raycast_;
    ShaderProgram slice_;
    GLuint sliceVao_ = 0, sliceVbo_ = 0;
};

// Build a FrameContext from a camera for a viewport size in pixels.
FrameContext makeFrameContext(const Camera& camera, int width, int height);

} // namespace occlusa::gfx
