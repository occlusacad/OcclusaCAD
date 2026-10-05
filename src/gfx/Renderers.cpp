#include "gfx/Renderers.h"

#include "core/Log.h"

#include <algorithm>
#include <vector>

namespace occlusa::gfx {

namespace {

constexpr const char* kFullscreenVS = R"GLSL(#version 330 core
out vec2 vUv;
void main() {
    // Full-screen triangle from gl_VertexID.
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    vUv = p;
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)GLSL";

constexpr const char* kBackgroundFS = R"GLSL(#version 330 core
in vec2 vUv;
out vec4 fragColor;
uniform vec3 uTop;
uniform vec3 uBottom;
void main() {
    float t = smoothstep(0.0, 1.0, vUv.y);
    fragColor = vec4(mix(uBottom, uTop, t), 1.0);
}
)GLSL";

constexpr const char* kMeshVS = R"GLSL(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;
uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProj;
uniform mat3 uNormalMatrix;
out vec3 vNormalView;
out vec3 vPosView;
out vec3 vColor;
void main() {
    vColor = aColor;
    vec4 pv = uView * uModel * vec4(aPos, 1.0);
    vPosView = pv.xyz;
    vNormalView = mat3(uView) * (uNormalMatrix * aNormal);
    gl_Position = uProj * pv;
}
)GLSL";

constexpr const char* kMeshFS = R"GLSL(#version 330 core
in vec3 vNormalView;
in vec3 vPosView;
in vec3 vColor;
out vec4 fragColor;
uniform vec3 uColor;
uniform int uVertexColor;
uniform float uOpacity;
uniform int uHighlight;
uniform int uOrtho;
void main() {
    vec3 n = normalize(vNormalView);
    vec3 v = uOrtho == 1 ? vec3(0.0, 0.0, 1.0) : normalize(-vPosView);
    bool back = dot(n, v) < 0.0;
    if (back) n = -n;
    // Headlight plus a soft fill light from above.
    vec3 l1 = v;
    vec3 l2 = normalize(vec3(-0.3, 0.8, 0.5));
    float diff = max(dot(n, l1), 0.0) * 0.75 + max(dot(n, l2), 0.0) * 0.25;
    vec3 h = normalize(l1 + v);
    float spec = pow(max(dot(n, h), 0.0), 48.0) * 0.25;
    vec3 albedo = uVertexColor == 1 ? vColor : uColor;
    vec3 base = back ? albedo * vec3(0.55, 0.45, 0.45) : albedo;
    vec3 c = base * (0.22 + 0.78 * diff) + vec3(spec);
    // Rim to separate the surface from the background.
    float rim = pow(1.0 - max(dot(n, v), 0.0), 3.0);
    c += rim * (uHighlight == 1 ? vec3(0.15, 0.45, 0.9) : vec3(0.08));
    fragColor = vec4(c * uOpacity, uOpacity); // premultiplied alpha
}
)GLSL";

constexpr const char* kRaycastFS = R"GLSL(#version 330 core
in vec2 vUv;
out vec4 fragColor;
uniform sampler3D uVolume;
uniform sampler2D uDepth;
uniform mat4 uInvViewProj;
uniform mat4 uViewProj;
uniform mat4 uWorldToTex;
uniform mat3 uGradToWorld;     // inverse-transpose of the tex->world linear part
uniform vec3 uDims;
uniform int uMode;             // 0 iso, 1 MIP, 2 DVR
uniform float uIso;            // normalized texel value
uniform vec2 uWindow;          // normalized [low, high]
uniform vec3 uColor;
uniform float uOpacity;
uniform vec3 uClipMin;
uniform vec3 uClipMax;
uniform float uStepVoxels;
uniform vec3 uViewDir;

float sampleVol(vec3 p) { return texture(uVolume, p).r; }

vec3 gradientAt(vec3 p) {
    vec3 e = 1.0 / uDims;
    return vec3(sampleVol(p + vec3(e.x, 0, 0)) - sampleVol(p - vec3(e.x, 0, 0)),
                sampleVol(p + vec3(0, e.y, 0)) - sampleVol(p - vec3(0, e.y, 0)),
                sampleVol(p + vec3(0, 0, e.z)) - sampleVol(p - vec3(0, 0, e.z))) * uDims * 0.5;
}

vec3 shade(vec3 pTex, vec3 base) {
    vec3 g = uGradToWorld * gradientAt(pTex);
    float len = length(g);
    if (len < 1e-6) return base * 0.6;
    vec3 n = -g / len;             // outward from dense tissue
    vec3 l = -normalize(uViewDir);
    if (dot(n, l) < 0.0) n = -n;
    float diff = max(dot(n, l), 0.0);
    float fill = max(dot(n, normalize(vec3(0.3, -0.5, 0.8))), 0.0);
    vec3 h = normalize(l + l);
    float spec = pow(max(dot(n, h), 0.0), 40.0) * 0.2;
    return base * (0.2 + 0.65 * diff + 0.15 * fill) + vec3(spec);
}

void main() {
    vec2 ndc = vUv * 2.0 - 1.0;
    vec4 a = uInvViewProj * vec4(ndc, -1.0, 1.0); a /= a.w;
    vec4 b = uInvViewProj * vec4(ndc, 1.0, 1.0);  b /= b.w;
    vec3 worldDir = b.xyz - a.xyz;

    // Stop at opaque geometry already in the depth buffer.
    float sceneDepth = texture(uDepth, vUv).r;
    float tScene = 1.0;
    if (sceneDepth < 1.0) {
        vec4 s = uInvViewProj * vec4(ndc, sceneDepth * 2.0 - 1.0, 1.0); s /= s.w;
        tScene = length(s.xyz - a.xyz) / length(worldDir);
    }

    vec3 o = (uWorldToTex * vec4(a.xyz, 1.0)).xyz;
    vec3 d = (uWorldToTex * vec4(worldDir, 0.0)).xyz;
    vec3 invD = 1.0 / d;
    vec3 t0 = (uClipMin - o) * invD;
    vec3 t1 = (uClipMax - o) * invD;
    vec3 tmin = min(t0, t1), tmax = max(t0, t1);
    float tEnter = max(max(tmin.x, tmin.y), max(tmin.z, 0.0));
    float tExit = min(min(tmax.x, tmax.y), min(tmax.z, tScene));
    if (tExit <= tEnter) discard;

    float rayVoxels = length(d * uDims) * (tExit - tEnter);
    int steps = int(clamp(rayVoxels / uStepVoxels, 1.0, 4096.0));
    float dt = (tExit - tEnter) / float(steps);
    // Jitter the start to hide wood-grain artefacts.
    float jitter = fract(sin(dot(gl_FragCoord.xy, vec2(12.9898, 78.233))) * 43758.5453);
    float t = tEnter + dt * jitter;

    if (uMode == 0) {
        float prevT = t;
        for (int i = 0; i < steps; ++i) {
            vec3 p = o + d * t;
            if (sampleVol(p) >= uIso) {
                float lo = prevT, hi = t;
                for (int k = 0; k < 6; ++k) {
                    float m = 0.5 * (lo + hi);
                    if (sampleVol(o + d * m) >= uIso) hi = m; else lo = m;
                }
                vec3 hit = o + d * hi;
                vec3 world = a.xyz + worldDir * hi;
                vec4 clip = uViewProj * vec4(world, 1.0);
                gl_FragDepth = clamp(clip.z / clip.w * 0.5 + 0.5, 0.0, 1.0);
                fragColor = vec4(shade(hit, uColor), 1.0);
                return;
            }
            prevT = t;
            t += dt;
        }
        discard;
    } else if (uMode == 1) {
        float m = 0.0;
        for (int i = 0; i < steps; ++i) {
            m = max(m, sampleVol(o + d * t));
            t += dt;
        }
        float g = clamp((m - uWindow.x) / max(uWindow.y - uWindow.x, 1e-6), 0.0, 1.0);
        gl_FragDepth = 1.0;
        fragColor = vec4(vec3(g), g);
    } else {
        vec4 acc = vec4(0.0);
        float stepScale = uStepVoxels / 0.5;
        for (int i = 0; i < steps && acc.a < 0.98; ++i) {
            vec3 p = o + d * t;
            float v = sampleVol(p);
            float x = clamp((v - uWindow.x) / max(uWindow.y - uWindow.x, 1e-6), 0.0, 1.0);
            if (x > 0.0) {
                float alpha = clamp(x * x * 0.08 * uOpacity * stepScale, 0.0, 1.0);
                vec3 base = mix(uColor * vec3(0.55, 0.38, 0.30), uColor, x);
                vec3 c = shade(p, base);
                acc.rgb += (1.0 - acc.a) * alpha * c;
                acc.a += (1.0 - acc.a) * alpha;
            }
            t += dt;
        }
        if (acc.a <= 0.001) discard;
        gl_FragDepth = 1.0;
        fragColor = acc;
    }
}
)GLSL";

constexpr const char* kSliceVS = R"GLSL(#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 uViewProj;
out vec3 vWorld;
void main() {
    vWorld = aPos;
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
)GLSL";

constexpr const char* kSliceFS = R"GLSL(#version 330 core
in vec3 vWorld;
out vec4 fragColor;
uniform sampler3D uVolume;
uniform mat4 uWorldToTex;
uniform vec2 uTexToReal;
uniform vec2 uWindow;          // real [low, high]
uniform int uShowThreshold;
uniform float uIso;            // real
uniform vec3 uThresholdColor;
void main() {
    vec3 p = (uWorldToTex * vec4(vWorld, 1.0)).xyz;
    if (any(lessThan(p, vec3(0.0))) || any(greaterThan(p, vec3(1.0)))) discard;
    float real = texture(uVolume, p).r * uTexToReal.x + uTexToReal.y;
    float g = clamp((real - uWindow.x) / max(uWindow.y - uWindow.x, 1e-6), 0.0, 1.0);
    vec3 c = vec3(g);
    if (uShowThreshold == 1 && real >= uIso) c = mix(c, uThresholdColor, 0.45);
    fragColor = vec4(c, 1.0);
}
)GLSL";

} // namespace

// ---------------------------------------------------------------------------

GpuMesh::GpuMesh(const Mesh& mesh)
{
    std::vector<float> interleaved;
    interleaved.reserve(mesh.positions.size() * 9);
    hasColors_ = mesh.colors.size() == mesh.positions.size();
    for (std::size_t i = 0; i < mesh.positions.size(); ++i) {
        const glm::vec3& p = mesh.positions[i];
        const glm::vec3 n = i < mesh.normals.size() ? mesh.normals[i] : glm::vec3(0, 0, 1);
        const glm::vec3 c = hasColors_ ? mesh.colors[i] : glm::vec3(1.0f);
        interleaved.insert(interleaved.end(), {p.x, p.y, p.z, n.x, n.y, n.z, c.x, c.y, c.z});
    }
    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);
    glGenBuffers(1, &vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(interleaved.size() * sizeof(float)), interleaved.data(), GL_STATIC_DRAW);
    glGenBuffers(1, &ibo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh.indices.size() * sizeof(std::uint32_t)), mesh.indices.data(),
                 GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), reinterpret_cast<void*>(6 * sizeof(float)));
    glBindVertexArray(0);
    indexCount_ = static_cast<GLsizei>(mesh.indices.size());
}

GpuMesh::~GpuMesh()
{
    glDeleteBuffers(1, &vbo_);
    glDeleteBuffers(1, &ibo_);
    glDeleteVertexArrays(1, &vao_);
}

void GpuMesh::draw() const
{
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, indexCount_, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

GpuVolume::GpuVolume(const Volume& vol) : geometry_(vol.geometry)
{
    GLint maxSize = 0;
    glGetIntegerv(GL_MAX_3D_TEXTURE_SIZE, &maxSize);
    const auto& d = vol.geometry.dims;
    if (d.x > maxSize || d.y > maxSize || d.z > maxSize)
        log::warn("Volume {}x{}x{} exceeds GL_MAX_3D_TEXTURE_SIZE {}; rendering may fail", d.x, d.y, d.z, maxSize);
    // TODO: bricking or downsampling for volumes beyond the GPU limits / memory.

    glGenTextures(1, &texture_);
    glBindTexture(GL_TEXTURE_3D, texture_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 2);
    glTexImage3D(GL_TEXTURE_3D, 0, GL_R16, d.x, d.y, d.z, 0, GL_RED, GL_UNSIGNED_SHORT, nullptr);
    // Upload slice by slice to keep the temporary conversion buffer small.
    std::vector<std::uint16_t> slice(static_cast<std::size_t>(d.x) * d.y);
    for (int z = 0; z < d.z; ++z) {
        const std::int16_t* src = vol.voxels.data() + static_cast<std::size_t>(z) * slice.size();
        for (std::size_t i = 0; i < slice.size(); ++i)
            slice[i] = static_cast<std::uint16_t>(static_cast<int>(src[i]) + 32768);
        glTexSubImage3D(GL_TEXTURE_3D, 0, 0, 0, z, d.x, d.y, 1, GL_RED, GL_UNSIGNED_SHORT, slice.data());
    }
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glBindTexture(GL_TEXTURE_3D, 0);
    checkGLError("GpuVolume upload");

    // normalized n = (stored + 32768) / 65535  ->  real = stored * slope + intercept
    const double a = 65535.0 * vol.rescaleSlope;
    const double b = -32768.0 * vol.rescaleSlope + vol.rescaleIntercept;
    texToReal_ = glm::vec2(static_cast<float>(a), static_cast<float>(b));
}

GpuVolume::~GpuVolume()
{
    glDeleteTextures(1, &texture_);
}

// ---------------------------------------------------------------------------

FrameContext makeFrameContext(const Camera& camera, int width, int height)
{
    FrameContext ctx;
    ctx.width = std::max(width, 1);
    ctx.height = std::max(height, 1);
    ctx.view = glm::mat4(camera.viewMatrix());
    ctx.projection = glm::mat4(camera.projectionMatrix(static_cast<double>(ctx.width) / ctx.height));
    ctx.eye = glm::vec3(camera.eye());
    ctx.viewDir = glm::vec3(camera.forward());
    return ctx;
}

SceneRenderer::SceneRenderer()
    : background_(kFullscreenVS, kBackgroundFS, "background"),
      mesh_(kMeshVS, kMeshFS, "mesh"),
      raycast_(kFullscreenVS, kRaycastFS, "raycast"),
      slice_(kSliceVS, kSliceFS, "slice")
{
    glGenVertexArrays(1, &sliceVao_);
    glGenBuffers(1, &sliceVbo_);
    glBindVertexArray(sliceVao_);
    glBindBuffer(GL_ARRAY_BUFFER, sliceVbo_);
    glBufferData(GL_ARRAY_BUFFER, 4 * 3 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glBindVertexArray(0);
}

void SceneRenderer::drawBackground(const glm::vec3& top, const glm::vec3& bottom)
{
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glDisable(GL_BLEND);
    background_.use();
    background_.set("uTop", top);
    background_.set("uBottom", bottom);
    glBindVertexArray(emptyVao());
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glDepthMask(GL_TRUE);
}

void SceneRenderer::drawMeshes(const FrameContext& ctx, const std::vector<MeshDraw>& meshes, bool transparentPass)
{
    std::vector<const MeshDraw*> list;
    for (const auto& m : meshes) {
        if (!m.mesh || m.opacity <= 0.0f)
            continue;
        const bool transparent = m.opacity < 0.999f;
        if (transparent == transparentPass)
            list.push_back(&m);
    }
    if (list.empty())
        return;
    if (transparentPass) {
        // Back to front by model origin distance (good enough for a handful of scans).
        std::sort(list.begin(), list.end(), [&](const MeshDraw* a, const MeshDraw* b) {
            return glm::length(glm::vec3(a->model[3]) - ctx.eye) > glm::length(glm::vec3(b->model[3]) - ctx.eye);
        });
    }
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_CULL_FACE);
    if (transparentPass) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
    } else {
        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);
    }
    mesh_.use();
    mesh_.set("uView", ctx.view);
    mesh_.set("uProj", ctx.projection);
    mesh_.set("uOrtho", ctx.projection[3][3] == 1.0f ? 1 : 0);
    for (const MeshDraw* m : list) {
        mesh_.set("uModel", m->model);
        mesh_.set("uNormalMatrix", glm::mat3(glm::transpose(glm::inverse(m->model))));
        mesh_.set("uColor", m->color);
        mesh_.set("uOpacity", m->opacity);
        mesh_.set("uHighlight", m->highlight ? 1 : 0);
        mesh_.set("uVertexColor", m->vertexColors && m->mesh->hasColors() ? 1 : 0);
        if (m->depthBias) {
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(-1.0f, -2.0f);
        }
        m->mesh->draw();
        if (m->depthBias)
            glDisable(GL_POLYGON_OFFSET_FILL);
    }
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void SceneRenderer::drawVolume(const FrameContext& ctx, const VolumeDraw& v, GLuint depthTexture)
{
    if (!v.volume)
        return;
    const glm::mat4 viewProj = ctx.projection * ctx.view;
    const glm::dmat4 w2t = v.volume->geometry().worldToTexture();
    const glm::dmat3 texToWorldLinear = glm::inverse(glm::dmat3(w2t));
    const glm::mat3 gradToWorld = glm::mat3(glm::transpose(glm::inverse(texToWorldLinear)));

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    // Isosurface hits write depth so later transparent meshes are occluded correctly.
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_ALWAYS);
    glDepthMask(v.mode == VolumeMode::Isosurface ? GL_TRUE : GL_FALSE);

    raycast_.use();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_3D, v.volume->texture());
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, depthTexture);
    raycast_.set("uVolume", 0);
    raycast_.set("uDepth", 1);
    raycast_.set("uInvViewProj", glm::inverse(viewProj));
    raycast_.set("uViewProj", viewProj);
    raycast_.set("uWorldToTex", glm::mat4(w2t));
    raycast_.set("uGradToWorld", gradToWorld);
    raycast_.set("uDims", glm::vec3(v.volume->geometry().dims));
    raycast_.set("uMode", static_cast<int>(v.mode));
    raycast_.set("uIso", v.volume->realToTex(v.isoValue));
    raycast_.set("uWindow", glm::vec2(v.volume->realToTex(v.windowCenter - v.windowWidth * 0.5),
                                      v.volume->realToTex(v.windowCenter + v.windowWidth * 0.5)));
    raycast_.set("uColor", v.color);
    raycast_.set("uOpacity", v.opacity);
    raycast_.set("uClipMin", v.clipMin);
    raycast_.set("uClipMax", v.clipMax);
    raycast_.set("uStepVoxels", std::max(v.stepVoxels, 0.1f));
    raycast_.set("uViewDir", ctx.viewDir);
    glBindVertexArray(emptyVao());
    glDrawArrays(GL_TRIANGLES, 0, 3);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_3D, 0);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void SceneRenderer::drawSlice(const FrameContext& ctx, const SliceDraw& s)
{
    if (!s.volume)
        return;
    float verts[12];
    for (int i = 0; i < 4; ++i) {
        verts[i * 3 + 0] = s.corners[i].x;
        verts[i * 3 + 1] = s.corners[i].y;
        verts[i * 3 + 2] = s.corners[i].z;
    }
    glBindBuffer(GL_ARRAY_BUFFER, sliceVbo_);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(verts), verts);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    slice_.use();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_3D, s.volume->texture());
    slice_.set("uVolume", 0);
    slice_.set("uViewProj", ctx.projection * ctx.view);
    slice_.set("uWorldToTex", glm::mat4(s.volume->geometry().worldToTexture()));
    slice_.set("uTexToReal", s.volume->texToReal());
    slice_.set("uWindow", glm::vec2(static_cast<float>(s.windowCenter - s.windowWidth * 0.5), static_cast<float>(s.windowCenter + s.windowWidth * 0.5)));
    slice_.set("uShowThreshold", s.showThreshold ? 1 : 0);
    slice_.set("uIso", static_cast<float>(s.isoValue));
    slice_.set("uThresholdColor", s.thresholdColor);
    glBindVertexArray(sliceVao_);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    glBindVertexArray(0);
    glBindTexture(GL_TEXTURE_3D, 0);
}

void SceneRenderer::render3D(RenderTarget& target, const FrameContext& ctx, const std::vector<MeshDraw>& meshes, const VolumeDraw* volume,
                             const glm::vec3& bgTop, const glm::vec3& bgBottom)
{
    target.bindForRendering();
    glClearColor(0, 0, 0, 1);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawBackground(bgTop, bgBottom);
    drawMeshes(ctx, meshes, false);
    if (volume && volume->volume) {
        target.copyDepth();
        drawVolume(ctx, *volume, target.depthTexture());
    }
    drawMeshes(ctx, meshes, true);
    target.resolve();
    checkGLError("render3D");
}

void SceneRenderer::renderSlice(RenderTarget& target, const FrameContext& ctx, const SliceDraw& slice, const glm::vec3& background)
{
    target.bindForRendering();
    glClearColor(background.r, background.g, background.b, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawSlice(ctx, slice);
    target.resolve();
    checkGLError("renderSlice");
}

} // namespace occlusa::gfx
