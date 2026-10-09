#include "core/MeshBoolean.h"

#include <manifold/manifold.h>

namespace occlusa {

namespace {

manifold::Manifold toManifold(const Mesh& mesh)
{
    manifold::MeshGL gl;
    gl.numProp = 3;
    gl.vertProperties.reserve(mesh.positions.size() * 3);
    for (const auto& p : mesh.positions)
        gl.vertProperties.insert(gl.vertProperties.end(), {p.x, p.y, p.z});
    gl.triVerts = mesh.indices;
    gl.Merge(); // STL-style meshes repeat vertices per triangle
    return manifold::Manifold(gl);
}

Mesh fromManifold(const manifold::Manifold& man)
{
    const manifold::MeshGL gl = man.GetMeshGL();
    Mesh m;
    const std::size_t n = gl.vertProperties.size() / gl.numProp;
    m.positions.reserve(n);
    for (std::size_t i = 0; i < n; ++i)
        m.positions.emplace_back(gl.vertProperties[i * gl.numProp], gl.vertProperties[i * gl.numProp + 1], gl.vertProperties[i * gl.numProp + 2]);
    m.indices.assign(gl.triVerts.begin(), gl.triVerts.end());
    m.computeVertexNormals();
    return m;
}

std::string statusText(manifold::Manifold::Error e)
{
    using E = manifold::Manifold::Error;
    switch (e) {
    case E::NoError: return {};
    case E::NotManifold: return "a part is not a closed manifold";
    case E::VertexOutOfBounds: return "invalid vertex index";
    case E::NonFiniteVertex: return "non-finite vertex";
    default: return "geometry error";
    }
}

} // namespace

BooleanResult meshBoolean(const std::vector<const Mesh*>& add, const std::vector<const Mesh*>& subtract)
{
    BooleanResult r;
    auto convert = [&](const std::vector<const Mesh*>& list, std::vector<manifold::Manifold>& out) {
        for (const Mesh* m : list) {
            out.push_back(toManifold(*m));
            if (out.back().Status() != manifold::Manifold::Error::NoError) {
                r.error = statusText(out.back().Status());
                return false;
            }
        }
        return true;
    };
    std::vector<manifold::Manifold> a, s;
    if (add.empty()) {
        r.error = "nothing to combine";
        return r;
    }
    if (!convert(add, a) || !convert(subtract, s))
        return r;
    manifold::Manifold result = manifold::Manifold::BatchBoolean(a, manifold::OpType::Add);
    if (!s.empty())
        result = result - manifold::Manifold::BatchBoolean(s, manifold::OpType::Add);
    if (result.Status() != manifold::Manifold::Error::NoError) {
        r.error = statusText(result.Status());
        return r;
    }
    r.mesh = fromManifold(result);
    if (r.mesh.empty())
        r.error = "the result is empty";
    return r;
}

} // namespace occlusa
