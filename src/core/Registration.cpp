#include "core/Registration.h"

#include "core/Log.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <numeric>

namespace occlusa {
namespace {

// Jacobi eigen-decomposition of a symmetric 4x4 matrix. Returns eigenvalues; eigenvectors in columns of v.
void jacobiEigen4(double a[4][4], double eig[4], double v[4][4])
{
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            v[i][j] = i == j ? 1.0 : 0.0;
    for (int sweep = 0; sweep < 64; ++sweep) {
        double off = 0.0;
        for (int p = 0; p < 4; ++p)
            for (int q = p + 1; q < 4; ++q)
                off += a[p][q] * a[p][q];
        if (off < 1e-30)
            break;
        for (int p = 0; p < 4; ++p) {
            for (int q = p + 1; q < 4; ++q) {
                if (std::abs(a[p][q]) < 1e-300)
                    continue;
                const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                const double t = (theta >= 0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
                const double c = 1.0 / std::sqrt(t * t + 1.0), s = t * c;
                for (int k = 0; k < 4; ++k) {
                    const double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < 4; ++k) {
                    const double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 4; ++k) {
                    const double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
        }
    }
    for (int i = 0; i < 4; ++i)
        eig[i] = a[i][i];
}

// Solve the symmetric 6x6 system A x = b with partial pivoting. Returns false if singular.
bool solve6(double A[6][6], double b[6], double x[6])
{
    double M[6][7];
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j)
            M[i][j] = A[i][j];
        M[i][6] = b[i];
    }
    for (int col = 0; col < 6; ++col) {
        int piv = col;
        for (int r = col + 1; r < 6; ++r)
            if (std::abs(M[r][col]) > std::abs(M[piv][col]))
                piv = r;
        if (std::abs(M[piv][col]) < 1e-12)
            return false;
        if (piv != col)
            for (int k = 0; k < 7; ++k)
                std::swap(M[col][k], M[piv][k]);
        for (int r = col + 1; r < 6; ++r) {
            const double f = M[r][col] / M[col][col];
            for (int k = col; k < 7; ++k)
                M[r][k] -= f * M[col][k];
        }
    }
    for (int i = 5; i >= 0; --i) {
        double s = M[i][6];
        for (int k = i + 1; k < 6; ++k)
            s -= M[i][k] * x[k];
        x[i] = s / M[i][i];
    }
    return true;
}

glm::dmat4 rotationAbout(const glm::dvec3& center, const glm::dmat3& r, const glm::dvec3& t)
{
    // x -> R (x - c) + c + t
    glm::dmat4 m(r);
    m[3] = glm::dvec4(center - r * center + t, 1.0);
    return m;
}

glm::dmat3 rodrigues(const glm::dvec3& w)
{
    const double angle = glm::length(w);
    if (angle < 1e-15)
        return glm::dmat3(1.0);
    return glm::dmat3(glm::rotate(glm::dmat4(1.0), angle, w / angle));
}

} // namespace

std::optional<PointPairResult> rigidFromPointPairs(const std::vector<glm::dvec3>& src, const std::vector<glm::dvec3>& dst)
{
    const std::size_t n = std::min(src.size(), dst.size());
    if (n < 3)
        return std::nullopt;
    glm::dvec3 cs(0.0), cd(0.0);
    for (std::size_t i = 0; i < n; ++i) {
        cs += src[i];
        cd += dst[i];
    }
    cs /= static_cast<double>(n);
    cd /= static_cast<double>(n);

    // Reject (near) collinear configurations: the rotation about the line would be undetermined.
    {
        glm::dmat3 cov(0.0);
        for (std::size_t i = 0; i < n; ++i) {
            const glm::dvec3 a = src[i] - cs;
            cov += glm::outerProduct(a, a);
        }
        // Second largest spread must be non-trivial: use the area spanned by the points.
        double maxArea = 0.0;
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = i + 1; j < n; ++j)
                for (std::size_t k = j + 1; k < n; ++k)
                    maxArea = std::max(maxArea, glm::length(glm::cross(src[j] - src[i], src[k] - src[i])));
        double extent = 0.0;
        for (std::size_t i = 0; i < n; ++i)
            extent = std::max(extent, glm::length(src[i] - cs));
        if (extent <= 0.0 || maxArea < 1e-3 * extent * extent)
            return std::nullopt;
    }

    double S[3][3] = {};
    for (std::size_t i = 0; i < n; ++i) {
        const glm::dvec3 a = src[i] - cs, b = dst[i] - cd;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                S[r][c] += a[r] * b[c];
    }
    const double Sxx = S[0][0], Sxy = S[0][1], Sxz = S[0][2];
    const double Syx = S[1][0], Syy = S[1][1], Syz = S[1][2];
    const double Szx = S[2][0], Szy = S[2][1], Szz = S[2][2];
    double N[4][4] = {
        {Sxx + Syy + Szz, Syz - Szy, Szx - Sxz, Sxy - Syx},
        {Syz - Szy, Sxx - Syy - Szz, Sxy + Syx, Szx + Sxz},
        {Szx - Sxz, Sxy + Syx, -Sxx + Syy - Szz, Syz + Szy},
        {Sxy - Syx, Szx + Sxz, Syz + Szy, -Sxx - Syy + Szz},
    };
    double eig[4], V[4][4];
    jacobiEigen4(N, eig, V);
    int best = 0;
    for (int i = 1; i < 4; ++i)
        if (eig[i] > eig[best])
            best = i;
    glm::dquat q(V[0][best], V[1][best], V[2][best], V[3][best]); // (w, x, y, z)
    q = glm::normalize(q);
    const glm::dmat3 R = glm::mat3_cast(q);

    PointPairResult res;
    res.transform = glm::dmat4(R);
    res.transform[3] = glm::dvec4(cd - R * cs, 1.0);
    double sum = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double e = glm::length(transformPoint(res.transform, src[i]) - dst[i]);
        res.residuals.push_back(e);
        sum += e * e;
    }
    res.rms = std::sqrt(sum / static_cast<double>(n));
    return res;
}

IcpTarget IcpTarget::fromMesh(const Mesh& worldMesh)
{
    IcpTarget t;
    t.normals = worldMesh.normals;
    if (t.normals.size() != worldMesh.positions.size()) {
        Mesh copy = worldMesh;
        copy.computeVertexNormals();
        t.normals = copy.normals;
    }
    t.tree.build(worldMesh.positions);
    return t;
}

IcpResult refineIcp(const Mesh& source, const glm::dmat4& initial, const IcpTarget& target, const IcpOptions& opt, const ProgressFn& progress)
{
    IcpResult result;
    result.transform = initial;
    if (target.empty() || source.positions.empty())
        return result;

    // Source samples (local coordinates).
    std::vector<glm::dvec3> samples;
    std::vector<glm::dvec3> sampleNormals;
    {
        const auto idx = subsampleVertices(source, opt.maxSourcePoints * (opt.sourceRegion.valid() ? 4 : 1));
        for (auto i : idx) {
            const glm::dvec3 p(source.positions[i]);
            if (opt.sourceRegion.valid()) {
                const glm::dvec3 w = transformPoint(initial, p);
                if (glm::any(glm::lessThan(w, opt.sourceRegion.min)) || glm::any(glm::greaterThan(w, opt.sourceRegion.max)))
                    continue;
            }
            samples.push_back(p);
            sampleNormals.push_back(i < source.normals.size() ? glm::dvec3(source.normals[i]) : glm::dvec3(0.0));
        }
        if (samples.size() > opt.maxSourcePoints) {
            std::vector<glm::dvec3> s2, n2;
            const double stride = static_cast<double>(samples.size()) / static_cast<double>(opt.maxSourcePoints);
            for (std::size_t k = 0; k < opt.maxSourcePoints; ++k) {
                const auto j = static_cast<std::size_t>(static_cast<double>(k) * stride);
                s2.push_back(samples[j]);
                n2.push_back(sampleNormals[j]);
            }
            samples = std::move(s2);
            sampleNormals = std::move(n2);
        }
    }
    if (samples.size() < 10)
        return result;

    struct Pair {
        glm::dvec3 p, q, n;
        double dist;
    };
    std::vector<Pair> pairs;
    pairs.reserve(samples.size());
    glm::dmat4 T = initial;
    double stageDist = opt.maxCorrespondenceDistance;
    const double finalDist = std::min(std::max(opt.finalCorrespondenceDistance, 1e-3), opt.maxCorrespondenceDistance);
    const double minCos = std::cos(glm::radians(75.0));

    for (int it = 0; it < opt.maxIterations; ++it) {
        reportProgress(progress, static_cast<float>(it) / static_cast<float>(opt.maxIterations), std::format("ICP iteration {}", it + 1));
        const glm::dmat3 R(T);
        const float maxDist = static_cast<float>(stageDist);
        pairs.clear();
        for (std::size_t i = 0; i < samples.size(); ++i) {
            const glm::dvec3 p = transformPoint(T, samples[i]);
            float d2 = 0.0f;
            const std::int64_t j = target.tree.nearest(glm::vec3(p), maxDist, &d2);
            if (j < 0)
                continue;
            const glm::dvec3 n(target.normals[static_cast<std::size_t>(j)]);
            const glm::dvec3 sn = R * sampleNormals[i];
            // Reject pairs whose surface orientations disagree strongly (sign-agnostic: STL winding may be flipped).
            if (glm::dot(sn, sn) > 0.0 && std::abs(glm::dot(sn, n)) < minCos)
                continue;
            pairs.push_back(Pair{p, glm::dvec3(target.tree.points()[static_cast<std::size_t>(j)]), n, std::sqrt(static_cast<double>(d2))});
        }
        if (pairs.size() < 6)
            break;
        std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) { return a.dist < b.dist; });
        const std::size_t keep = std::max<std::size_t>(6, static_cast<std::size_t>(static_cast<double>(pairs.size()) * opt.trimFraction));
        pairs.resize(std::min(keep, pairs.size()));

        glm::dvec3 center(0.0);
        for (const auto& pr : pairs)
            center += pr.p;
        center /= static_cast<double>(pairs.size());

        glm::dmat4 delta(1.0);
        bool solved = false;
        if (opt.pointToPlane) {
            double A[6][6] = {}, b[6] = {}, x[6] = {};
            for (const auto& pr : pairs) {
                const glm::dvec3 p = pr.p - center;
                const glm::dvec3 c = glm::cross(p, pr.n);
                const double row[6] = {c.x, c.y, c.z, pr.n.x, pr.n.y, pr.n.z};
                const double r = glm::dot(pr.q - pr.p, pr.n);
                for (int i = 0; i < 6; ++i) {
                    for (int j = 0; j < 6; ++j)
                        A[i][j] += row[i] * row[j];
                    b[i] += row[i] * r;
                }
            }
            // Light damping keeps the system solvable when the surface constrains a direction poorly.
            const double trace = A[0][0] + A[1][1] + A[2][2] + A[3][3] + A[4][4] + A[5][5];
            for (int i = 0; i < 6; ++i)
                A[i][i] += 1e-9 * trace + 1e-12;
            if (solve6(A, b, x)) {
                delta = rotationAbout(center, rodrigues(glm::dvec3(x[0], x[1], x[2])), glm::dvec3(x[3], x[4], x[5]));
                solved = true;
            }
        }
        if (!solved) {
            std::vector<glm::dvec3> P, Q;
            P.reserve(pairs.size());
            Q.reserve(pairs.size());
            for (const auto& pr : pairs) {
                P.push_back(pr.p);
                Q.push_back(pr.q);
            }
            if (auto pp = rigidFromPointPairs(P, Q))
                delta = pp->transform;
            else
                break;
        }
        T = orthonormalize(delta * T);
        result.iterations = it + 1;

        const double dt = glm::length(glm::dvec3(delta[3]) - (glm::dvec3(0.0)));
        const double cosAngle = std::clamp((delta[0][0] + delta[1][1] + delta[2][2] - 1.0) * 0.5, -1.0, 1.0);
        const double dAngle = glm::degrees(std::acos(cosAngle));
        if (dt < opt.convergenceTranslation && dAngle < opt.convergenceRotationDeg) {
            if (stageDist > finalDist * 1.0001) {
                stageDist = std::max(finalDist, stageDist * 0.5);
                continue;
            }
            result.converged = true;
            break;
        }
    }

    // Final statistics against the refined transform, at the final search distance.
    const float maxDist = static_cast<float>(stageDist);
    double sum = 0.0, sumSq = 0.0;
    std::size_t inliers = 0;
    for (const auto& s : samples) {
        const glm::dvec3 p = transformPoint(T, s);
        float d2 = 0.0f;
        if (target.tree.nearest(glm::vec3(p), maxDist, &d2) >= 0) {
            const double d = std::sqrt(static_cast<double>(d2));
            sum += d;
            sumSq += d * d;
            ++inliers;
        }
    }
    result.transform = T;
    if (inliers > 0) {
        result.meanDistance = sum / static_cast<double>(inliers);
        result.rms = std::sqrt(sumSq / static_cast<double>(inliers));
    }
    result.inlierFraction = static_cast<double>(inliers) / static_cast<double>(samples.size());
    log::info("ICP: {} iterations, rms {:.3f} mm, inliers {:.1f}%{}", result.iterations, result.rms, result.inlierFraction * 100.0,
              result.converged ? " (converged)" : "");
    return result;
}

DeviationStats measureDeviation(const Mesh& source, const glm::dmat4& transform, const IcpTarget& target, double tolerance, double maxDistance,
                                std::size_t maxSamples)
{
    DeviationStats st;
    if (target.empty())
        return st;
    std::vector<double> d;
    for (auto i : subsampleVertices(source, maxSamples)) {
        const glm::dvec3 p = transformPoint(transform, glm::dvec3(source.positions[i]));
        float d2 = 0.0f;
        if (target.tree.nearest(glm::vec3(p), static_cast<float>(maxDistance), &d2) >= 0)
            d.push_back(std::sqrt(static_cast<double>(d2)));
        else
            d.push_back(maxDistance);
    }
    if (d.empty())
        return st;
    st.samples = d.size();
    double sum = 0.0, sq = 0.0;
    std::size_t within = 0;
    for (double v : d) {
        sum += v;
        sq += v * v;
        within += v <= tolerance ? 1 : 0;
    }
    st.mean = sum / static_cast<double>(d.size());
    st.rms = std::sqrt(sq / static_cast<double>(d.size()));
    st.fractionWithin = static_cast<double>(within) / static_cast<double>(d.size());
    std::sort(d.begin(), d.end());
    st.median = d[d.size() / 2];
    st.p90 = d[std::min(d.size() - 1, d.size() * 9 / 10)];
    return st;
}

} // namespace occlusa
