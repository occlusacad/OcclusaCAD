// occlusa_toothlib: create, inspect and export tooth libraries.
//
//   occlusa_toothlib list   [--scan <dir>]                     libraries known to OcclusaCAD
//   occlusa_toothlib export --library <id> --out <dir>         write a library folder (built-in or scanned)
//   occlusa_toothlib create --in <dir> --name <name> [--author A] [--license L] [--description D]
//                                                              add library.json to a folder of <FDI>.stl files
//                                                              (teeth already in the tooth frame)
//   occlusa_toothlib info   --in <dir>                         check a library folder and print tooth dimensions
//   occlusa_toothlib preview --library <id> --out <file.stl>   all teeth of a library side by side in one STL
//
// Tooth frame: x = mesial, y = buccal / labial, z = occlusal, origin on the tooth axis at the
// cervical line. See docs/TOOTH_LIBRARIES.md.

#include "core/CommandLine.h"
#include "core/Dental.h"
#include "core/MeshTopology.h"
#include "core/Platform.h"
#include "core/StlIO.h"
#include "core/crown/ToothLibrary.h"

#include <cstdio>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;
using namespace occlusa;
using namespace occlusa::crown;

namespace {

int usage()
{
    std::fprintf(stderr,
                 "usage:\n"
                 "  occlusa_toothlib list   [--scan <dir>]\n"
                 "  occlusa_toothlib export --library <id> --out <dir> [--scan <dir>]\n"
                 "  occlusa_toothlib create --in <dir> --name <name> [--author A] [--license L] [--description D]\n"
                 "  occlusa_toothlib info   --in <dir>\n"
                 "  occlusa_toothlib preview --library <id> --out <file.stl> [--scan <dir>]\n");
    return 2;
}

void printShape(int fdi, const Mesh& mesh, std::optional<double> cervical)
{
    const auto shape = sampleToothShape(mesh, cervical);
    std::printf("  %2d  %-24s MD %5.2f  BL %5.2f  height %5.2f  contour %3.0f%%  %6zu triangles%s\n", fdi,
                std::string(toString(toothKindFromFdi(fdi))).c_str(), shape->halfMesial + shape->halfDistal, shape->halfBuccal + shape->halfLingual,
                shape->height, shape->contourHeight * 100.0, mesh.triangleCount(), isClosedManifold(mesh) ? "" : "  (not closed)");
}

std::vector<int> standardTeeth()
{
    std::vector<int> t;
    for (int q : {1, 3})
        for (int p = 1; p <= 7; ++p)
            t.push_back(q * 10 + p);
    return t;
}

} // namespace

int main(int argc, char** argv)
{
    const CommandLine cl = CommandLine::fromMain(argc, argv);
    if (cl.positional().empty())
        return usage();
    const std::string cmd = cl.positional().front();
    auto& registry = ToothLibraryRegistry::instance();
    if (auto dir = cl.get("scan"))
        registry.scanDirectory(platform::pathFromUtf8(*dir));
    try {
        if (cmd == "list") {
            for (const auto& lib : registry.libraries())
                std::printf("%-24s %-28s %s%s\n", lib.id.c_str(), lib.name.c_str(), lib.builtIn ? "built-in" : platform::pathToUtf8(lib.folder).c_str(),
                            lib.license.empty() ? "" : ("  [" + lib.license + "]").c_str());
            return 0;
        }
        if (cmd == "export") {
            if (!cl.get("library") || !cl.get("out"))
                return usage();
            const auto info = registry.find(*cl.get("library"));
            if (!info)
                throw std::runtime_error("Unknown library " + *cl.get("library"));
            std::vector<std::pair<int, Mesh>> teeth;
            for (int fdi : info->builtIn ? standardTeeth() : info->teeth)
                if (auto mesh = registry.toothMesh(info->id, toothKindFromFdi(fdi), dental::isUpper(fdi)))
                    teeth.emplace_back(fdi, *mesh);
            ToothLibraryManifest m;
            m.id = info->id;
            m.name = info->name;
            m.author = info->author;
            m.license = info->license;
            m.description = info->description;
            const fs::path out = platform::pathFromUtf8(*cl.get("out"));
            writeToothLibrary(out, m, teeth);
            std::printf("Wrote %zu teeth of %s to %s\n", teeth.size(), info->name.c_str(), platform::pathToUtf8(out).c_str());
            return 0;
        }
        if (cmd == "create") {
            if (!cl.get("in") || !cl.get("name"))
                return usage();
            const fs::path dir = platform::pathFromUtf8(*cl.get("in"));
            ToothLibraryManifest m = manifestFromStlFolder(dir, *cl.get("name"));
            m.author = cl.get("author").value_or("");
            m.license = cl.get("license").value_or("");
            m.description = cl.get("description").value_or("");
            writeToothLibraryManifest(dir, m);
            std::printf("Created %s with %zu teeth (id %s):\n", platform::pathToUtf8(dir / "library.json").c_str(), m.teeth.size(), m.id.c_str());
            for (const auto& t : m.teeth)
                printShape(t.fdi, loadLibraryTooth(dir, t), t.cervicalZ);
            return 0;
        }
        if (cmd == "info") {
            if (!cl.get("in"))
                return usage();
            const fs::path dir = platform::pathFromUtf8(*cl.get("in"));
            const ToothLibraryManifest m = readToothLibraryManifest(dir);
            std::printf("%s (id %s)\n  author: %s\n  license: %s\n  %s\n", m.name.c_str(), m.id.c_str(), m.author.c_str(), m.license.c_str(),
                        m.description.c_str());
            for (const auto& t : m.teeth)
                printShape(t.fdi, loadLibraryTooth(dir, t), t.cervicalZ);
            return 0;
        }
        if (cmd == "preview") {
            if (!cl.get("library") || !cl.get("out"))
                return usage();
            // Upper teeth 17..11 in the top row, lower 47..41 below, occlusal side up, spaced by width.
            Mesh sheet;
            for (int row = 0; row < 2; ++row) {
                double x = 0.0;
                for (int p = 7; p >= 1; --p) {
                    const bool upper = row == 0;
                    auto mesh = registry.toothMesh(*cl.get("library"), toothKindFromFdi(p), upper);
                    if (!mesh)
                        continue;
                    const Aabb box = mesh->bounds();
                    const double w = box.size().x;
                    const auto baseIndex = static_cast<std::uint32_t>(sheet.positions.size());
                    for (const auto& q : mesh->positions)
                        sheet.positions.emplace_back(static_cast<float>(x - box.min.x + q.x), static_cast<float>(row == 0 ? 14.0 + q.y : q.y), q.z);
                    for (auto i : mesh->indices)
                        sheet.indices.push_back(baseIndex + i);
                    x += w + 2.0;
                }
            }
            sheet.computeVertexNormals();
            writeStlBinary(platform::pathFromUtf8(*cl.get("out")), sheet, glm::dmat4(1.0), "OcclusaCAD tooth library preview");
            std::printf("Wrote %s\n", cl.get("out")->c_str());
            return 0;
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return usage();
}
