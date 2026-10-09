// occlusa_implantlib: inspect and export implant libraries (custom abutments).
//
//   occlusa_implantlib list   [--scan <dir>]                    libraries known to OcclusaCAD
//   occlusa_implantlib export --library <id> --out <dir> [--id <id>] [--name <name>] [--scan <dir>]
//                                                             write a library folder in the OcclusaCAD format
//   occlusa_implantlib info   --in <dir>                        check a library folder and print its connections
//
// Implant frame: origin at the implant platform on the implant axis, +z out of the implant.
// See docs/IMPLANT_LIBRARIES.md.

#include "core/CommandLine.h"
#include "core/MeshTopology.h"
#include "core/Platform.h"
#include "core/implant/ImplantLibrary.h"

#include <cstdio>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;
using namespace occlusa;
using namespace occlusa::implant;

namespace {

int usage()
{
    std::fprintf(stderr,
                 "usage:\n"
                 "  occlusa_implantlib list   [--scan <dir>]\n"
                 "  occlusa_implantlib export --library <id> --out <dir> [--id <id>] [--name <name>] [--scan <dir>]\n"
                 "  occlusa_implantlib info   --in <dir>\n");
    return 2;
}

void printConnections(ImplantLibraryRegistry& reg, const LibraryInfo& lib)
{
    for (const auto& c : lib.connections) {
        std::printf("  %-10s %-28s platform %4.2f mm", c.id.c_str(), (c.system + " " + c.name).c_str(), c.platformDiameter);
        try {
            const auto conn = reg.connection(lib.id, c.id);
            std::printf("  top \xC3\x98%.2f at %.2f mm  interface %zu tri%s  scan body %zu tri", conn->topRadius * 2.0, conn->topHeight,
                        conn->interfaceMesh->triangleCount(), conn->interfaceClosed ? "" : " (NOT closed)", conn->scanBody->triangleCount());
            if (conn->screwChannel)
                std::printf("  +screw channel");
            if (conn->minThickness)
                std::printf("  +min thickness");
            if (conn->blank)
                std::printf("  +blank");
            std::printf("\n");
        } catch (const std::exception& e) {
            std::printf("  ERROR: %s\n", e.what());
        }
    }
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
        return usage();
    const std::string cmd = argv[1];
    const CommandLine cl = CommandLine::fromMain(argc - 1, argv + 1);
    auto& reg = ImplantLibraryRegistry::instance();
    try {
        if (auto scan = cl.get("scan"))
            reg.scanDirectory(platform::pathFromUtf8(*scan));
        if (cmd == "list") {
            for (const auto& lib : reg.libraries()) {
                std::printf("%-24s %s%s  (%zu connections)\n", lib.id.c_str(), lib.name.c_str(), lib.builtIn ? " [built-in]" : "", lib.connections.size());
                printConnections(reg, lib);
            }
            return 0;
        }
        if (cmd == "export") {
            const auto id = cl.get("library");
            const auto out = cl.get("out");
            if (!id || !out)
                return usage();
            auto lib = reg.find(*id);
            if (!lib)
                throw std::runtime_error("No implant library '" + *id + "'");
            std::vector<std::shared_ptr<const Connection>> conns;
            for (const auto& c : lib->connections)
                conns.push_back(reg.connection(lib->id, c.id));
            LibraryInfo copy = *lib;
            copy.id = cl.get("id").value_or(lib->builtIn ? lib->id + "-copy" : lib->id);
            copy.name = cl.get("name").value_or(lib->name);
            writeImplantLibrary(platform::pathFromUtf8(*out), copy, conns);
            std::printf("Wrote implant library '%s' (%zu connections) to %s\n", copy.id.c_str(), conns.size(), out->c_str());
            return 0;
        }
        if (cmd == "info") {
            const auto in = cl.get("in");
            if (!in)
                return usage();
            const std::string id = reg.addFolder(platform::pathFromUtf8(*in));
            const auto lib = reg.find(id);
            std::printf("%s (%s), %s\n", lib->name.c_str(), lib->id.c_str(), lib->manufacturer.c_str());
            printConnections(reg, *lib);
            return 0;
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return usage();
}
