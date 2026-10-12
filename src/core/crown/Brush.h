#pragma once

#include "core/crown/CrownBuilder.h"

#include <filesystem>
#include <string>
#include <vector>

namespace occlusa::crown {

// Free-forming tools (the "wax knife"). A tool is a radial profile: how much material it adds or
// removes at a distance r from its centre (r = 0 .. 1 of the brush radius).
//
//   Ball     round ball: a hemisphere, soft everywhere
//   Knife    pointed knife: concentrated at the tip, for grooves and fissures
//   Texture  stipples the surface (a noise pattern under a ball), for surface texture
//   Custom   a 2D cross-section drawn by the technician plus a falloff towards the edge
//
// Custom tools are saved as small JSON files ("occlusacad.brush") that can be exported and
// imported between installations.
enum class BrushKind { Ball, Knife, Texture, Custom };

struct BrushTool {
    std::string name = "Round ball";
    BrushKind kind = BrushKind::Ball;
    // Custom: cross-section points (r, height), r in [0, 1] from the centre to the edge, height in
    // [0, 1]; sorted by r, linear in between.
    std::vector<glm::dvec2> profile{{0.0, 1.0}, {0.5, 0.75}, {1.0, 0.0}};
    double falloffStart = 0.6; // fraction of the radius where the edge falloff begins
    double falloffPower = 1.0; // shape of the falloff: > 1 harder edge, < 1 softer
    // Texture
    double textureScale = 0.35; // feature size, mm
    double textureContrast = 1.0;

    // Material factor at normalised distance r (0..1) for a surface point p (texture uses p).
    double weight(double r, const glm::vec3& p) const;
};

std::vector<BrushTool> builtinBrushTools(); // round ball, pointed knife, texture
std::string brushToolToJson(const BrushTool& tool);
BrushTool brushToolFromJson(const std::string& json); // throws on an invalid file
void saveBrushTool(const std::filesystem::path& file, const BrushTool& tool);
BrushTool loadBrushTool(const std::filesystem::path& file);
// Every *.json brush in a folder (invalid files are skipped).
std::vector<BrushTool> loadBrushTools(const std::filesystem::path& folder);

// Apply a tool at `center` (one dab; call repeatedly while the mouse is held). Add/Remove move
// the surface along its normal by up to `strength` mm, Smooth relaxes it, Flatten pulls it onto
// the plane of the brushed area.
void applyBrushTool(const CrownMesh& crown, std::vector<float>& displacement, const glm::vec3& center, float radius, float strength, BrushMode mode,
                    const BrushTool& tool);

} // namespace occlusa::crown
