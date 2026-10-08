#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace occlusa::dental {

// ---------------------------------------------------------------------------
// Teeth (FDI two-digit notation is used internally everywhere)
// ---------------------------------------------------------------------------

enum class Numbering { FDI, Universal };

bool isValidFdi(int fdi);
int quadrantOf(int fdi);      // 1..4 (permanent dentition)
int positionInQuadrant(int fdi); // 1 (central incisor) .. 8 (third molar)
bool isUpper(int fdi);
int fdiToUniversal(int fdi);  // 1..32
int universalToFdi(int universal); // 0 if out of range
Numbering numberingFromString(std::string_view s); // "universal" -> Universal, anything else FDI
// Tooth number as entered in the given system (FDI or Universal), as FDI; 0 if invalid.
int parseTooth(int number, Numbering numbering);
std::string toothLabel(int fdi, Numbering numbering);
std::string toothName(int fdi); // e.g. "Upper right first molar"
// "36, 37" style lists (FDI in, labels out).
std::string toothList(const std::vector<int>& fdi, Numbering numbering, std::string_view separator = ", ");
std::string formatToothList(std::string_view fdiList, Numbering numbering); // re-labels a comma separated FDI list

// All 32 permanent teeth in chart order: upper right->upper left, lower left->lower right.
const std::vector<int>& upperArch();
const std::vector<int>& lowerArch();

// ---------------------------------------------------------------------------
// Restoration types (what the lab is asked to make for a tooth)
// ---------------------------------------------------------------------------

struct RestorationType {
    const char* key;      // stable identifier stored in the database
    const char* label;
    const char* category;
    const char* workflow; // workflow key used to design this restoration
    float color[3];       // chart colour (linear-ish sRGB, 0..1)
};

const std::vector<RestorationType>& restorationTypes();
const RestorationType* findRestorationType(std::string_view key);

std::vector<std::string> restorationCategories();

} // namespace occlusa::dental
