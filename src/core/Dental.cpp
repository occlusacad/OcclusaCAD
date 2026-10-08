#include "core/Dental.h"

#include <format>

namespace occlusa::dental {

bool isValidFdi(int fdi)
{
    const int q = fdi / 10, p = fdi % 10;
    return q >= 1 && q <= 4 && p >= 1 && p <= 8;
}

int quadrantOf(int fdi) { return fdi / 10; }
int positionInQuadrant(int fdi) { return fdi % 10; }
bool isUpper(int fdi) { return quadrantOf(fdi) == 1 || quadrantOf(fdi) == 2; }

int fdiToUniversal(int fdi)
{
    const int q = quadrantOf(fdi), p = positionInQuadrant(fdi);
    switch (q) {
    case 1: return 9 - p;   // 18 -> 1 ... 11 -> 8
    case 2: return 8 + p;   // 21 -> 9 ... 28 -> 16
    case 3: return 25 - p;  // 38 -> 17 ... 31 -> 24
    case 4: return 24 + p;  // 41 -> 25 ... 48 -> 32
    default: return 0;
    }
}

int universalToFdi(int u)
{
    if (u >= 1 && u <= 8)
        return 19 - u;  // 1 -> 18 ... 8 -> 11
    if (u >= 9 && u <= 16)
        return 12 + u;  // 9 -> 21 ... 16 -> 28
    if (u >= 17 && u <= 24)
        return 55 - u;  // 17 -> 38 ... 24 -> 31
    if (u >= 25 && u <= 32)
        return 16 + u;  // 25 -> 41 ... 32 -> 48
    return 0;
}

Numbering numberingFromString(std::string_view s)
{
    return s == "universal" ? Numbering::Universal : Numbering::FDI;
}

int parseTooth(int number, Numbering numbering)
{
    if (numbering == Numbering::Universal)
        return universalToFdi(number);
    return isValidFdi(number) ? number : 0;
}

std::string toothList(const std::vector<int>& fdi, Numbering numbering, std::string_view separator)
{
    std::string out;
    for (int t : fdi) {
        if (!out.empty())
            out += separator;
        out += toothLabel(t, numbering);
    }
    return out;
}

std::string formatToothList(std::string_view fdiList, Numbering numbering)
{
    if (numbering == Numbering::FDI)
        return std::string(fdiList);
    std::vector<int> teeth;
    int value = 0;
    bool any = false;
    for (char c : fdiList) {
        if (c >= '0' && c <= '9') {
            value = value * 10 + (c - '0');
            any = true;
        } else if (any) {
            teeth.push_back(value);
            value = 0;
            any = false;
        }
    }
    if (any)
        teeth.push_back(value);
    return toothList(teeth, numbering);
}

std::string toothLabel(int fdi, Numbering numbering)
{
    if (numbering == Numbering::Universal)
        return std::to_string(fdiToUniversal(fdi));
    return std::to_string(fdi);
}

std::string toothName(int fdi)
{
    static constexpr const char* kNames[] = {"", "central incisor", "lateral incisor", "canine", "first premolar", "second premolar",
                                             "first molar", "second molar", "third molar"};
    const int q = quadrantOf(fdi);
    const char* jaw = (q == 1 || q == 2) ? "Upper" : "Lower";
    const char* side = (q == 1 || q == 4) ? "right" : "left";
    return std::format("{} {} {}", jaw, side, kNames[positionInQuadrant(fdi)]);
}

const std::vector<int>& upperArch()
{
    static const std::vector<int> v = {18, 17, 16, 15, 14, 13, 12, 11, 21, 22, 23, 24, 25, 26, 27, 28};
    return v;
}

const std::vector<int>& lowerArch()
{
    static const std::vector<int> v = {48, 47, 46, 45, 44, 43, 42, 41, 31, 32, 33, 34, 35, 36, 37, 38};
    return v;
}

const std::vector<RestorationType>& restorationTypes()
{
    static const std::vector<RestorationType> types = {
        // Implant planning (OcclusaCAD: Implant Studio workflow)
        {"implant_planning", "Implant planning", "Implant planning", "implant_planning", {0.20f, 0.62f, 0.86f}},
        {"surgical_guide", "Implant + surgical guide", "Implant planning", "surgical_guide", {0.13f, 0.47f, 0.80f}},
        // Implant restorations (future workflows)
        {"custom_abutment", "Custom abutment", "Implant restoration", "custom_abutment", {0.55f, 0.40f, 0.85f}},
        {"screw_retained_crown", "Screw-retained crown", "Implant restoration", "custom_abutment", {0.68f, 0.45f, 0.88f}},
        // Crown & bridge (future workflows)
        {"anatomic_crown", "Anatomic crown", "Crown & bridge", "crown_bridge", {0.95f, 0.70f, 0.20f}},
        {"coping", "Coping", "Crown & bridge", "crown_bridge", {0.90f, 0.55f, 0.25f}},
        {"pontic", "Pontic", "Crown & bridge", "crown_bridge", {0.85f, 0.40f, 0.30f}},
        {"inlay_onlay", "Inlay / onlay", "Crown & bridge", "crown_bridge", {0.40f, 0.75f, 0.45f}},
        {"veneer", "Veneer", "Crown & bridge", "crown_bridge", {0.55f, 0.80f, 0.60f}},
    };
    return types;
}

const RestorationType* findRestorationType(std::string_view key)
{
    for (const auto& t : restorationTypes())
        if (key == t.key)
            return &t;
    return nullptr;
}

std::vector<std::string> restorationCategories()
{
    std::vector<std::string> out;
    for (const auto& t : restorationTypes())
        if (out.empty() || out.back() != t.category)
            out.emplace_back(t.category);
    return out;
}

} // namespace occlusa::dental
