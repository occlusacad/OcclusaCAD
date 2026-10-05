#pragma once

#include "core/Mesh.h"
#include "core/crown/ToothLibrary.h"

#include <string_view>

namespace occlusa::crown {

// Procedural tooth anatomy for the built-in libraries (generated, so they carry no third-party
// license). Each crown is described by its outline, heights of contour, occlusal table and
// relief features (cusps, triangular / marginal ridges, grooves, pits, lingual fossa) and is
// turned into a closed mesh in the tooth frame:
//   x = mesial, y = buccal / labial, z = occlusal; z = 0 at the cervical line, origin on the axis.

enum class LibraryStyle { Natural, Young, Mature };

std::string_view toString(LibraryStyle style);

Mesh generateToothCrown(ToothKind kind, bool upper, LibraryStyle style, int azimuthSamples = 160);

} // namespace occlusa::crown
