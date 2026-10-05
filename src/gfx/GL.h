#pragma once

// Single include point for OpenGL. OcclusaCAD targets OpenGL 3.3 core (macOS: 4.1 core).
#include <glad/gl.h>

namespace occlusa::gfx {

using GLLoadProc = void* (*)(const char* name);

// Load OpenGL entry points for the current context. Returns false on failure.
bool loadGL(GLLoadProc loader);

// Human readable renderer/version info for the about box and logs.
const char* glRendererString();
const char* glVersionString();

void checkGLError(const char* where);

} // namespace occlusa::gfx
