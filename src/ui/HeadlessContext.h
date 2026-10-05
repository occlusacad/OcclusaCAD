#pragma once

namespace occlusa::ui::headless {

// Off-screen OpenGL 3.3 core context via EGL (Mesa surfaceless). Kept in its own translation
// unit because EGL and glad headers do not mix. Returns false where unsupported.
bool createContext();
void* getProcAddress(const char* name);
void destroyContext();

} // namespace occlusa::ui::headless
