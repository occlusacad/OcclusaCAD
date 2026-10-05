#pragma once

#include <imgui.h>

#include <string>
#include <vector>

namespace occlusa::ui {

// Accent-filled call to action.
bool primaryButton(const char* label, const ImVec2& size = ImVec2(0, 0), bool enabled = true);
bool dangerButton(const char* label, const ImVec2& size = ImVec2(0, 0));
// Regular button that can be disabled.
bool button(const char* label, const ImVec2& size = ImVec2(0, 0), bool enabled = true);

void heading(const char* text);
void subheading(const char* text);
void mutedText(const char* fmt, ...) IM_FMTARGS(1);
void wrappedMutedText(const char* text);
void helpMarker(const char* text);

// Coloured rounded label, e.g. case status.
void pill(const char* text, const ImVec4& color);

// Horizontal segmented control; returns true when the selection changed.
bool segmented(const char* id, const std::vector<const char*>& labels, int& current, float itemWidth = 0.0f);

// Card: a bordered child region with padding.
bool beginCard(const char* id, const ImVec2& size = ImVec2(0, 0), bool autoHeight = true);
void endCard();

// Small "keyboard hint" chip.
void keyHint(const char* text);

// Two-column form layout: beginForm()/formField()/endForm(). Each field is a label above a full-width input.
bool beginForm(const char* id, int columns = 2);
void endForm();
bool formField(const char* label, std::string& value, const char* hint = nullptr);
// Moves to the next form cell and draws the label; caller draws a widget with width -1.
void formLabel(const char* label);

// Text input with a visible label above it (form style). Returns true when edited.
bool labeledInput(const char* label, std::string& value, const char* hint = nullptr, float width = -1.0f);

// Simple modal dialogs managed globally (rendered by drawModals()).
void showError(const std::string& title, const std::string& message);
void showInfo(const std::string& title, const std::string& message);
void drawModals();

// Toast notifications (bottom right).
enum class ToastKind { Info, Success, Warning, Error };
void toast(ToastKind kind, const std::string& message);
void drawToasts();

} // namespace occlusa::ui
