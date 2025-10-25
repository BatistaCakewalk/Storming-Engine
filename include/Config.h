#ifndef CONFIG_H
#define CONFIG_H

#pragma once
#include "Vector2D.h"
#include <string>

// ----------------- Engine Settings -----------------
inline bool unlockFPS = false;             // Toggle FPS unlock: true = uncapped, false = limited to targetFPS
inline bool showFPS = true;                // Show FPS counter, u want it? It's yours my friend, as long as u have enough trues.
inline int targetFPS = 75;                 // Custom FPS limit when unlockFPS is false

// ----------------- Dragging Settings -----------------
inline bool allowDragging = true;          // Enable/disable dragging
inline Vector2D OffsetDragging = Vector2D(0, 0); // Default offset applied during drag
inline float StiffnessDragging = 1000.0f; // Spring stiffness when dragging (higher = snappier)
inline float DampingDragging = 25.0f;     // Drag damping (higher = less oscillation)

// ----------------- Application Settings -----------------
inline std::string windowName = "Storming Engine | BETA BRANCH"; // Window title
inline int windowWidthSize = 800;   // Window width
inline int windowHeightSize = 600;  // Window height

#endif

