#pragma once
#include "globals.hpp"

// Optional background telemetry task.
// Remove it from initialize() if you do not want screen output.
void screen_print_task(void* args);

// Optional subsystem task scaffold.
// Add non-drive subsystem logic here if you prefer task-based control.
void subsystem_template_task(void* args);