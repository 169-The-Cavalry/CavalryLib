#pragma once
#include "globals.hpp"
#include "autonomous.hpp"
#include "tasks.hpp"

// Competition driver control entry point.
void opcontrol();

// Helper handlers used by opcontrol().
void handle_drive_controls();
void handle_intake_controls();
void handle_pneumatic_controls();
void handle_auto_selector_controls();