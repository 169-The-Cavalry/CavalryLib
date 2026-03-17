#pragma once
#include "globals.hpp"

// Competition autonomous entry point.
void autonomous();

// Selects and runs the currently chosen autonomous routine.
void run_autonomous();

// Template autonomous routines:
// 1) Keep AUTO_DO_NOTHING as a safe fallback.
// 2) Replace AUTO_TEST_DRIVE with your actual match autonomous.
// 3) Use AUTO_SKILLS_TEMPLATE as your skills run starting point.
void auto_do_nothing();
void auto_test_drive();
void auto_skills_template();