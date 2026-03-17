#pragma once
#include "globals.hpp"
#include "autonomous.hpp"
#include "tasks.hpp"
/**
 * Entry point for operator (driver) control
 * Executed in new VEX task (thread)
 * Runs after autonomous during a competition
*/
void opcontrol();
void handle_intake();
void handle_colorsort();
void handle_wings();
void handle_doinker();
void handle_medium_goal();
void handle_middle_toggle();
void handle_mcl_toggle();
void handle_intake_dejam();