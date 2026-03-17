#pragma once
#include "globals.hpp"
#include "tasks.hpp"

/**
 * Entry point for autonomous code
 * Executed in new VEX task (thread)
 * Runs before opcontrol (during competition)
*/
void autonomous();
void skills();

void awp_left_blue();
void awp_right_blue();
void awp_left_red();
void awp_right_red();

void elim_left_blue();
void elim_right_blue();
void elim_left_red();
void elim_right_red();

void fastpush_right();
void fastpush_left();

void select_auto();

void shimmy_shake();
void shimmy_shake_fast(float endTheta);