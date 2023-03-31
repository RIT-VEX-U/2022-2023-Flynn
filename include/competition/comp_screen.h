#pragma once

#include "../core/include/subsystems/screen.h"
#include "robot-config.h"
#include "splash_image_small.h"
#include "../core/include/utils/graph_drawer.h"
#include "../core/include/utils/vector2d.h"
#include <array>
extern bool keep_collecting;

void page_one(vex::brain::lcd &screen, int x, int y, int width, int height, bool first_run);

void page_two(vex::brain::lcd &screen, int x, int y, int width, int height, bool first_run);

void page_three(vex::brain::lcd &screen, int x, int y, int width, int height, bool first_run);

void page_four(vex::brain::lcd &screen, int x, int y, int width, int height, bool first_run);

void page_five(vex::brain::lcd &screen, int x, int y, int width, int height, bool first_run);

void page_six(vex::brain::lcd &screen, int x, int y, int width, int height, bool first_run);
