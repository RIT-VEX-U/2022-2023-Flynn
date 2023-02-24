#pragma once

#include "../core/include/subsystems/screen.h"
#include "robot-config.h"
#include "splash_image_small.h"
#include "../core/include/utils/graph_drawer.h"

void page_one(vex::brain::lcd &screen, int x, int y, int width, int height, bool first_run);

void page_two(vex::brain::lcd &screen, int x, int y, int width, int height, bool first_run);

void page_three(vex::brain::lcd &screen, int x, int y, int width, int height, bool first_run);