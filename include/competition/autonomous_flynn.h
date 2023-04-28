#include "../core/include/utils/command_structure/auto_command.h"
#include "../core/include/utils/command_structure/command_controller.h"
#include "../core/include/utils/command_structure/drive_commands.h"
#include "../core/include/utils/command_structure/flywheel_commands.h"
#include "../core/include/utils/graph_drawer.h"
#include "automation.h"
#include "competition/autonomous.h"
#include "robot-config.h"
#include <vector>
#include <float.h>


void test_stuff();
CommandController prog_skills_loader_side();
CommandController auto_loader_side();
CommandController auto_loader_side_disks_last();
CommandController skills_rollers_last();
CommandController disk_rush_auto();
CommandController only_roller_auto();

void draw_image();