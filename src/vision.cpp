/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature RED_GOAL (1, 7297, 8801, 8049, -1051, -301, -676, 6.500, 0);
vex::vision::signature BLUE_GOAL (2, -1927, -1177, -1552, 6855, 8809, 7832, 4.500, 0);
vex::vision::signature SIG_3 = vex::vision::signature (3, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision cam = vex::vision (vex::PORT5, 73, RED_GOAL, BLUE_GOAL, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/