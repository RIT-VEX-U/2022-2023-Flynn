/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature RED_GOAL = vex::vision::signature (1, 8505, 9819, 9162, -1863, -1469, -1666, 6.5, 0);
vex::vision::signature BLUE_GOAL = vex::vision::signature (2, -3177, -2519, -2848, 6657, 8817, 7738, 4.8, 0);
vex::vision::signature SIG_3 = vex::vision::signature (3, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision cam = vex::vision (vex::PORT5, 60, RED_GOAL, BLUE_GOAL, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/