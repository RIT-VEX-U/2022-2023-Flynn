#include "competition/opcontrol.h"
#include "C:/Users/richi/VEX/2022-2023-Flynn/core/include/subsystems/odometry/odometry_base.h"
#include "robot-config.h"
/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */
void opcontrol()
{
  // Initialization

  //Feed Forward Auto Tuning
  //double pct = .9;
  //double duration = 1;
  //
  //printf("pct: %f - dur: %f\n", pct, duration);
  ////printf("starting tune\n");fflush(stdout);
  //auto mc = new MotionController(mprof_drive_normal_cfg);
  //auto ff_config = mc->tune_feedforward(drive_sys, odom, pct, duration);
  //printf("ff_config:\n\tkS: %f\n\tkV: %f\n\tkA: %f\n", ff_config.kS, ff_config.kV, ff_config.kA);
  //printf("finished tune\n");fflush(stdout);


  double time = 0;

  if (1){
  while (!drive_sys.drive_forward(1, fwd) && time<4){
    time+=.02;  
    
    vexDelay(20);
  }
  }
  
  // Periodic
  while(true)
  {
    drive_sys.drive_tank(main_controller.Axis2.value()*.015,main_controller.Axis3.value()*.015);
    auto pos = odom.get_position();
    printf("(%f, %f)\n", pos.x, pos.y);fflush(stdout);
     // ========== DRIVING CONTROLS ==========

    // ========== MANIPULATING CONTROLS ==========

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========    
    time+=.02;

    // Wait 20 milliseconds for control loops to calculate time correctly
    vexDelay(20);
  }
}