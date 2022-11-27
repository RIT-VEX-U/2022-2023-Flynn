#include "competition/opcontrol.h"
#include "C:/Users/richi/VEX/2022-2023-Flynn/core/include/subsystems/odometry/odometry_base.h"
#include "robot-config.h"


//output: kS 0.072000, kV 0.000103, kA -0.000003
//output: kS 0.044000, kV 0.000106, kA -0.000003
//output: kS 0.005000, kV 0.000111, kA -0.000003
//output: kS 0.052000, kV 0.000105, kA -0.000003
//output: kS 0.002500, kV 0.000111, kA -0.000010
//output: kS 0.076500, kV 0.000051, kA -0.000005


/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */
void opcontrol()
{
  // Initialization
  double time = 0;
  imu.calibrate();
  while(imu.isCalibrating()){
    vexDelay(20);
  }
  printf("Finished Calibrating✅\n");fflush(stdout);
  

  if (1){
    while (main_controller.ButtonA.pressing() && !drive_sys.turn_degrees(-90)){      //!drive_sys.turn_to_heading(0)){
      time+=.02;
      vexDelay(20);
    }

    while (main_controller.ButtonA.pressing() && !drive_sys.drive_to_point(12, 0, fwd)){      //!drive_sys.turn_to_heading(0)){
      time+=.02;
      vexDelay(20);
    }
  }
  
  // Periodic
  while(true)
  {
    drive_sys.drive_tank(main_controller.Axis3.value()*1,main_controller.Axis2.value()*1);
    auto pos = odom.get_position();
    if (main_controller.ButtonB.pressing()){
      printf("(%f, %f) %f\n",pos.x, pos.y, pos.rot);fflush(stdout);
    }
    // ========== DRIVING CONTROLS ==========

    // ========== MANIPULATING CONTROLS ==========

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========    
    time+=.02;

    // Wait 20 milliseconds for control loops to calculate time correctly
    vexDelay(20);
  }
}