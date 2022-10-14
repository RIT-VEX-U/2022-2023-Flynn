#include "competition/opcontrol.h"
#include "../core/include/utils/math_util.h"
#include "robot-config.h"
#include <iostream>
/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */


void opcontrol()
{
  // Initialization  
  main_controller.Screen.clearScreen();
  // Periodic
  double time = 0.0;
  
  static double sp = 0.0;
  main_controller.ButtonDown.pressed([](){sp-=100;});
  main_controller.ButtonUp.pressed([](){sp+=100;});

  main_controller.ButtonLeft.pressed([](){sp=3000;});
  main_controller.ButtonRight.pressed([](){sp=0;});

  main_controller.ButtonL1.pressed([](){
    indexer.spin(directionType::rev, 100, percentUnits::pct);
  });
  main_controller.ButtonL1.released([](){
    indexer.stop();
  });


  main_controller.ButtonR1.pressed([](){
    intake.spin(directionType::rev, 10, percentUnits::pct);
  });

  shoot_limit.pressed([](){
      intake.stop();
    });
  
  while(true)
  {
    flywheel.spinRPM(sp);
        
    main_controller.Screen.setCursor(1, 0);
    main_controller.Screen.print("SP: %3.2f", flywheel.getDesiredRPM());

    main_controller.Screen.setCursor(2, 0);
    main_controller.Screen.print(" P: %3.2f", flywheel.getRPM());


    main_controller.Screen.setCursor(3, 0);
    main_controller.Screen.print("A : %3.2f", odom.get_position().rot);
          
    double sensitivity = 1.5*2;
    double l = (main_controller.Axis3.position() / 100.0);
    double r = (main_controller.Axis2.position() / 100.0);
    
    // make funner
    double adjuster = 1.6;

    double ls = sign(l), rs = sign(r);

    l = ls * pow(fabs(l), adjuster) * sensitivity;
    r = rs * pow(fabs(r), adjuster) * sensitivity;

    drive_sys.drive_tank(l, r);
    //printf("%f, %f\n", l,r);
    //drive_sys.drive_arcade(main_controller.Axis1., main_controller.Axis2);

    //double rpm = flywheel.getRPM();
    //double setrpm = flywheel.getDesiredRPM();

    //double err = setrpm - rpm;
    //double pid_contrib = flywheel.getPIDValue() * 12;
    //double feed_forward_contrib = flywheel.getFeedforwardValue() * 12;
    //double output = pid_contrib + feed_forward_contrib;
    //printf("%f \t%f \t %f \t %f \t %f \t %f \t %f\n", time, rpm, setrpm, err, pid_contrib, feed_forward_contrib, output);



    // ========== DRIVING CONTROLS ==========

    // ========== MANIPULATING CONTROLS ==========

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========    

    // Wait 20 milliseconds for control loops to calculate time correctly
    vexDelay(20);
    time+=(40.0)/(1000.0);
  }
}