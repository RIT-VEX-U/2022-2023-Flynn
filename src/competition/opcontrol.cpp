#include "competition/opcontrol.h"
#include "robot-config.h"
#include "tuning.h"
void print_flywheel_affectors(){
  printf("bat votage: %f(v)\n", Brain.Battery.voltage());
  printf("bat temp: %f(F)\n", Brain.Battery.temperature(fahrenheit));
  printf("mot temp: %f(F)\n", flywheel.temperature(fahrenheit));
  printf("mot voltage: %f(v)\n", flywheel.voltage());
}
/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */
void opcontrol()
{
  print_flywheel_affectors();
  return;
  while(1){
    tune_flywheel_distcalc();
    vexDelay(20);
  }
   
  return;
  /*
  double t = 0;
  printf("time setpt rpm out");
  while (1){
    if (main_controller.ButtonA.pressing()){
      intake.spin(fwd, 12, volt);
    } else {
      intake.spin(fwd, 0, volt);
    }

    if (t < 6){
      flywheel_sys.spinRPM(1000);
    } else if (t < 12){
      flywheel_sys.spinRPM(2000);
    } else if (t < 18){
      flywheel_sys.spinRPM(3000);
    } else {
      flywheel_sys.spinRPM(3500);
    }
    printf("%f %f %f %f\n", t, flywheel_sys.getDesiredRPM(), flywheel_sys.getRPM(), flywheel_sys.getFeedforwardValue() + flywheel_sys.getPIDValue());
    t += 0.02;
    vexDelay(20);
  }
  return;
  */
  // Initialization
  /*
  printf("starting\n");
  fflush(stdout);
  double oneshot_time = .05;//Change 1 second to whatever is needed
  bool oneshotting = false;
  
  flywheel_sys.spin_manual(3000);//TODO measure speed that is needed
  
  if (1){
    main_controller.ButtonR1.pressed([](){intake.spin(reverse, 12, volt);}); // Intake
    main_controller.ButtonR2.pressed([](){intake.spin(fwd, 12, volt);}); // Shoot
    main_controller.ButtonL2.pressed([](){intake.spin(fwd, 12, volt);oneshot_tmr.reset();}); //Single Shoot
    main_controller.ButtonL1.pressed([](){roller.spin(fwd);}); //Roller
    main_controller.ButtonB.pressed([](){odometry_sys.set_position();});

    main_controller.ButtonUp.pressed([](){flywheel_sys.spinRPM(flywheel_sys.getDesiredRPM() + 500);});
    main_controller.ButtonDown.pressed([](){flywheel_sys.spinRPM(flywheel_sys.getDesiredRPM() - 500);});

    main_controller.ButtonLeft.pressed([](){flywheel_sys.spinRPM(3000);});
    main_controller.ButtonRight.pressed([](){flywheel_sys.stop();});
  }
  // Periodic
  while(true)
  {

    //tune_odometry_wheel_diam();
    //tune_odometry_wheelbase();
    
    auto pos = odometry_sys.get_position();
    main_controller.Screen.clearScreen();
    main_controller.Screen.setCursor(0, 0);
    main_controller.Screen.print("(%.3f, %.3f) : %.3f", pos.x, pos.y, pos.rot);
    
    // ========== DRIVING CONTROLS ==========
    // drive_sys.drive_tank(main_controller.Axis3.position()/100.0,main_controller.Axis2.position() / 100.0);
    drive_sys.drive_arcade(main_controller.Axis3.position()/100.0, main_controller.Axis1.position()/100.0);
    
    // ========== MANIPULATING CONTROLS ==========


    if(main_controller.ButtonY.pressing() && main_controller.ButtonRight.pressing()){
      //TODO eject string
    }

    oneshotting = oneshot_tmr.time(vex::sec) < oneshot_time;
    if (!main_controller.ButtonR1.pressing() && !main_controller.ButtonR2.pressing() && !oneshotting){
      intake.stop();
    }

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========    

    vexDelay(20);
  }
  */ 
}