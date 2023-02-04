#include "../include/autonomous_flynn.h"
#include "vision.h"

#define TURN_SPEED 0.6
#define INTAKE_VOLT 12
#define SHOOTING_RPM 3500
#define SINGLE_SHOT_TIME 0.2
#define SINGLE_SHOT_VOLT 12
#define SINGLE_SHOT_RECOVER_DELAY 1

#define DriveToPointFast(x, y) new DriveToPointCommand(drive_sys, drive_fast_mprofile, x, y, fwd, 1.0)
#define DriveForwardFast(dist, dir) new DriveForwardCommand(drive_sys, drive_fast_mprofile, dist, dir, 1.0)
#define TurnToHeading(heading_deg) new TurnToHeadingCommand(drive_sys, *config.turn_feedback, heading_deg, TURN_SPEED)
#define StartIntake new StartIntakeCommand(intake, INTAKE_VOLT)
#define StopIntake new StopIntakeCommand(intake)

#define VisionAim (new VisionAimCommand(cam, {RED_GOAL, BLUE_GOAL}, drive_sys))
#define WaitForFW (new WaitUntilUpToSpeedCommand(flywheel_sys, SHOOTING_RPM))
#define ShootDisk (new ShootCommand(intake, SINGLE_SHOT_TIME, SINGLE_SHOT_VOLT))

#define PrintOdom (new PrintOdomCommand(odometry_sys))
#define PrintOdomContinous (new PrintOdomContinousCommand(odometry_sys))


void add_single_shot_cmd(CommandController &controller, double vis_timeout=0.0)
{
    controller.add(WAIT_FOR_FLYWHEEL);
    if(vis_timeout == 0.0)
        controller.add(VisionAim);
    else
        controller.add(VisionAim, vis_timeout);
    controller.add(SHOOT_DISK);
    controller.add_delay(SINGLE_SHOT_RECOVER_DELAY);
}

void pleasant_opcontrol();

void test_stuff(){
  while(imu.isCalibrating()){
    vexDelay(20);
  }


  CommandController mine;
  mine.add(new PrintOdomCommand(odometry_sys));
  mine.add(PrintOdomContinous);
  mine.add(new PrintOdomCommand(odometry_sys));  mine.run();
  vex_printf("timedout %d\n", mine.last_command_timed_out());
  vex_printf("finshed\n");

  pleasant_opcontrol();
}


void pleasant_opcontrol(){
  vex_printf("opcontrollin");
  // Initialization
  double oneshot_time = .05;//Change 1 second to whatever is needed
  bool oneshotting = false;

  main_controller.ButtonUp.pressed([](){flywheel_sys.spinRPM(3500);});
  main_controller.ButtonDown.pressed([](){flywheel_sys.stop();});
  main_controller.ButtonR1.pressed([](){intake.spin(reverse, 12, volt);}); // Intake
  main_controller.ButtonR2.pressed([](){intake.spin(fwd, 12, volt);}); // Shoot
  main_controller.ButtonL2.pressed([](){intake.spin(fwd, 12, volt);oneshot_tmr.reset();}); //Single Shoot
  main_controller.ButtonL1.pressed([](){roller.spin(vex::reverse, 12, vex::volt);}); //Roller
  main_controller.ButtonL1.released([](){roller.stop();}); //Roller
  
  main_controller.ButtonB.pressed([](){odometry_sys.set_position();});

  odometry_sys.end_async();
  int i = 0;
  // Periodic
  while(true)
  {
    i++;
    if (i%5==0){
    main_controller.Screen.setCursor(0, 0);
    main_controller.Screen.clearScreen();
    main_controller.Screen.print("fw rpm: %f", flywheel_sys.getRPM());
    main_controller.Screen.setCursor(2, 0);
    main_controller.Screen.print("fw temp: %.1ff", flywheel.temperature(vex::fahrenheit));
    main_controller.Screen.setCursor(4, 0);
    main_controller.Screen.print("bat fw : %.2fv %.2fv", Brain.Battery.voltage(vex::volt), flywheel.voltage(volt));
    
  }

    // ========== DRIVING CONTROLS ==========
    drive_sys.drive_arcade(main_controller.Axis2.position(pct)/200.0, main_controller.Axis4.position(pct)/100.0);    
    // ========== MANIPULATING CONTROLS ==========


    if(main_controller.ButtonY.pressing() && main_controller.ButtonRight.pressing())
    {
      endgame_solenoid.set(true);
    }

    oneshotting = oneshot_tmr.time(vex::sec) < oneshot_time;
    if (!main_controller.ButtonR1.pressing() && !main_controller.ButtonR2.pressing() && !oneshotting)
    {
      intake.stop();
    }

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========    

    vexDelay(20);
  }
}


/*
Auto loader side

Map from page 40 of the game manual

(R) = Red Hoop, Blue Zone
(B) = Blue Hoop, Red Zone
 *  = Starting position for this auto

           ^ 180 degrees
  +-------------------+
  |        |____| (R) |
  |___           |    |
  | * |          |    |
  |   |          |    |  --> 90 degrees
  |   |          |_*__|
  |___|  ____         |
  |(B)  |____|        |
  +-------------------+
           v 0 degrees

 Human Instructions:
 Align robot to specified place and angle using LOADER SIDE AUTO jig
*/
CommandController auto_loader_side(){
    position_t start_pos = position_t{.x = 9.75, .y = 105.25, .rot = -90};

    CommandController lsa;
    lsa.add(new OdomSetPosition(odometry_sys, start_pos));
    lsa.add(new SpinRPMCommand(flywheel_sys, 3400));


    // spin -90 degree roller
    lsa.add(DriveForwardFast(1, fwd)); //[measure]
    lsa.add(new SpinRollerCommandAUTO(drive_sys, roller));
    lsa.add(DriveForwardFast(4, reverse)); // [measure]
    
    //shoot
    lsa.add(TurnToHeading(45)); // [measure]
    lsa.add(DriveToPointFast(47.4, 141.64));
    
    lsa.add(TurnToHeading(129)); // [measure]
 
    //lsa.add(new WaitUntilUpToSpeedCommand(flywheel_sys, 10));
    lsa.add(ShootDisk);
    lsa.add(StopIntake);

    return lsa;

}


/*
Skills loader side

Map from page 40 of the game manual

(R) = Red Hoop, Blue Zone
(B) = Blue Hoop, Red Zone
 *  = Starting position for this auto
    -x   ^ 180 degrees
  0-------------------+
  |        |____| (R) |
  |___           |    |
  | * |          |    |
  |   |          |    |  --> 90 degrees (+y)
  |   |          |_*__|
  |___|  ____         |
  |(B)  |____|        |
  +-------------------+ (140, 140)
           v 0 degrees
           (+x)

 Human Instructions:
 Align robot to specified place and angle using LOADER SIDE SKILLS jig
*/

CommandController prog_skills_loader_side(){
  
    position_t start_pos = position_t{.x = 9.75, .y = 34.75, .rot = -90};

    CommandController lss;
    lss.add(new OdomSetPosition(odometry_sys, start_pos));

    // Arrow 1 -------------------------
    // spin -90 degree roller
    lss.add(DriveForwardFast(1, fwd)); //[measure]
    lss.add(new SpinRollerCommandAUTO(drive_sys, roller));
    lss.add(DriveForwardFast(4, reverse)); // [measure]
    
    lss.add(TurnToHeading(180)); //[measure]
    
    // Arrow 2 -------------------------
    // intake corner disk
    lss.add(StartIntake);
    lss.add(DriveToPointFast(17.5, 17.5)); // [measure]
    lss.add(StopIntake);

    // align to 180 degree roller
    lss.add(TurnToHeading(45));  // [measure]
    lss.add(DriveToPointFast(24, 115.0)); //[measure] for sure
    lss.add(TurnToHeading(180));  // [measure]
    
    // spin 180 degree roller
    lss.add(DriveForwardFast(2, fwd)); //[measure]
    lss.add(new SpinRollerCommandAUTO(drive_sys, roller));
    lss.add(DriveForwardFast(2, reverse)); //[measure]

    //spin and shoot 3
    lss.add(TurnToHeading(80)); //[measure]
    lss.add(new SpinRPMCommand(flywheel_sys, 3500)); // [measure]
    lss.add(new WaitUntilUpToSpeedCommand(flywheel_sys, 10));
    lss.add(ShootDisk);

    // Arrow 3 -------------------------
    lss.add(TurnToHeading(45)); //[measure]
    lss.add(StartIntake);
    lss.add(DriveToPointFast(70, 50)); //[measure]
    lss.add(StopIntake);

    //face hoop and fire
    lss.add(TurnToHeading(135)); // [measure]
    lss.add(new SpinRPMCommand(flywheel_sys, 3000)); // [measure]
    lss.add(new WaitUntilUpToSpeedCommand(flywheel_sys, 10));
    lss.add(ShootDisk); // [measure]
    
    // Arrow 4 -------------------------
    lss.add(TurnToHeading(75)); // [measure]
    lss.add(DriveToPointFast(80, 132)); //[measure]

    // Move to endgame pos
    lss.add(TurnToHeading(10)); // [measure]
    lss.add(DriveToPointFast(122.5, 122.5)); //[measure]

    // Endgame
    lss.add(TurnToHeading(215)); //[measure]
    lss.add(new EndgameCommand(endgame_solenoid));

  return lss;
}