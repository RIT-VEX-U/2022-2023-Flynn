#include "competition/autonomous_flynn.h"
#include "vision.h"
#include "tuning.h"

#define TURN_SPEED 0.6
#define INTAKE_VOLT 12
#define SHOOTING_RPM 3200
#define SINGLE_SHOT_TIME 0.085
#define SINGLE_SHOT_VOLT 12
#define SINGLE_SHOT_RECOVER_DELAY_MS 1000

#define DriveToPointSlow(x, y) new DriveToPointCommand(drive_sys, drive_slow_mprofile, x, y, fwd, 1.0)
#define DriveToPointSlowPt(pt) new DriveToPointCommand(drive_sys, drive_slow_mprofile, pt, fwd, 1.0)

#define DriveToPointFast(x, y) new DriveToPointCommand(drive_sys, drive_fast_mprofile, x, y, fwd, 1.0)
#define DriveToPointFastPt(pt) new DriveToPointCommand(drive_sys, drive_fast_mprofile, pt, fwd, 1.0)
#define DriveForwardFast(dist, dir) new DriveForwardCommand(drive_sys, drive_fast_mprofile, dist, dir, 1.0)
#define TurnToHeading(heading_deg) new TurnToHeadingCommand(drive_sys, *config.turn_feedback, heading_deg, TURN_SPEED)

#define TurnToPoint(point) new TurnToPointCommand(drive_sys, odometry_sys, *config.turn_feedback, point)

#define StartIntake new StartIntakeCommand(intake, INTAKE_VOLT)
#define StopIntake new StopIntakeCommand(intake)

#define VisionAim (new VisionAimCommand())
#define WaitForFW (new WaitUntilUpToSpeedCommand(flywheel_sys, 10))
#define ShootDisk (new ShootCommand(intake, SINGLE_SHOT_TIME, SINGLE_SHOT_VOLT))
#define SpinFWAt(rpm) (new SpinRPMCommand(flywheel_sys, rpm))

#define PrintOdom (new PrintOdomCommand(odometry_sys))
#define PrintOdomContinous (new PrintOdomContinousCommand(odometry_sys))


static void add_single_shot_cmd(CommandController &controller, double vis_timeout=0.0)
{
    controller.add(WaitForFW);
    if(vis_timeout == 0.0)
        controller.add(VisionAim);
    else
        controller.add(VisionAim, vis_timeout);
    controller.add(ShootDisk);
    controller.add_delay(1000);
}

void pleasant_opcontrol();

void test_stuff(){
    while(imu.isCalibrating()){
    vexDelay(20);
  }



  pleasant_opcontrol();


  CommandController mine = prog_skills_loader_side();
  // mine.run();
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

  VisionAimCommand visaim;

  odometry_sys.end_async();
  int i = 0;
  
  timer loop_timer;
  loop_timer.reset();
  double delay_time = 0.0;
  double loop_time=0.0;
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
    printf("loop time: %fs\n", loop_time);    
  }

    // ========== DRIVING CONTROLS ==========
    if(main_controller.ButtonA.pressing())
      visaim.run();
    else
      drive_sys.drive_arcade(main_controller.Axis3.position(pct)/100.0, main_controller.Axis1.position(pct)/300.0);    

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


    // ======== Real timing ========
    loop_time = loop_timer.time(vex::timeUnits::sec);
    delay_time = max(0.02 - loop_time, 0.0);
    vexDelay((int)(delay_time*1000));
    loop_timer.reset();

  }
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
  
    position_t start_pos = position_t{.x = 30.5, .y = 10.2, .rot = -90};

    CommandController lss;
    lss.add(new OdomSetPosition(odometry_sys, start_pos)); // #1

    // Arrow 1 -------------------------
    // spin -90 degree roller
    Vector2D::point_t corner_disk_point = {.x = 8, .y = 12};
    lss.add(DriveForwardFast(1, fwd)); // #2
    lss.add(new SpinRollerCommandAUTO(drive_sys, roller)); // #3
    lss.add(DriveForwardFast(4, reverse)); // #4
    
    lss.add(TurnToPoint(corner_disk_point), 1.5); // #5
    
    // Arrow 2 -------------------------
    // intake corner disk
    
    lss.add(StartIntake); // #6
    lss.add(DriveToPointSlowPt(corner_disk_point)); // #7
    lss.add(DriveForwardFast(4, reverse)); // #8
    lss.add_delay(1000);
    lss.add(StopIntake); // #9



    // align to 180 degree roller
    lss.add(TurnToHeading(90), 1.5);  // #10
    lss.add(DriveToPointFast(12, 31.5)); // #11
    lss.add(TurnToHeading(180), 1.5);  // #12

    
    // spin 180 degree roller
    
      lss.add(DriveForwardFast(2, fwd)); // #13
      lss.add(new SpinRollerCommandAUTO(drive_sys, roller)); // #14
      lss.add(DriveForwardFast(2, reverse)); // #15
    


    //spin and shoot 3
    Vector2D::point_t shoot_point = {.x = 12, .y = 78};
    lss.add(TurnToPoint(shoot_point), 1.5); // #16
    lss.add(DriveToPointFastPt(shoot_point), 4.0); // #17
    
    lss.add(TurnToHeading(85), 0.5); // #18
    
    lss.add(new SpinRPMCommand(flywheel_sys, 3100)); // #19

    repeat(3) 
      add_single_shot_cmd(lss); //21, 22, 23

    lss.add_delay(1000);
    //lss.add(PrintOdomContinous); /// the guy youre looking for =================================================================================> :)


    // Arrow 3 -------------------------
    
    Vector2D::point_t start_of_line = {.x = 36, .y = 58};
    Vector2D::point_t end_of_line = {.x = 60, .y = 84};
    
      lss.add(TurnToPoint(start_of_line));
      lss.add(StartIntake);
      lss.add(DriveToPointSlowPt(start_of_line));
      lss.add(DriveForwardFast(4, reverse));

      lss.add(TurnToPoint(end_of_line));      
      lss.add(DriveToPointSlowPt(end_of_line));

      lss.add(StopIntake);
    
    //face hoop and fire
    lss.add(TurnToHeading(135)); // [measure]
    lss.add(new SpinRPMCommand(flywheel_sys, 3100)); // [measure]

    add_single_shot_cmd(lss);
    add_single_shot_cmd(lss);
    add_single_shot_cmd(lss);

    // Arrow 4 -------------------------
    Vector2D::point_t out_of_way_point = {.x = 70, .y = 124};
    lss.add(TurnToPoint(out_of_way_point)); // [measure]
    lss.add(DriveToPointFastPt(out_of_way_point)); //[measure]

    // Move to endgame pos
    Vector2D::point_t endgame_point = {.x = 122, .y = 122};
    lss.add(TurnToPoint(endgame_point));
    lss.add(DriveToPointFastPt(endgame_point)); //[measure]

    // Endgame
    lss.add(TurnToHeading(45)); //[measure]
    lss.add(new EndgameCommand(endgame_solenoid));
    lss.add(PrintOdom);

  return lss;
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