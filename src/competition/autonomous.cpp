#include "../include/competition/autonomous.h"

#define TURN_SPEED 0.6


//functions that define autos. construct a CommandController when called.
CommandController auto_loader_side();
CommandController prog_skills_loader_side();



int print_odom(){
    while(true){
        position_t pos = odometry_sys.get_position();
        printf("%.2f, %.2f, %.2f\n", pos.x, pos.y, pos.rot);
        vexDelay(20);
    }
    return 0;
}


/**
 * Contains all the code run during autonomous.
 */ 
void autonomous()
{

    while(imu.isCalibrating()){
      vexDelay(20);
    }
    vex::task printodom(print_odom);
    CommandController current_auto = prog_skills_loader_side();
    current_auto.run();
    while(true){
        drive_sys.stop();
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
    int loader_side_full_shot_rpm = 3000;  // [measure]

    position_t start_pos = position_t{.x = 9.75, .y = 105.25, .rot = -90};

    CommandController lsa;
    lsa.add(new OdomSetPosition(odometry_sys, start_pos));
    lsa.add(new StartIntakeCommand(flywheel, -12));
    //lsa.add(new SpinRPMCommand(flywheel_sys, 3400));


    // spin -90 degree roller
    lsa.add(new DriveForwardCommand(drive_sys, drive_slow_mprofile, 1, fwd)); //[measure]
    lsa.add(new SpinRollerCommandAUTO(drive_sys, roller));
    lsa.add(new DriveForwardCommand(drive_sys, drive_slow_mprofile, 4, reverse)); // [measure]
    
    //shoot
    lsa.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 45, .6)); // [measure]
    lsa.add(new DriveToPointCommand(drive_sys, drive_fast_mprofile, 47.4, 141.64, fwd, 1));
    
    lsa.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 131-10+8, .6)); // [measure]
 
    //lsa.add(new WaitUntilUpToSpeedCommand(flywheel_sys, 10));
    lsa.add(new ShootCommand(intake, 3, 3));
    lsa.add(new StopIntakeCommand(flywheel));

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
    int loader_side_full_shot_rpm = 3000;  // [measure]
    position_t start_pos = position_t{.x = 9.75, .y = 105.25, .rot = -90};
  

    CommandController lss;

    lss.add(new OdomSetPosition(odometry_sys, start_pos));
    lss.add(new StartIntakeCommand(flywheel, -12));
    //lsa.add(new SpinRPMCommand(flywheel_sys, 3400));


    // spin -90 degree roller
    lss.add(new DriveForwardCommand(drive_sys, drive_slow_mprofile, 1, fwd)); //[measure]
    lss.add(new SpinRollerCommandSKILLS(drive_sys, roller));
    lss.add(new DriveForwardCommand(drive_sys, drive_slow_mprofile, 4, reverse)); // [measure]
    
    //shoot
    lss.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 45, .6)); // [measure]
    lss.add(new DriveToPointCommand(drive_sys, drive_fast_mprofile, 47.4, 141.64, fwd, 1));


    lss.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 131-10+8, .6)); // [measure]
     lss.add(new DriveForwardCommand(drive_sys, drive_slow_mprofile, 4, fwd)); //[measure]

    //lsa.add(new WaitUntilUpToSpeedCommand(flywheel_sys, 10));
    lss.add(new ShootCommand(intake, 3, 3));
    lss.add(new StopIntakeCommand(flywheel));


    lss.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 133+90, .6));
    lss.add(new DriveToPointCommand(drive_sys, drive_fast_mprofile, 9.75, 108, fwd, 1));

    // do a joe moment
    lss.add(new EndgameCommand(endgame_solenoid));

  return lss;
}
