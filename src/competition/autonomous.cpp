#include "../include/competition/autonomous.h"


//functions that define autos. construct a CommandController when called.
CommandController auto_loader_side();
CommandController auto_non_loader_side();
CommandController prog_skills_loader_side();
CommandController prog_skills_non_loader_size();


// Pick the auto based off of which one we've selected on the screen
CommandController get_chosen_auto(){
    std::string choice = autochooser.get_choice();
    if (choice == AutoLoaderSideDisplayName){
        return auto_loader_side();
    }  else if(choice == AutoNonLoaderSideDisplayName){
        return auto_non_loader_side();
    } else if (choice == SkillsLoaderSideDisplayName){
        return prog_skills_loader_side();
    } else if (choice == SkillsLoaderSideDisplayName){
        return prog_skills_non_loader_size();
    }   

    // Empty Command Controller - something has gone very wrong
    return CommandController();
}





/**
 * Contains all the code run during autonomous.
 */ 
void autonomous()
{

    // while(!imu.is_calibrating){
    //  vexDelay(20);
    // }
    CommandController current_auto = get_chosen_auto();   
    current_auto.run();

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
    int loader_side_full_shot_rpm = 3000;  // TODO measure this RPM based on testing

    position_t start_pos = position_t{.x = 9.75, .y = 105.25, .rot = -90};

    CommandController lsa;
    lsa.add(new OdomSetPosition(odometry_sys, start_pos));

    // spin -90 degree roller
    lsa.add(new DriveForwardCommand(drive_sys, drive_slow_mprofile, 1, fwd)); //[measure]
    lsa.add(new SpinRollerCommand(roller));
    lsa.add(new DriveForwardCommand(drive_sys, drive_slow_mprofile, 4, reverse)); // [measure]
    
    lsa.add(new TurnToHeadingCommand(drive_sys, turn_fast_mprofile, 180));
    
    // intake corner disk
    lsa.add(new StartIntakeCommand(intake, 12));
    lsa.add(new DriveToPointCommand(drive_sys, drive_fast_mprofile, 17.5, 122.5, fwd, 1));
    lsa.add(new StopIntakeCommand(intake));

    // align to 180 degree roller
    lsa.add(new TurnToHeadingCommand(drive_sys, turn_fast_mprofile, 45));
    lsa.add(new DriveToPointCommand(drive_sys, drive_fast_mprofile, 25, 116.0, fwd, 1)); //[measure] for sure
    lsa.add(new TurnToHeadingCommand(drive_sys, turn_fast_mprofile, 180));
    
    // spin 180 degree roller
    lsa.add(new DriveForwardCommand(drive_sys, drive_slow_mprofile, 2, fwd)); //[measure]
    lsa.add(new SpinRollerCommand(roller));
    lsa.add(new DriveForwardCommand(drive_sys, drive_fast_mprofile, 2, reverse)); //[measure]
    


    return lsa;
}

/*
Auto Non-loader side

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
 Align robot to specified place and angle using NON LOADER SIDE AUTO jig
*/
CommandController auto_non_loader_side(){
    int non_loader_side_full_court_shot_rpm = 3000;  // TODO measure this RPM based on testing
    CommandController non_loader_side_auto;

    non_loader_side_auto.add(new SpinRPMCommand(flywheel_sys, non_loader_side_full_court_shot_rpm));
    non_loader_side_auto.add(new WaitUntilUpToSpeedCommand(flywheel_sys, 10)); //TODO measure what a good +/- threshold for shooting is
    non_loader_side_auto.add(new ShootCommand(intake, 2, .75)); // TODO measure how long we need to wait to shoot all 
    non_loader_side_auto.add(new FlywheelStopCommand(flywheel_sys));
    non_loader_side_auto.add(new TurnDegreesCommand(drive_sys, turn_fast_mprofile, -60, 1)); // Angle to point directly upwards. Towards far field edge. // TODO measure this angle once initial shooting angle is determined
    non_loader_side_auto.add(new DriveForwardCommand(drive_sys, drive_fast_mprofile, 20, fwd, 1)); // Drive to align vertically with the spinners. // TODO measure this distance on the field with the actual robot
    non_loader_side_auto.add(new TurnDegreesCommand(drive_sys, turn_fast_mprofile, -90, 1)); // Turn from facing directly upwards to facing the spinner.
    non_loader_side_auto.add(new DriveForwardCommand(drive_sys, drive_fast_mprofile, 2, fwd, 1)); // Drive until touching the spinner. // TODO measure this distance on the field with the actual robot
    non_loader_side_auto.add(new SpinRollerCommand(roller)); // TODO measure the distances in SpinRollerCommand to travel

    return non_loader_side_auto;
}


/*
Skills loader side

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
 Align robot to specified place and angle using LOADER SIDE SKILLS jig
*/
CommandController prog_skills_loader_side(){
  
    position_t start_pos = position_t{.x = 9.75, .y = 105.25, .rot = -90};

    CommandController lss;
    lss.add(new OdomSetPosition(odometry_sys, start_pos));

    // Arrow 1 -------------------------
    // spin -90 degree roller
    lss.add(new DriveForwardCommand(drive_sys, drive_slow_mprofile, 1, fwd)); //[measure]
    lss.add(new SpinRollerCommand(roller));
    lss.add(new DriveForwardCommand(drive_sys, drive_slow_mprofile, 4, reverse)); // [measure]
    
    lss.add(new TurnToHeadingCommand(drive_sys, turn_fast_mprofile, 180));
    
    // Arrow 2 -------------------------
    // intake corner disk
    lss.add(new StartIntakeCommand(intake, 12));
    lss.add(new DriveToPointCommand(drive_sys, drive_fast_mprofile, 17.5, 122.5, fwd, 1));
    lss.add(new StopIntakeCommand(intake));

    // align to 180 degree roller
    lss.add(new TurnToHeadingCommand(drive_sys, turn_fast_mprofile, 45));
    lss.add(new DriveToPointCommand(drive_sys, drive_fast_mprofile, 25, 116.0, fwd, 1)); //[measure] for sure
    lss.add(new TurnToHeadingCommand(drive_sys, turn_fast_mprofile, 180));
    
    // spin 180 degree roller
    lss.add(new DriveForwardCommand(drive_sys, drive_slow_mprofile, 2, fwd)); //[measure]
    lss.add(new SpinRollerCommand(roller));
    lss.add(new DriveForwardCommand(drive_sys, drive_fast_mprofile, 2, reverse)); //[measure]

    //spin and shoot 3
    lss.add(new TurnToHeadingCommand(drive_sys, turn_fast_mprofile, 80)); //[measure]
    lss.add(new SpinRPMCommand(flywheel_sys, 3500));
    lss.add(new WaitUntilUpToSpeedCommand(flywheel_sys, 10));
    lss.add(new ShootCommand(intake, 3, .5));


    // Arrow 3 -------------------------
    lss.add(new TurnToHeadingCommand(drive_sys, turn_fast_mprofile, 45)); //[measure]
    lss.add(new StartIntakeCommand(intake, 12));
    lss.add(new DriveToPointCommand(drive_sys, drive_fast_mprofile, 80, 80, fwd, 1)); //[measure]

    lss.add(new TurnToHeadingCommand(drive_sys, turn_fast_mprofile, 135));


  return lss;
}



/*
Skills Non-loader side

Map from page 40 of the game manual

(R) = Red Hoop, Blue Zone
(B) = Blue Hoop, Red Zone
 *  = Starting position for this auto
+-------------------+
|        |*___| (R) |
|___           |    |
|   |          |    |
|   |          |    |  --> 90 degrees
|   |          |____|
|___|  ____         |
|(B)  |___*|        |
+-------------------+

 Human Instructions:
 Align robot to specified place and angle using NON LOADER SIDE SKILLS jig
*/
CommandController prog_skills_non_loader_size(){

  CommandController prog_skills_non_loader_side;

  return prog_skills_non_loader_side;
}
