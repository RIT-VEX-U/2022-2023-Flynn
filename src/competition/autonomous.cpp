#include "../include/competition/autonomous.h"

#define TURN_SPEED 0.6


//functions that define autos. construct a CommandController when called.
CommandController auto_loader_side();
CommandController auto_non_loader_side();
CommandController prog_skills_loader_side();
CommandController prog_skills_non_loader_side();


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
        return prog_skills_non_loader_side();
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
    int loader_side_full_shot_rpm = 3000;  // [measure]

    position_t start_pos = position_t{.x = 9.75, .y = 105.25, .rot = -90};

    CommandController lsa;
    lsa.add(new OdomSetPosition(odometry_sys, start_pos));

    // spin -90 degree roller
    lsa.add(new DriveForwardCommand(drive_sys, drive_slow_mprofile, 1, fwd)); //[measure]
    lsa.add(new SpinRollerCommand(drive_sys, roller));
    lsa.add(new DriveForwardCommand(drive_sys, drive_slow_mprofile, 4, reverse)); // [measure]
    
    //shoot
    lsa.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 180, TURN_SPEED)); // [measure]
    lsa.add(new SpinRPMCommand(flywheel_sys, loader_side_full_shot_rpm));
    lsa.add(new WaitUntilUpToSpeedCommand(flywheel_sys, 10));
    lsa.add(new ShootCommand(intake, 3, 4));

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
  |        |*___| (R) |
  |___           |    |
  |   |          |    |
  |   |          |    |  --> 90 degrees
  |   |          |____|
  |___|  ____         |
  |(B)  |__*_|        |
  +-------------------+
           v 0 degrees

 Human Instructions:
 Align robot to specified place and angle using NON LOADER SIDE AUTO jig
*/
CommandController auto_non_loader_side(){
    int non_loader_side_shot_rpm = 3000;  // [measure]
    CommandController nlsa;
    position_t start_pos = {.x = 128, .y = 84, .rot = 90}; // [measure]
    nlsa.add(new OdomSetPosition(odometry_sys, start_pos));
    // Arrow 1 -------------------
    nlsa.add(new DriveToPointCommand(drive_sys, drive_fast_mprofile, 128, 96, fwd, 1)); // [measure]
    nlsa.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 0, TURN_SPEED)); // [measure]
    
    
    // Arrow 2 -------------------
    nlsa.add(new DriveForwardCommand(drive_sys, drive_fast_mprofile, 2, fwd, 1)); // [measure]
    nlsa.add(new SpinRollerCommand(drive_sys, roller));
    nlsa.add(new DriveForwardCommand(drive_sys, drive_fast_mprofile, 2, reverse, 1)); // [measure]

    // Spin and shoot
    nlsa.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 170, TURN_SPEED)); //[measure]
    nlsa.add(new SpinRPMCommand(flywheel_sys, non_loader_side_shot_rpm));
    nlsa.add(new WaitUntilUpToSpeedCommand(flywheel_sys, 10));
    nlsa.add(new ShootCommand(intake, 3, 2)); // [measure]
    return nlsa;
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
    lss.add(new DriveForwardCommand(drive_sys, drive_slow_mprofile, 1, fwd)); //[measure]
    lss.add(new SpinRollerCommand(drive_sys, roller));
    lss.add(new DriveForwardCommand(drive_sys, drive_slow_mprofile, 4, reverse)); // [measure]
    
    lss.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 180, TURN_SPEED)); //[measure]
    
    // Arrow 2 -------------------------
    // intake corner disk
    lss.add(new StartIntakeCommand(intake, 12));
    lss.add(new DriveToPointCommand(drive_sys, drive_fast_mprofile, 17.5, 17.5, fwd, 1)); // [measure]
    lss.add(new StopIntakeCommand(intake));

    // align to 180 degree roller
    lss.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 45, TURN_SPEED));  // [measure]
    lss.add(new DriveToPointCommand(drive_sys, drive_fast_mprofile, 24, 115.0, fwd, 1)); //[measure] for sure
    lss.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 180, TURN_SPEED));  // [measure]
    
    // spin 180 degree roller
    lss.add(new DriveForwardCommand(drive_sys, drive_slow_mprofile, 2, fwd)); //[measure]
    lss.add(new SpinRollerCommand(drive_sys, roller));
    lss.add(new DriveForwardCommand(drive_sys, drive_fast_mprofile, 2, reverse)); //[measure]

    //spin and shoot 3
    lss.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 80, TURN_SPEED)); //[measure]
    lss.add(new SpinRPMCommand(flywheel_sys, 3500)); // [measure]
    lss.add(new WaitUntilUpToSpeedCommand(flywheel_sys, 10));
    lss.add(new ShootCommand(intake, 3, .5));

    // Arrow 3 -------------------------
    lss.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 45, TURN_SPEED)); //[measure]
    lss.add(new StartIntakeCommand(intake, 12));
    lss.add(new DriveToPointCommand(drive_sys, drive_fast_mprofile, 70, 50, fwd, 1)); //[measure]
    lss.add(new StopIntakeCommand(intake));

    //face hoop and fire
    lss.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 135, TURN_SPEED)); // [measure]
    lss.add(new SpinRPMCommand(flywheel_sys, 3000)); // [measure]
    lss.add(new WaitUntilUpToSpeedCommand(flywheel_sys, 10));
    lss.add(new ShootCommand(intake, 2, 4)); // [measure]
    
    // Arrow 4 -------------------------
    lss.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 75, TURN_SPEED)); // [measure]
    lss.add(new DriveToPointCommand(drive_sys, drive_fast_mprofile, 80, 132, fwd,  1)); //[measure]

    // Move to endgame pos
    lss.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 10, TURN_SPEED)); // [measure]
    lss.add(new DriveToPointCommand(drive_sys, drive_fast_mprofile, 122.5, 122.5, fwd,  1)); //[measure]

    // Endgame
    lss.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 215, TURN_SPEED)); //[measure]
    lss.add(new EndgameCommand(endgame_solenoid));

  return lss;
}



/*
Skills Non-loader side

Map from page 40 of the game manual

(R) = Red Hoop, Blue Zone
(B) = Blue Hoop, Red Zone
 *  = Starting position for this auto

           ^ 180 degrees
  +-------------------+
  |        |*___| (R) |
  |___           |    |
  |   |          |    |
  |   |          |    |  --> 90 degrees
  |   |          |____|
  |___|  ____         |
  |(B)  |__*_|        |
  +-------------------+
           v 0 degrees

 Human Instructions:
 Align robot to specified place and angle using NON LOADER SIDE SKILLS jig
*/
CommandController prog_skills_non_loader_side(){

  CommandController nlss;
  
  position_t start_pos = {.x = 128, .y = 84, .rot = 90}; // [measure]
  nlss.add(new OdomSetPosition(odometry_sys, start_pos));

  // Arrow 1 -------------------
  nlss.add(new DriveToPointCommand(drive_sys, drive_fast_mprofile, 128, 96, fwd, 1)); // [measure]
  nlss.add(new TurnToHeadingCommand(drive_sys, *config.turn_feedback, 0, 1)); // [measure]
    
    
    // Arrow 2 -------------------
  nlss.add(new DriveForwardCommand(drive_sys, drive_fast_mprofile, 2, fwd, 1)); // [measure]
  nlss.add(new SpinRollerCommand(drive_sys, roller)); 
  nlss.add(new DriveForwardCommand(drive_sys, drive_fast_mprofile, 2, reverse, 1)); // [measure]


  return nlss;
}
