#include "../include/competition/autonomous.h"


//functions that define autos. construct a GenericAuto when called.
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
  return CommandController();
}





/**
 * Contains all the code run during autonomous.
 */ 
void autonomous()
{
    CommandController current_auto = auto_loader_side();    
    current_auto.run();

}


/*
Auto loader side

Map from page 40 of the game manual

(R) = Red Hoop, Blue Zone
(B) = Blue Hoop, Red Zone
 *  = Starting position for this auto
+-------------------+
|        |____| (R) |
|___           |    |
| * |          |    |
|   |          |    |
|   |          |_*__|
|___|  ____         |
|(B)  |____|        |
+-------------------+

 Human Instructions:
 Align robot to specified place and angle using LOADER SIDE AUTO jig
*/
CommandController auto_loader_side(){
    int loader_side_full_court_shot_rpm = 3000;  // TODO measure this RPM based on testing
    CommandController loader_side_auto;

    loader_side_auto.add(new SpinRPMCommand(flywheel_sys, loader_side_full_court_shot_rpm));
    loader_side_auto.add(new WaitUntilUpToSpeedCommand(flywheel_sys, 10)); //TODO measure what a good +/- threshhold for shooting is
    loader_side_auto.add(new ShootCommand(intake, 2)); // TODO measure how long we need to wait to shoot all 
    loader_side_auto.add(new FlywheelStopCommand(flywheel_sys));
    loader_side_auto.add(new TurnDegreesCommand(drive_sys, 60, 1)); // Angle to point directly upwards. Towards far field edge. // TODO measure this angle once initial shooting angle is determined
    loader_side_auto.add(new DriveForwardCommand(drive_sys, 2, fwd, 1)); // Drive to align vertically with the spinners. // TODO measure this distance on the field with the actual robot
    loader_side_auto.add(new TurnDegreesCommand(drive_sys, 90, 1)); // Turn from facing directly upwards to facing the spinner.
    loader_side_auto.add(new DriveForwardCommand(drive_sys, 2, fwd, 1)); // Drive until touching the spinner. // TODO measure this distance on the field with the actual robot
    loader_side_auto.add(new SpinRollerCommand(roller)); // TODO measure the distances in SpinRollerCommand to travel

    return loader_side_auto;
}

/*
Auto Non-loader side

Map from page 40 of the game manual

(R) = Red Hoop, Blue Zone
(B) = Blue Hoop, Red Zone
 *  = Starting position for this auto
+-------------------+
|        |*___| (R) |
|___           |    |
|   |          |    |
|   |          |    |
|   |          |____|
|___|  ____         |
|(B)  |___*|        |
+-------------------+

 Human Instructions:
 Align robot to specified place and angle using NON LOADER SIDE AUTO jig
*/
CommandController auto_non_loader_side(){
    int non_loader_side_full_court_shot_rpm = 3000;  // TODO measure this RPM based on testing
    CommandController non_loader_side_auto;

    non_loader_side_auto.add(new SpinRPMCommand(flywheel_sys, non_loader_side_full_court_shot_rpm));
    non_loader_side_auto.add(new WaitUntilUpToSpeedCommand(flywheel_sys, 10)); //TODO measure what a good +/- threshhold for shooting is
    non_loader_side_auto.add(new ShootCommand(intake, 2)); // TODO measure how long we need to wait to shoot all 
    non_loader_side_auto.add(new FlywheelStopCommand(flywheel_sys));
    non_loader_side_auto.add(new TurnDegreesCommand(drive_sys, -60, 1)); // Angle to point directly upwards. Towards far field edge. // TODO measure this angle once initial shooting angle is determined
    non_loader_side_auto.add(new DriveForwardCommand(drive_sys, 20, fwd, 1)); // Drive to align vertically with the spinners. // TODO measure this distance on the field with the actual robot
    non_loader_side_auto.add(new TurnDegreesCommand(drive_sys, -90, 1)); // Turn from facing directly upwards to facing the spinner.
    non_loader_side_auto.add(new DriveForwardCommand(drive_sys, 2, fwd, 1)); // Drive until touching the spinner. // TODO measure this distance on the field with the actual robot
    non_loader_side_auto.add(new SpinRollerCommand(roller)); // TODO measure the distances in SpinRollerCommand to travel

    return non_loader_side_auto;
}



CommandController prog_skills_loader_side(){
  const int times_to_shoot_then_load_from_station = 4; //TODO figure out how many iterations we want to do this for
  const double length_between_before_loader_and_shooting_position = 60; //TODO measure this distance
  const double prog_skills_loader_side_shot_rpm = 3000; // TODO measure how many rpms we need to make this shot

  CommandController prog_skills_loader_side;
  for (int i = 0; i< times_to_shoot_then_load_from_station; i++){
    prog_skills_loader_side.add(new SpinRPMCommand(flywheel_sys, prog_skills_loader_side_shot_rpm));
    prog_skills_loader_side.add(new WaitUntilUpToSpeedCommand(flywheel_sys, 10)); //TODO measure what a good +/- threshhold for shooting is
    prog_skills_loader_side.add(new ShootCommand(intake, 1)); // TODO use shooter when it exists
    prog_skills_loader_side.add(new DriveForwardCommand(drive_sys, length_between_before_loader_and_shooting_position, vex::reverse, 1));
    prog_skills_loader_side.add(new StartIntakeCommand(intake, 10)); // TODO measure voltage needed to intake
    prog_skills_loader_side.add(new DriveForwardCommand(drive_sys, length_between_before_loader_and_shooting_position, vex::fwd, .25)); // TODO measure how slow we need to to pick up disks from the loading zone
    prog_skills_loader_side.add(new StopIntakeCommand(intake));

  }  

  return prog_skills_loader_side;
}
CommandController prog_skills_non_loader_size(){
  CommandController prog_skills_non_loader_side;


  return prog_skills_non_loader_side;
}
