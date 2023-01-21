#include "competition/autonomous.h"



//functions to choose which auto we want. construct a GenericAuto when chosen.
CommandController auto_loader_side();
CommandController auto_non_loader_side();

CommandController prog_skills_loader_side();
CommandController priog_skills_non_loader_size();




/**
 * Contains all the code run during autonomous.
 */ 
void autonomous()
{
    CommandController current_auto = auto_loader_side();    
    //current_auto.run(true);

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
    //loader_side_auto.add(new ShooterShootAllCommand()); // TODO use shooter when it exists
    loader_side_auto.add(new FlywheelStopCommand(flywheel_sys));
    loader_side_auto.add(new TurnDegreesCommand(drive_sys, 60, 1)); // Angle to point directly upwards. Towards far field edge. // TODO measure this angle once initial shooting angle is determined
    loader_side_auto.add(new DriveForwardCommand(drive_sys, 2, fwd, 1)); // Drive to align vertically with the spinners. // TODO measure this distance on the field with the actual robot
    loader_side_auto.add(new TurnDegreesCommand(drive_sys, 90, 1)); // Turn from facing directly upwards to facing the spinner.
    loader_side_auto.add(new DriveForwardCommand(drive_sys, 2, fwd, 1)); // Drive until touching the spinner. // TODO measure this distance on the field with the actual robot
    loader_side_auto.add(new SpinRollerCommand(roller)); // TODO implement auto_spin_spinner_to_red based on testing on our field

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
    //loader_side_auto.add(new ShooterShootAllCommand()); // TODO use shooter when it exists
    non_loader_side_auto.add(new FlywheelStopCommand(flywheel_sys));
    non_loader_side_auto.add(new TurnDegreesCommand(drive_sys, -60, 1)); // Angle to point directly upwards. Towards far field edge. // TODO measure this angle once initial shooting angle is determined
    non_loader_side_auto.add(new DriveForwardCommand(drive_sys, 20, fwd, 1)); // Drive to align vertically with the spinners. // TODO measure this distance on the field with the actual robot
    non_loader_side_auto.add(new TurnDegreesCommand(drive_sys, -90, 1)); // Turn from facing directly upwards to facing the spinner.
    non_loader_side_auto.add(new DriveForwardCommand(drive_sys, 2, fwd, 1)); // Drive until touching the spinner. // TODO measure this distance on the field with the actual robot
    non_loader_side_auto.add(new SpinRollerCommand(roller)); // TODO implement auto_spin_spinner_to_red based on testing on our field

    return non_loader_side_auto;
}

