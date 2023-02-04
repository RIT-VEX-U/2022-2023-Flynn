#include "../include/autonomous_flynn.h"

#define DriveToPointFast(x, y) new DriveToPointCommand(drive_sys, drive_fast_mprofile, x, y, fwd, 1.0)
#define DriveForwardFast(dist, dir) new DriveForwardCommand(drive_sys, drive_fast_mprofile, dist, dir, 1.0)
#define TurnToHeading(heading_deg) new TurnToHeadingCommand(drive_sys, *config.turn_feedback, heading_deg, .6)

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
    position_t start_pos = position_t{.x = 9.75, .y = 105.25, .rot = -90};

    CommandController lss;

    lss.add(new OdomSetPosition(odometry_sys, start_pos));
    lss.add(new SpinRPMCommand(flywheel_sys, 3400));


    // spin -90 degree roller
    lss.add(new DriveForwardCommand(drive_sys, drive_slow_mprofile, 1, fwd)); //[measure]
    lss.add(DriveForwardFast(1, fwd));
    lss.add(new SpinRollerCommandSKILLS(drive_sys, roller));
    lss.add(DriveForwardFast(4, reverse));
    
    //shoot
    lss.add(TurnToHeading(45)); // [measure]
    lss.add(DriveToPointFast(47.4, 141.64));


    lss.add(TurnToHeading(129)); // [measure]
    lss.add(DriveForwardFast(4, fwd)); //[measure]

    lss.add(new ShootCommand(intake, 3, 3));
    lss.add(new StopIntakeCommand(flywheel));


    lss.add(TurnToHeading(223));
    lss.add(DriveToPointFast(9.75, 108));

    // do a joe moment
    lss.add(new EndgameCommand(endgame_solenoid));

  return lss;
}
