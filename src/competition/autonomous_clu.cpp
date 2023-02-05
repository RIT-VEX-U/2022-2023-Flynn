#include "competition/autonomous_clu.h"
#include "robot-config.h"
#include "automation.h"
#include "vision.h"

#define TURN_SPEED 0.6
#define INTAKE_VOLT 12
#define SHOOTING_RPM 3500
#define THRESHOLD_RPM 150
#define SINGLE_SHOT_TIME 0.2
#define SINGLE_SHOT_VOLT 12
#define SINGLE_SHOT_RECOVER_DELAY 1

// drive commands
#define DRIVE_TO_POINT_FAST(x,y,dir) (new DriveToPointCommand(drive_sys, drive_fast_mprofile, x, y, directionType::dir))
#define DRIVE_TO_POINT_SLOW(x,y,dir) (new DriveToPointCommand(drive_sys, drive_slow_mprofile, x, y, directionType::dir))
#define DRIVE_FORWARD_FAST(in, dir) (new DriveForwardCommand(drive_sys, drive_fast_mprofile, in, directionType::dir))
#define DRIVE_FORWARD_SLOW(in, dir) (new DriveForwardCommand(drive_sys, drive_slow_mprofile, in, directionType::dir))

// turn commands
#define TURN_TO_HEADING(dir) (new TurnToHeadingCommand(drive_sys, *config.turn_feedback, dir, TURN_SPEED))
#define TURN_DEGREES(dir) (new TurnDegreesCommand(drive_sys, *config.turn_feedback, dir, TURN_SPEED))

// shooting commands
#define AUTO_AIM (new VisionAimCommand(cam, {RED_GOAL, BLUE_GOAL}, drive_sys))
#define WAIT_FOR_FLYWHEEL (new WaitUntilUpToSpeedCommand(flywheel_sys, THRESHOLD_RPM))
#define SHOOT_DISK (new ShootCommand(intake, SINGLE_SHOT_TIME, SINGLE_SHOT_VOLT))

static void add_single_shot_cmd(CommandController &controller, double vis_timeout=0.0)
{
    controller.add(WAIT_FOR_FLYWHEEL);
    if(vis_timeout == 0.0)
        controller.add(AUTO_AIM);
    else
        controller.add(AUTO_AIM, vis_timeout);
    controller.add(SHOOT_DISK);
    controller.add_delay(SINGLE_SHOT_RECOVER_DELAY);
}

/*
Auto Non-loader side
JOEBOT
Map from page 40 of the game manual

(R) = Red Hoop, Blue Zone
(B) = Blue Hoop, Red Zone
 *  = Starting position for this auto
           ^ 180 degrees
  0-------------------+
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
    CommandController nlsa;

    // Initialization
    position_t start_pos = {.x=0, .y=0, .rot=90}; 
    nlsa.add(new OdomSetPosition(odometry_sys, start_pos));

    // Arrow 1 -------------------
    nlsa.add(DRIVE_TO_POINT_FAST(127.9, 114, fwd));
    nlsa.add(TURN_TO_HEADING(0));
    
    // Arrow 2 -------------------
    nlsa.add(DRIVE_FORWARD_FAST(4, fwd));
    nlsa.add(new SpinRollerCommandAUTO(drive_sys, roller));
    nlsa.add(DRIVE_FORWARD_FAST(12, rev));

    // Spin and shoot
    nlsa.add(new SpinRPMCommand(flywheel_sys, SHOOTING_RPM)); 
    nlsa.add(TURN_TO_HEADING(225));
    nlsa.add(DRIVE_TO_POINT_FAST(89, 76.5, fwd));
    nlsa.add(TURN_TO_HEADING(145));
    add_single_shot_cmd(nlsa, 3);
    add_single_shot_cmd(nlsa, 3);

    // NORMAL STOP
    return nlsa;

    // Furthur auto testing
    // Drive towards the 3 disks along the barrier
    nlsa.add(TURN_TO_HEADING(0));
    nlsa.add(DRIVE_TO_POINT_FAST(0, 0, fwd));
    nlsa.add(TURN_TO_HEADING(0));

    // Pick up the first
    nlsa.add(new StartIntakeCommand(intake, INTAKE_VOLT));
    nlsa.add(DRIVE_TO_POINT_SLOW(0, 0, fwd), 3);
    nlsa.add(DRIVE_TO_POINT_FAST(0, 0, rev));

    // Pick up second disk
    nlsa.add(TURN_TO_HEADING(0));
    nlsa.add(DRIVE_TO_POINT_SLOW(0, 0, fwd), 3);
    nlsa.add(DRIVE_TO_POINT_FAST(0, 0, rev));
    // Pick up third disk
    nlsa.add(TURN_TO_HEADING(0));
    nlsa.add(DRIVE_TO_POINT_SLOW(0, 0, fwd), 3);
    nlsa.add(DRIVE_TO_POINT_FAST(0, 0, rev));

    // Turn around to shoot
    nlsa.add(TURN_TO_HEADING(0));
    nlsa.add(DRIVE_TO_POINT_FAST(0, 0, fwd));
    nlsa.add(new StopIntakeCommand(intake));
    nlsa.add(TURN_TO_HEADING(0));
    
    
    // Shoot 1-3
    nlsa.add(new DriveStopCommand(drive_sys));
    add_single_shot_cmd(nlsa, 3);
    add_single_shot_cmd(nlsa, 3);
    add_single_shot_cmd(nlsa, 3);
    
    // Disable flywheel
    nlsa.add(new FlywheelStopCommand(flywheel_sys));

    return nlsa;
    
    
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

    position_t start_pos = {.x = 0, .y = 0, .rot = 90};
    nlss.add(new OdomSetPosition(odometry_sys, start_pos));

    // Shoot 1
    add_single_shot_cmd(nlss, 3);
    add_single_shot_cmd(nlss, 3);
    nlss.add(TURN_TO_HEADING(90));

    // Roller 1 
    nlss.add(DRIVE_TO_POINT_FAST(0, 0, fwd));
    nlss.add(TURN_TO_HEADING(0));
    nlss.add(DRIVE_FORWARD_FAST(0, fwd));
    // nlss.add(new SpinRollerCommandAUTO(drive_sys, roller)); 
    nlss.add(DRIVE_FORWARD_FAST(0, rev));
     nlss.add(TURN_TO_HEADING(0));

    // Intake Disk 1
    nlss.add(new StartIntakeCommand(intake, INTAKE_VOLT));
    nlss.add(DRIVE_TO_POINT_SLOW(0, 0, fwd));
    nlss.add(TURN_TO_HEADING(0));

    // Roller 2
    nlss.add(DRIVE_TO_POINT_FAST(0, 0, fwd));
    nlss.add(TURN_TO_HEADING(0));
    nlss.add(new StopIntakeCommand(intake));
    nlss.add(DRIVE_FORWARD_FAST(0, fwd));
    nlss.add(new SpinRollerCommandAUTO(drive_sys, roller));
    nlss.add(DRIVE_FORWARD_FAST(0, rev));
    nlss.add(TURN_TO_HEADING(0));

    // Intake Disk 2
    nlss.add(new StartIntakeCommand(intake, INTAKE_VOLT));
    nlss.add(DRIVE_TO_POINT_SLOW(0, 0, fwd));
    nlss.add(TURN_DEGREES(0));
    
    // Intake Disk 3
    nlss.add(DRIVE_TO_POINT_SLOW(0, 0, fwd));
    nlss.add(TURN_TO_HEADING(0));

    // Shoot




  return nlss;
}