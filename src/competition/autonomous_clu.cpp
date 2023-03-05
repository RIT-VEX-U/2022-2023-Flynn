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
#define SINGLE_SHOT_RECOVER_DELAY_MS 200
#define TRI_SHOT_TIME 1
#define TRI_SHOT_VOLT 12
#define TRI_SHOT_RECOVER_DELAY_MS 200

// drive commands
#define DRIVE_TO_POINT_FAST(x,y,dir) (new DriveToPointCommand(drive_sys, drive_fast_mprofile, x, y, directionType::dir))
#define DRIVE_TO_POINT_SLOW(x,y,dir) (new DriveToPointCommand(drive_sys, drive_slow_mprofile, x, y, directionType::dir))
#define DRIVE_FORWARD_FAST(in, dir) (new DriveForwardCommand(drive_sys, drive_fast_mprofile, in, directionType::dir))
#define DRIVE_FORWARD_SLOW(in, dir) (new DriveForwardCommand(drive_sys, drive_slow_mprofile, in, directionType::dir))

// turn commands
#define TURN_TO_HEADING(dir) (new TurnToHeadingCommand(drive_sys, *config.turn_feedback, dir, TURN_SPEED))
#define TURN_DEGREES(dir) (new TurnDegreesCommand(drive_sys, *config.turn_feedback, dir, TURN_SPEED))

// shooting commands
#define AUTO_AIM (new VisionAimCommand())
#define WAIT_FOR_FLYWHEEL (new WaitUntilUpToSpeedCommand(flywheel_sys, THRESHOLD_RPM))
#define SHOOT_DISK (new ShootCommand(intake, SINGLE_SHOT_TIME, SINGLE_SHOT_VOLT))
#define TRI_SHOT_DISK (new ShootCommand(intake, TRI_SHOT_TIME, TRI_SHOT_VOLT))

static void add_single_shot_cmd(CommandController &controller, double timeout=0.0)
{
    controller.add(WAIT_FOR_FLYWHEEL, timeout);
    controller.add(AUTO_AIM, timeout);
    controller.add(SHOOT_DISK);
    controller.add_delay(SINGLE_SHOT_RECOVER_DELAY_MS);
}

static void add_tri_shot_cmd(CommandController &controller, double timeout=0.0)
{
    controller.add(WAIT_FOR_FLYWHEEL, timeout);
    controller.add(TRI_SHOT_DISK);
    controller.add_delay(TRI_SHOT_RECOVER_DELAY_MS);
}

void testing()
{
    while(imu.isCalibrating()) {}
    CommandController ctrl = auto_non_loader_side();
    ctrl.run();

    while(true)
    {
        position_t pos = odometry_sys.get_position();
        printf("x: %2f, y: %2f, z: %2f\n", pos.x, pos.y, pos.rot);
        vexDelay(50);
    }
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
/*
CommandController auto_non_loader_side(){

    #define PAUSE return nlsa;
    CommandController nlsa;

    // Initialization
    position_t start_pos = {.x=105.75, .y=86.5, .rot=90}; 
    nlsa.add(new OdomSetPosition(odometry_sys, start_pos));

    // Drive to roller
    nlsa.add(DRIVE_TO_POINT_FAST(104, 113, fwd));
    nlsa.add(TURN_TO_HEADING(0));
    
    // Spin Roller
    nlsa.add(DRIVE_FORWARD_FAST(4, fwd));
    nlsa.add(new SpinRollerCommandAUTO(drive_sys, roller));
    nlsa.add(DRIVE_FORWARD_FAST(12, rev));
    

    // Spin and shoot
    // nlsa.add(new SpinRPMCommand(flywheel_sys, SHOOTING_RPM)); 
    nlsa.add(TURN_TO_HEADING(228));

    nlsa.add(DRIVE_TO_POINT_FAST(74, 83, fwd));
    nlsa.add(TURN_TO_HEADING(153));
    add_single_shot_cmd(nlsa, 3);
    add_single_shot_cmd(nlsa, 3);

    // NORMAL STOP
    // return nlsa;

    // Furthur auto testing
    // Drive towards the 3 disks along the barrier
    nlsa.add(TURN_TO_HEADING(320));
    nlsa.add(DRIVE_TO_POINT_FAST(83.6, 72.5, fwd));
    nlsa.add(TURN_TO_HEADING(270));

    // Pick up the first
    nlsa.add(new StartIntakeCommand(intake, INTAKE_VOLT));
    nlsa.add(DRIVE_TO_POINT_SLOW(83.4, 55.4, fwd), 3);
    nlsa.add(DRIVE_TO_POINT_FAST(83.4, 66.6, rev));

    // Pick up second disk
    nlsa.add(TURN_TO_HEADING(302));
    nlsa.add(DRIVE_TO_POINT_SLOW(90, 57, fwd), 3);
    nlsa.add(DRIVE_TO_POINT_FAST(73.7, 74.6, rev));
    // Pick up third disk
    nlsa.add(TURN_TO_HEADING(270));
    nlsa.add(DRIVE_TO_POINT_SLOW(73.7, 55.7, fwd), 3);
    nlsa.add(DRIVE_TO_POINT_FAST(68.3, 74.7, rev));

    // Turn around to shoot
    nlsa.add(TURN_TO_HEADING(147));

    PAUSE

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

*/
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
/*
CommandController prog_skills_non_loader_side(){

    CommandController nlss;

    position_t start_pos = {.x = 0, .y = 0, .rot = 90};
    nlss.add(new OdomSetPosition(odometry_sys, start_pos));

    // Shoot 1 (2 disks)
    nlss.add(new FlapUpCommand());
    nlss.add(DRIVE_FORWARD_SLOW(0, fwd));
    nlss.add(TURN_TO_HEADING(0));
    add_tri_shot_cmd(nlss, 3);

    // Roller 1 
    nlss.add(DRIVE_TO_POINT_FAST(0, 0, rev));
    nlss.add(TURN_TO_HEADING(0));
    // Drive forward and back to roll
    nlss.add(DRIVE_FORWARD_FAST(0, fwd));
    nlss.add(DRIVE_FORWARD_FAST(0, rev));
    // One more time
    nlss.add(DRIVE_FORWARD_FAST(0, fwd));
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
    // Drive forward and back to roll
    nlss.add(DRIVE_FORWARD_FAST(0, fwd));
    nlss.add(DRIVE_FORWARD_FAST(0, rev));
    // One more time
    nlss.add(DRIVE_FORWARD_FAST(0, fwd));
    nlss.add(DRIVE_FORWARD_FAST(0, rev));
    nlss.add(TURN_TO_HEADING(0));

    // Intake Disk 2
    nlss.add(new StartIntakeCommand(intake, INTAKE_VOLT));
    nlss.add(DRIVE_TO_POINT_SLOW(0, 0, fwd));
    nlss.add(TURN_DEGREES(0));
    
    // Intake Disk 3
    nlss.add(DRIVE_TO_POINT_SLOW(0, 0, fwd));
    nlss.add(TURN_TO_HEADING(0));

    // Shoot 2 (3 disks)
    nlss.add(new FlapDownCommand());
    nlss.add(new StopIntakeCommand(intake));
    add_single_shot_cmd(nlss, 3);
    add_single_shot_cmd(nlss, 3);
    add_single_shot_cmd(nlss, 3);

    // Drive to Intake 2
    nlss.add(DRIVE_TO_POINT_FAST(0, 0, fwd));
    nlss.add(DRIVE_TO_POINT_FAST(0, 0, fwd));

    // Intake 2 disk 1
    nlss.add(new StartIntakeCommand(intake, INTAKE_VOLT));
    nlss.add(DRIVE_TO_POINT_SLOW(0, 0, fwd));
    nlss.add(DRIVE_TO_POINT_FAST(0, 0, rev));

    // Intake 2 disk 2
    nlss.add(DRIVE_TO_POINT_SLOW(0, 0, fwd));
    nlss.add(DRIVE_TO_POINT_FAST(0, 0, rev));

    // Intake 2 disk 3
    nlss.add(DRIVE_TO_POINT_SLOW(0, 0, fwd));
    nlss.add(DRIVE_TO_POINT_FAST(0, 0, rev));

    // Drive to Shooting Pos
    nlss.add(TURN_TO_HEADING(0));
    nlss.add(DRIVE_TO_POINT_FAST(0, 0, fwd));
    nlss.add(TURN_TO_HEADING(0));
    nlss.add(DRIVE_TO_POINT_FAST(0, 0, fwd));
    nlss.add(new StopIntakeCommand(intake));
    
    // Shoot 3 (3 disks)
    nlss.add(new FlapUpCommand());
    nlss.add(TURN_TO_HEADING(0));
    add_tri_shot_cmd(nlss, 3);

    // quick 'n dirty wall align
    nlss.add(TURN_TO_HEADING(0));
    nlss.add(DRIVE_TO_POINT_SLOW(0, 0, rev));
    // === INSERT WALL ALIGN FUNCTION
    nlss.add(DRIVE_FORWARD_FAST(0, fwd));
    
    // Drive to Intake 3
    nlss.add(TURN_TO_HEADING(0));
    nlss.add(DRIVE_TO_POINT_FAST(0, 0, fwd));
    nlss.add(TURN_TO_HEADING(0));

    // Intake 3 disk 1-3
    nlss.add(DRIVE_TO_POINT_SLOW(0, 0, fwd));

    // Drive to Shoot 4
    nlss.add(TURN_TO_HEADING(0));
    nlss.add(DRIVE_TO_POINT_FAST(0, 0, fwd));
    nlss.add(TURN_TO_HEADING(0));
    nlss.add(DRIVE_TO_POINT_FAST(0, 0, fwd));
    nlss.add(new StopIntakeCommand(intake));

    // Shoot 4
    nlss.add(TURN_TO_HEADING(0));
    add_tri_shot_cmd(nlss, 3);

    // Cheeky lil' wall align
    nlss.add(TURN_TO_HEADING(0));
    nlss.add(DRIVE_TO_POINT_SLOW(0, 0, rev));
    // === INSERT WALL ALIGN FUNCTION
    nlss.add(DRIVE_FORWARD_FAST(0, fwd));

    // Drive to Intake 4
    nlss.add(DRIVE_TO_POINT_FAST(0, 0, fwd));
    nlss.add(TURN_TO_HEADING(0));

    // Intake 4 disk 1
    nlss.add(new StartIntakeCommand(intake, INTAKE_VOLT));
    nlss.add(DRIVE_TO_POINT_SLOW(0, 0, fwd));
    nlss.add(DRIVE_TO_POINT_FAST(0, 0, rev));

    // Intake 4 disk 2
    nlss.add(DRIVE_TO_POINT_SLOW(0, 0, fwd));
    nlss.add(DRIVE_TO_POINT_FAST(0, 0, rev));

    // Intake 4 disk 3
    nlss.add(DRIVE_TO_POINT_SLOW(0, 0, fwd));
    nlss.add(DRIVE_TO_POINT_FAST(0, 0, rev));

    // Drive to Shoot 5
    nlss.add(TURN_TO_HEADING(0));
    nlss.add(DRIVE_TO_POINT_FAST(0, 0, fwd));
    nlss.add(TURN_TO_HEADING(0));
    nlss.add(DRIVE_TO_POINT_FAST(0, 0, fwd));
    nlss.add(new StopIntakeCommand(intake));

    // Shoot 5
    nlss.add(TURN_TO_HEADING(0));
    add_tri_shot_cmd(nlss, 3);

    // A wall align for the road
    nlss.add(TURN_TO_HEADING(0));
    nlss.add(DRIVE_TO_POINT_SLOW(0, 0, rev));
    // === INSERT WALL ALIGN FUNCTION
    nlss.add(DRIVE_FORWARD_FAST(0, fwd));

    // Drive to Intake 5
    nlss.add(TURN_TO_HEADING(0));
    nlss.add(DRIVE_TO_POINT_FAST(0, 0, fwd));

    // Intake 5
    nlss.add(new StartIntakeCommand(intake, INTAKE_VOLT));
    nlss.add(DRIVE_TO_POINT_SLOW(0, 0, fwd));
    
    // Shoot 5
    nlss.add(new FlapDownCommand());
    nlss.add(TURN_TO_HEADING(0));
    nlss.add(new StopIntakeCommand(intake));
    add_single_shot_cmd(nlss, 3);

    // Endgame
    nlss.add(TURN_TO_HEADING(0));
    nlss.add(new DriveStopCommand(drive_sys));
    nlss.add(new EndgameCommand(endgame_solenoid));

  return nlss;
}*/