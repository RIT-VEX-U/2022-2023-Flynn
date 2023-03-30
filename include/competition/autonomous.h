#pragma once
#include "vex.h"

// drive commands
#define DRIVE_TO_POINT_FAST(x,y,dir) (new DriveToPointCommand(drive_sys, drive_fast_mprofile, x, y, directionType::dir))
#define DRIVE_TO_POINT_SLOW(x,y,dir) (new DriveToPointCommand(drive_sys, drive_slow_mprofile, x, y, directionType::dir))
#define DRIVE_FORWARD_FAST(in, dir) (new DriveForwardCommand(drive_sys, drive_fast_mprofile, in, directionType::dir))
#define DRIVE_FORWARD_SLOW(in, dir) (new DriveForwardCommand(drive_sys, drive_slow_mprofile, in, directionType::dir))
#define DRIVE_TO_POINT_SLOW_PT(pt, dir) (new DriveToPointCommand(drive_sys, drive_slow_mprofile, pt, dir))
#define DRIVE_TO_POINT_FAST_PT(pt, dir) (new DriveToPointCommand(drive_sys, drive_fast_mprofile, pt, dir))

// turn commands
#define TURN_TO_HEADING(dir) (new TurnToHeadingCommand(drive_sys, *config.turn_feedback, dir, TURN_SPEED))
#define TURN_DEGREES(dir) (new TurnDegreesCommand(drive_sys, *config.turn_feedback, dir, TURN_SPEED))
#define TURN_TO_POINT(pt) (new TurnToPointCommand(drive_sys, odometry_sys, *config.turn_feedback, pt))

// shooting commands
#define SPIN_FW_AT(rpm) (new SpinRPMCommand(flywheel_sys, rpm))
#define AUTO_AIM (new VisionAimCommand(true, 150, 5))
#define WAIT_FOR_FLYWHEEL (new WaitUntilUpToSpeedCommand(flywheel_sys, THRESHOLD_RPM))
#define SHOOT_DISK (new ShootCommand(intake, SINGLE_SHOT_TIME, SINGLE_SHOT_VOLT))
#define TRI_SHOT_DISK (new ShootCommand(intake, TRI_SHOT_TIME, TRI_SHOT_VOLT))

// intaking commands
#define START_INTAKE (new StartIntakeCommand(intake, INTAKE_VOLT))
#define STOP_INTAKE (new StopIntakeCommand(intake))
#define CLEAR_DISKS (new ShootCommand(intake, .75, SINGLE_SHOT_VOLT))


/**
 * Contains all the code run during autonomous.
 */ 
void autonomous();