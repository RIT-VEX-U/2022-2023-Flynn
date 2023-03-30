#pragma once
#include "vex.h"

// drive commands
#define DRIVE_TO_POINT_FAST(x,y,dir) (new DriveToPointCommand(drive_sys, drive_fast_mprofile, x, y, directionType::dir))
#define DRIVE_TO_POINT_SLOW(x,y,dir) (new DriveToPointCommand(drive_sys, drive_slow_mprofile, x, y, directionType::dir))
#define DRIVE_FORWARD_FAST(in, dir) (new DriveForwardCommand(drive_sys, drive_fast_mprofile, in, directionType::dir))
#define DRIVE_FORWARD_SLOW(in, dir) (new DriveForwardCommand(drive_sys, drive_slow_mprofile, in, directionType::dir))

// turn commands
#define TURN_TO_HEADING(dir) (new TurnToHeadingCommand(drive_sys, *config.turn_feedback, dir, TURN_SPEED))
#define TURN_DEGREES(dir) (new TurnDegreesCommand(drive_sys, *config.turn_feedback, dir, TURN_SPEED))
#define TURN_TO_POINT(x, y) (new TurnToPointCommand(drive_sys, odometry_sys, *config.turn_feedback, {x, y}))

// shooting commands
#define AUTO_AIM (new VisionAimCommand(true, 150, 5))
#define WAIT_FOR_FLYWHEEL (new WaitUntilUpToSpeedCommand(flywheel_sys, THRESHOLD_RPM))
#define SHOOT_DISK (new ShootCommand(intake, SINGLE_SHOT_TIME, SINGLE_SHOT_VOLT))
#define TRI_SHOT_DISK (new ShootCommand(intake, TRI_SHOT_TIME, TRI_SHOT_VOLT))


/**
 * Contains all the code run during autonomous.
 */ 
void autonomous();