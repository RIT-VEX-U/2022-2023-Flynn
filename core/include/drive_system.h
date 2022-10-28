/**
 * File: drive_system.h
 * Desc:
 *    Interface for drive systems
 */

#pragma once

using namespace vex;

class DriveSystem {
  public:
    /**
     * Use odometry to automatically drive the robot to a point on the field.
     * X and Y is the final point we want the robot.
     */
    virtual bool drive_to_point(double x, double y, double speed, double correction_speed, directionType dir=directionType::fwd);

    /**
     * Use the controller input to manually drive the robot
     */
    virtual void op_drive(double axis1, double axis2, double axis3, double axis4, int power=1);

    /**
     * Autonomously turn the robot X degrees to the right (negative for left), with a maximum motor speed
     * of percent_speed (-1.0 -> 1.0)
     * 
     * Uses a PID loop for it's control.
     */
    virtual bool turn_degrees(double degrees, double percent_speed);

    /**
     * Turn the robot in place to an exact heading relative to the field.
     * 0 is forward, and 0->360 is clockwise.
     */
    virtual bool turn_to_heading(double heading_deg, double speed);

    /**
     * Stops rotation of all the motors using their "brake mode"
     */
    virtual void stop();
};