/**
 * File: drive_system.h
 * Desc:
 *    Interface for drive systems
 */

#pragma once

#include "../core/include/subsystems/odometry/odometry_base.h"

using namespace vex;

class DriveSystem {
  public:
    DriveSystem(OdometryBase *odom) : odom(odom) {};


    // ==== AUTO DRIVING ====

    // drive_to_point and turn_to_heading MUST be implemented by any subclasses of DriveSystem

    /**
     * Use odometry to automatically drive the robot to a point on the field.
     * X and Y is the final point we want the robot.
     */
    virtual bool drive_to_point(double x, double y, double speed, double correction_speed, directionType dir=directionType::fwd);

    /**
     * Turn the robot in place to an exact heading relative to the field.
     * 0 is forward, and 0->360 is clockwise.
     */
    virtual bool turn_to_heading(double heading_deg, double speed);

    // drive_forward and turn_degrees can be overridden, but offer default 
    // implementations in terms of drive_to_point and turn_to_heading

    /**
     * Autonomously drive forward or backwards, X inches infront or behind the robot's current position.
     * This driving method is relative, so excessive use may cause the robot to get off course!
     *
     * @param inches Distance to drive in a straight line
     * @param speed How fast the robot should travel, 0 -> 1.0
     * @param correction How much the robot should correct for being off angle
     * @param dir Whether the robot is travelling forwards or backwards
     */
    bool drive_forward(double inches, double speed, double correction, directionType dir); // TODO: include implementation in terms of drive_to_point

    /**
     * Autonomously turn the robot X degrees to the right (negative for left), with a maximum motor speed
     * of percent_speed (-1.0 -> 1.0)
     * 
     * Uses a PID loop for it's control.
     */
    virtual bool turn_degrees(double degrees, double percent_speed);  // TODO: include implementation in terms of turn_to_heading


    // ==== MANUAL DRIVING ====

    /**
     * Use the controller input to manually drive the robot
     * Must be implemented by subclasses
     */
    virtual void op_drive(double axis1, double axis2, double axis3, double axis4, int power=1);


    // ==== ODOMETRY ====

    /**
     * Have odometry functions be called through the DriveSystem so users
     * don't have direct access to it
     */

    /**
     * Gets the current position and rotation
     */
    position_t get_position(void) { return odom->get_position(); }

    /**
     * Sets the current position of the robot
     */
    void set_position(const position_t &newpos=zero_pos) { odom->set_position(newpos); }

    /**
     * Update the current position on the field based on the sensors
     */
    position_t update() { return odom->update(); }


    inline static constexpr position_t zero_pos = {.x=0, .y=0, .rot=90};

    /**
     * Stops rotation of all the motors using their "brake mode"
     */
    virtual void stop();

  private:
    OdometryBase *odom;
};