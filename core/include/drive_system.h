#pragma once

using namespace vex;

class DriveSystem {
  public:
    enum OpStyle { tank, arcade };

    virtual bool drive_to_point(double x, double y, double speed, double correction_speed, directionType dir=directionType::fwd);

    virtual void op_drive(double axis1, double axis2, double axis3, double axis4, OpStyle style, int power=1);

    virtual bool turn_degrees(double degrees, double percent_speed);

    virtual bool turn_to_heading(double heading_deg, double speed);

    virtual void stop();
};