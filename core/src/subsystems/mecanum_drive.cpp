#include "../core/include/subsystems/mecanum_drive.h"
#include "../core/include/utils/math_util.h"
#include "../core/include/utils/pid.h"
#include "../core/include/utils/vector2d.h"
/**
* Create the Mecanum drivetrain object
*/
MecanumDrive::MecanumDrive(vex::motor &left_front, vex::motor &right_front, vex::motor &left_rear, vex::motor &right_rear, 
                           vex::rotation *lateral_wheel, vex::inertial *imu, mecanumdrive_config_t *config)
: left_front(left_front), right_front(right_front), left_rear(left_rear), right_rear(right_rear), // MOTOR CONFIG
  config(config), // CONFIG ...uh... config
  lateral_wheel(lateral_wheel), // NON-DRIVEN WHEEL CONFIG
  imu(imu) //IMU CONFIG
{

  // If the configuration exists, then allocate memory for the drive and turn pids
  if(config != NULL)
  {
    drive_pid = new Drive_Pid(config->drive_pid_conf);
    drive_gyro_pid = new Turn_Pid(config->drive_gyro_pid_conf);
    turn_pid = new Turn_Pid(config->turn_pid_conf);
  }

}

/**
 * Drive the robot using vectors. This handles all the math required for mecanum
 * control.
 *
 * @param direction  the direction to drive the robot, in degrees. 0 is forward,
 *                       180 is back, clockwise is positive, counterclockwise is
 * negative.
 * @param magnitude      How fast the robot should drive, in percent: 0.0->1.0
 * @param rotation       How fast the robot should rotate, in percent:
 * -1.0->+1.0
 */
void MecanumDrive::drive_raw(units::Angle direction, units::Voltage magnitude,
                             units::Voltage rotation)
{
  using namespace unit_literals;
  // ALGORITHM - "rotate" the vector by 45
  // degrees and apply each corner to
  // a wheel
  // .. Oh, and mix rotation too
  units::Voltage lf
      = (magnitude * cos(direction - units::Angle(PI / 4.0))) + rotation;
  units::Voltage rf
      = (magnitude * cos(direction + units::Angle(PI / 4.0))) - rotation;
  units::Voltage lr
      = (magnitude * cos(direction + units::Angle(PI / 4.0))) + rotation;

  units::Voltage rr
      = (magnitude * cos(direction - units::Angle(PI / 4.0))) - rotation;

  // Limit the output between -1.0 and +1.0
  lf = clamp(lf, -12_v, 12_v);
  rf = clamp(rf, -12_v, 12_v);
  lr = clamp(lr, -12_v, 12_v);
  rr = clamp(rr, -12_v, 12_v);

  // Finally, spin the motors
  left_front.spin(vex::directionType::fwd, lf.Convert(units::volt),
                  vex::voltageUnits::volt);
  right_front.spin(vex::directionType::fwd, rf.Convert(units::volt),
                   vex::voltageUnits::volt);
  left_rear.spin(vex::directionType::fwd, lr.Convert(units::volt),
                 vex::voltageUnits::volt);
  right_rear.spin(vex::directionType::fwd, rr.Convert(units::volt),
                  vex::voltageUnits::volt);
}

/**
 * Drive the robot with a mecanum-style / arcade drive. Inputs are in percent (-100.0 -> 100.0) straight from the controller.
 * Controls are mixed, so the robot can drive forward / strafe / rotate all at the same time. 
 *
 * @param left_y left joystick, Y axis (forward / backwards)
 * @param left_x left joystick, X axis (strafe left / right)
 * @param right_x right joystick, X axis (rotation left / right)
 * @param power = 2 how much of a "curve" there should be on drive controls; better for low speed maneuvers.
 *                Leave blank for a default curve of 2 (higher means more fidelity)
 */
void MecanumDrive::drive(double left_y, double left_x, double right_x, int power)
{
  // LATERAL CONTROLS - convert cartesion to a vector
  double magnitude = sqrt(pow(left_y / 100.0, 2) + pow(left_x / 100.0, 2));
  magnitude = pow(magnitude, power);

  units::Angle direction
      = units::radian * atan2(left_x / 100.0, left_y / 100.0);

  // ROTATIONAL CONTROLS - just the right x joystick
  double rotation = right_x / 100.0;

  //
  rotation = sign(rotation) * fabs(pow(rotation, power));

  return this->drive_raw(direction, 12_v * magnitude, 12_v * rotation);
}

/**
 * Drive the robot in a straight line automatically.
 * If the inertial was declared in the constructor, use it to correct while
 * driving. If the lateral wheel was declared in the constructor, use it for
 * more accurate positioning while strafing.
 *
 * @param inches   How far the robot should drive, in inches
 * @param direction    What direction the robot should travel in, in degrees.
 *                     0 is forward, +/-180 is reverse, clockwise is positive.
 * @param limit    The maximum voltage the robot should be commanded,
 * @param gyro_correction = true   Whether or not to use the gyro to help
 * correct while driving. Will always be false if no gyro was declared in the
 * constructor.
 * @return Whether or not the maneuver is complete.
 */
bool MecanumDrive::auto_drive(units::Length inches, units::Angle direction,
                              units::Voltage limit, bool gyro_correction)
{
  if(config == NULL || drive_pid == NULL)
  {
    fprintf(stderr, "Failed to run MecanumDrive::auto_drive - Missing mecanumdrive_config_t in constructor\n");
    return true; // avoid an infinte loop within auto
  }

  bool enable_gyro = gyro_correction && (imu != NULL);
  bool enable_wheel = (lateral_wheel != NULL);


  // INITIALIZE - only run ONCE "per drive" on startup
  if(init == true)
  {

    // Reset all driven encoders, and PID
    left_front.resetPosition();
    right_front.resetPosition();
    left_rear.resetPosition();
    right_rear.resetPosition();

    drive_pid->reset();

    // Reset only if gyro exists
    if(enable_gyro)
    {
      imu->resetRotation();
      drive_gyro_pid->reset();
      drive_gyro_pid->set_target(0.0_deg);
    }
    // reset only if lateral wheel exists
    if(enable_wheel)
      lateral_wheel->resetPosition();

    // Finish setting up the PID loop - max speed and position target
    drive_pid->set_limits(-abs(limit), abs(limit));
    drive_pid->set_target(abs(inches));

    init = false;
  }

  units::Length dist_avg = 0.0_in;
  units::Length drive_avg = 0.0_in;

  // This algorithm should be DEFINITELY good for forward/back, left/right.
  // Directions other than 0, 180, -90 and 90 will be hit or miss, but should be mostly right.
  // Recommend THOUROUGH testing at many angles.

  // IF in quadrant 1 or 3, use left front and right rear wheels as "drive" movement
  // ELSE in quadrant 2 or 4, use left rear and right front wheels as "drive" movement
  // Some wheels are NOT being averaged at any given time since the general mecanum algorithm makes them go slower than our robot speed
  // somewhat of a nasty hack, but wheel slippage should make up for it, and multivariable calc is hard.
  if ((direction > 0_deg && direction <= 90_deg)
      || (direction < -90_deg && direction > -180_deg)) {
    drive_avg = abs(arc_length(left_front.position(rev) * 1_rev,
                               config->drive_wheel_diam / 2.0))
                + abs(arc_length(right_rear.position(rev) * 1_rev,
                                 config->drive_wheel_diam / 2.0));

  } else {
    drive_avg = abs(arc_length(left_rear.position(rev) * 1_rev,
                               config->drive_wheel_diam / 2.0))
                + abs(arc_length(right_front.position(rev) * 1_rev,
                                 config->drive_wheel_diam / 2.0));
  }

  // Only use the encoder wheel if it exists.
  // Without the wheel should be usable, but with it will be muuuuch more accurate.
  if(enable_wheel)
  {
    // Distance driven = Magnitude = sqrt(x^2 + y^2)
    // Since drive_avg is already a polar magnitude, turn it into "Y" with cos(theta)
    units::Length lat
        = (lateral_wheel->position(rev)) * config->lateral_wheel_diam * PI;
    units::Length dir = cos(direction) * drive_avg;
    dist_avg = sqrt(square(lat) + square(dir));
  }else
  {
    dist_avg = drive_avg;
  }

  // ...double check to avoid an infinite loop
  dist_avg = abs(dist_avg);

  // ROTATION CORRECTION
  units::Voltage rot = 0_v;

  if(enable_gyro)
  {
    drive_gyro_pid->update(1_deg * imu->rotation());
    rot = drive_gyro_pid->get();
  }

  // Update the PID and drive
  drive_pid->update(drive_avg);

  this->drive_raw(direction, drive_pid->get(), rot);

  // Stop and return true whenever the robot has completed it's drive.
  if(drive_pid->is_on_target())
  {
    drive_raw(0_deg, 0_v, 0_v);
    init = true;
    return true;
  }

  // Return false while the robot is still driving.
  return false;
}

/**
 * Autonomously turn the robot X degrees over it's center point. Uses a closed
 * loop for control.
 * @param angle How much to rotate the robot. Clockwise postive.
 * @param speed What percentage to run the motors at: 0.0 -> 1.0
 * @param ignore_imu = false Whether or not to use the Inertial for determining
 * angle. Will instead use circumference formula + robot's wheelbase + encoders
 * to determine.
 *
 * @return whether or not the robot has finished the maneuver
 */
bool MecanumDrive::auto_turn(units::Angle angle, units::Voltage speed,
                             bool ignore_imu)
{
  // Make sure the configurations exist before continuing
  if(config == NULL || turn_pid == NULL)
  {
    fprintf(stderr, "Failed to run MecanumDrive::auto_turn - Missing mecanumdrive_config_t in constructor\n");
    return true;
  }

  // Decide whether or not to use the Inertial
  ignore_imu = ignore_imu || (this->imu == NULL);

  // INITIALIZE - clear encoders / imu / pid loops
  if(init == true)
  {
    if(ignore_imu)
    {
      this->left_front.resetPosition();
      this->right_front.resetPosition();
      this->left_rear.resetPosition();
      this->right_rear.resetPosition();
    }else
    {
      this->imu->resetRotation();
    }

    this->turn_pid->reset();
    this->turn_pid->set_limits(-abs(speed), abs(speed));
    this->turn_pid->set_target(angle);

    init = false;
  }

  // RUN PERIODICALLY

  units::Angle current_angle = 0.0_rev;

  if(ignore_imu)
  {
    units::Angle avg = 1_rev
                       * (left_front.position(rotationUnits::rev)
                          + left_rear.position(rotationUnits::rev)
                          - right_front.position(rotationUnits::rev)
                          - right_rear.position(rotationUnits::rev))
                       / 4.0;

    auto angle_per_driven_dist
        = 1_rev / units::arc_length(1_rev, config->wheelbase_width / 2.0);

    current_angle = angle_per_driven_dist
                    * units::arc_length(avg, config->drive_wheel_diam / 2.0);
  } else
  {
    current_angle = 1_deg * imu->rotation();
  }

  this->turn_pid->update(current_angle);
  this->drive_raw(0_rad, 0_v, turn_pid->get());

  // We have reached the target.
  if(this->turn_pid->is_on_target())
  {
    this->drive_raw(0_rad, 0_v, 0_v);
    init = true;
    return true;
  }

  return false;
}
