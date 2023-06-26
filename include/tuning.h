#pragma once

#include "../core/include/utils/feedback_base.h"

enum DriveType
{
    DRIVE, TURN
};

// Odometry Tuning
void tune_odometry_gear_ratio_right_wheel();
void tune_odometry_wheelbase();
void tune_odometry_offax_dist();
void tune_odometry_wheel_diam();

// Drive Tuning
void tune_drive_ff_ks(DriveType dt);
void tune_drive_ff_kv(DriveType dt, double ks);
void tune_drive_pid(DriveType dt);
void tune_drive_motion_maxv(DriveType dt);
void tune_drive_motion_accel(DriveType dt, double maxv);

// Flywheel Tuning
void tune_flywheel_ff();
void tune_flywheel_pid();
void tune_flywheel_distcalc();

void tune_shooting();


void tune_generic_pid(Feedback &pid2tune, double error_lower_bound, double error_upper_bound);
