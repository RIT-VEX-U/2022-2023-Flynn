#pragma once

// Odometry Tuning
void tune_odometry_wheelbase();
void tune_odometry_offax_dist();
void tune_odometry_wheel_diam();

// Drive Tuning
void tune_drive_ff_ks();
void tune_drive_ff_kv();
void tune_drive_pid();
void tune_drive_motion_maxv();
void tune_drive_motion_accel(double maxv);

// Flywheel Tuning
void tune_flywheel_ff();
void tune_flywheel_pid();
void tune_flywheel_distcalc();