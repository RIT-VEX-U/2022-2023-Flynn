#include "competition/autonomous_flynn.h"
#include "../core/include/splash.h"
#include "tuning.h"
#include "vision.h"
#include <algorithm>
#include <stdio.h>

#define DRIVE_TO_POINT_SLOW_PT(pt, dir) (new DriveToPointCommand(drive_sys, drive_slow_mprofile, pt, dir))
#define DRIVE_TO_POINT_FAST_PT(pt, dir) (new DriveToPointCommand(drive_sys, drive_fast_mprofile, pt, dir))

#define CALIBRATE_IMU()       \
  while (imu.isCalibrating()) \
  {                           \
  }

const double TURN_SPEED = 0.6;
const double INTAKE_VOLT = 12;
const double SINGLE_SHOT_TIME = 0.2;
const double SINGLE_SHOT_VOLT = 6;
const double THRESHOLD_RPM = 150;
int glbl_vision_center = 135;

const double TRI_SHOT_TIME = 1.0;
const double TRI_SHOT_VOLT = 2;
const double TRI_SHOT_RECOVER_DELAY_MS = 200;

static void add_single_shot_cmd(CommandController &controller, double vis_timeout = 1.0)
{
  controller.add(WAIT_FOR_FLYWHEEL, vis_timeout);
  if (vis_timeout == 0.0)
    controller.add(AUTO_AIM);
  else
    controller.add(AUTO_AIM, vis_timeout);
  controller.add(SHOOT_DISK);
  controller.add_delay(600);
}

static void add_tri_shot_cmd(CommandController &controller, double timeout = 0.0)
{
  controller.add(WAIT_FOR_FLYWHEEL, timeout);
  controller.add(TRI_SHOT_DISK, 2.0);
  controller.add_delay(TRI_SHOT_RECOVER_DELAY_MS);
}

void pleasant_opcontrol();

int print_odom()
{
  while (true)
  {
    pose_t pos = odometry_sys.get_position();
    printf("%.2f, %.2f, %.2f\n", pos.x, pos.y, pos.rot);
    vexDelay(500);
  }
  return 0;
}

void test_stuff()
{
  CALIBRATE_IMU();

  vex::task odom_print(print_odom);

  CommandController mine = skills_rollers_last(); // disk_rush_auto();
  mine.run();
  printf("Running pleasant op\n");

  pleasant_opcontrol();
}

void pleasant_opcontrol()
{
  // Initialization
  double oneshot_time = .02; // Change 1 second to whatever is needed
  bool oneshotting = false;

  main_controller.ButtonUp.pressed([]()
                                   { flywheel_sys.spinRPM(flywheel_sys.getDesiredRPM() + 250); });
  main_controller.ButtonDown.pressed([]()
                                     { flywheel_sys.spinRPM(flywheel_sys.getDesiredRPM() - 250); });
  main_controller.ButtonR1.pressed([]()
                                   { intake.spin(vex::reverse, 12, volt); }); // Intake
  main_controller.ButtonR2.pressed([]()
                                   { intake.spin(fwd, 12, volt); }); // Shoot

  main_controller.ButtonB.pressed([]()
                                  { odometry_sys.set_position(); });

  // intake.spin(fwd, 12, volt);
  main_controller.ButtonX.pressed([]()
                                  {  intake.spin(fwd, 12, volt); vexDelay(40); intake.spin(fwd, 0, volt); });

  main_controller.ButtonL2.pressed([]()
                                   { flapup_solenoid.set(false); }); // Single Shoot

  main_controller.ButtonL1.pressed([]()
                                   { flapup_solenoid.set(true); });

  VisionAimCommand visaim(false, 145, 5);

  timer loop_timer;
  loop_timer.reset();
  double delay_time = 0.0;
  double loop_time = 0.0;
  // Periodic
  while (true)
  {

    // ========== DRIVING CONTROLS ==========
    if (main_controller.ButtonA.pressing())
    {
      right_motors.setStopping(brakeType::hold);
      left_motors.setStopping(brakeType::hold);
      left_motors.stop();
      right_motors.stop();
    }
    else
      drive_sys.drive_arcade(main_controller.Axis3.position(pct) / 100.0, main_controller.Axis1.position(pct) / 120.0);

    // ========== MANIPULATING CONTROLS ==========

    if (main_controller.ButtonY.pressing() && main_controller.ButtonRight.pressing())
    {
      endgame_solenoid.set(true);
    }

    oneshotting = oneshot_tmr.time(vex::sec) < oneshot_time;
    if (!main_controller.ButtonR1.pressing() && !main_controller.ButtonR2.pressing() && !oneshotting)
    {
      intake.stop();
    }

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========

    // ======== Real timing ========
    loop_time = loop_timer.time(vex::timeUnits::sec);
    delay_time = max(0.02 - loop_time, 0.0);
    vexDelay((int)(delay_time * 1000));
    loop_timer.reset();
  }
}

/*
Skills loader side

Map from page 40 of the game manual

(R) = Red Hoop, Blue Zone
(B) = Blue Hoop, Red Zone
 *  = Starting position for this auto
    -x   ^ 180 degrees
  0-------------------+
  |        |____| (R) |
  |___           |    |
  | * |          |    |
  |   |          |    |  --> 90 degrees (+y)
  |   |          |_*__|
  |___|  ____         |
  |(B)  |____|        |
  +-------------------+ (140, 140)
           v 0 degrees
           (+x)

 Human Instructions:
 Align robot to specified place and angle using LOADER SIDE SKILLS jig
*/

CommandController prog_skills_loader_side()
{
  glbl_vision_center = 125;
  target_red = true;
  flapup_solenoid.set(true);
  pose_t start_pos = pose_t{.x = 36.0, .y = 12.2, .rot = -90};

  pose_t roller_in_pos = {.x = 36.0, .y = 4.16, .rot = -90};

  CommandController lss;
  lss.add(new OdomSetPosition(odometry_sys, start_pos)); // #1

  // spin -90 degree roller
  lss.add(new SpinRollerCommand(roller_in_pos), 5.0);
  lss.add(DRIVE_FORWARD_FAST(6, rev));

  point_t corner_disk_point = {.x = 10, .y = 12};

  // intake corner disk
  lss.add({
      TURN_TO_POINT(corner_disk_point)->withTimeout(1.5),               // #5
      START_INTAKE,                                                     // #6
      DRIVE_TO_POINT_SLOW_PT(corner_disk_point, fwd)->withTimeout(1.5), // #7
      DRIVE_FORWARD_FAST(4, rev)->withTimeout(1.5),                     // #8
  });

  point_t shoot_point = {.x = 10.5, .y = 93};

  // align to 180 degree roller
  lss.add(new SpinRPMCommand(flywheel_sys, 2900)); // #21

  point_t roller_out_pos2 = {.x = 14, .y = 33};
  pose_t roller_in_pos2 = {.x = 4.20, .y = 33, .rot = 180};

  lss.add({
      TURN_TO_POINT(roller_out_pos2)->withTimeout(1.5),
      DRIVE_TO_POINT_FAST_PT(roller_out_pos2, fwd)->withTimeout(1.5), // #12
      TURN_TO_HEADING(180)->withTimeout(1.5),                         // #13

      STOP_INTAKE,

      // spin 180 degree roller
      (new SpinRollerCommand(roller_in_pos2))->withTimeout(6.0),
      DRIVE_FORWARD_FAST(4.0, rev)->withTimeout(1.5),
      // drive to shoot point
      TURN_TO_POINT(shoot_point)->withTimeout(1.5), // #17
      // PrintOdomContinous->withTimeout(18888),
      DRIVE_TO_POINT_FAST_PT(shoot_point, fwd)->withTimeout(2.0) // #19
  });

  // Shoot
  lss.add(TURN_TO_HEADING(84), 0.85); // #20

  lss.add_delay(1000);

  lss.add(WAIT_FOR_FLYWHEEL, 1.0);
  lss.add(SHOOT_DISK);
  lss.add_delay(1000);

  lss.add(WAIT_FOR_FLYWHEEL, 1.0);
  lss.add(SHOOT_DISK);
  lss.add_delay(1000);

  lss.add(WAIT_FOR_FLYWHEEL, 1.0);
  lss.add(CLEAR_DISKS);

  // lss.add(PrintOdomContinous); /// the guy youre looking for =================================================================================> :)

  // DISKS AGAINST L PIECE
  point_t disk_pos1 = {.x = 24.0, .y = 81.5};
  point_t disk_pos2 = {.x = 32.0, .y = 81.5};
  point_t disk_pos3 = {.x = 42.0, .y = 81.5};

  point_t disk_prep_pos1 = {.x = 25, .y = 70};
  point_t disk_prep_pos2 = {.x = 28, .y = 70};
  point_t disk_prep_pos3 = {.x = 33, .y = 70};

  // disks against L piece
  lss.add({
      // farthest
      DRIVE_TO_POINT_FAST_PT(disk_prep_pos1, vex::reverse)->withTimeout(2.0), // #27
      START_INTAKE,                                                           // #26
      TURN_TO_POINT(disk_pos1)->withTimeout(1.5),                             // #28
      DRIVE_TO_POINT_SLOW_PT(disk_pos1, fwd)->withTimeout(2.0),               // #29

      // middle
      DRIVE_TO_POINT_FAST_PT(disk_prep_pos2, vex::reverse)->withTimeout(2.0), // #30
      TURN_TO_POINT(disk_pos2)->withTimeout(1.5),                             // #31
      DRIVE_TO_POINT_SLOW_PT(disk_pos2, fwd)->withTimeout(2.0),               // #32

      // closest disk
      DRIVE_TO_POINT_FAST_PT(disk_prep_pos3, vex::reverse)->withTimeout(2.0),       // #33
      TURN_TO_POINT(disk_pos3)->withTimeout(1.5),                                   // #34
      DRIVE_TO_POINT_SLOW_PT(disk_pos3, vex::directionType::fwd)->withTimeout(2.0), // #35
      DRIVE_TO_POINT_FAST_PT(disk_prep_pos3, vex::reverse)->withTimeout(2.0),       // #36
  });
  lss.add(new SpinRPMCommand(flywheel_sys, 3100)); // #40

  // point_t pre_shoot_point_other = {.x = 10.5, .y = 70};
  // point_t shoot_point_other = {.x = 10.5, .y = 90};

  // lss.add(TURN_TO_POINT(pre_shoot_point_other), 1.5);        // #37
  // lss.add(DriveToPointFastPt(pre_shoot_point_other), 4.0); // #38
  // lss.add(TURN_TO_POINT(shoot_point_other), 1.5);            // #37
  // lss.add(DriveToPointFastPt(shoot_point_other), 4.0);     // #38
  lss.add(new FlapDownCommand());

  // lss.add(TURN_TO_HEADING(75), 0.5); // #39
  point_t hoop_point_early = {.x = 17, .y = 123};
  lss.add(TURN_TO_POINT(hoop_point_early));

  add_single_shot_cmd(lss);
  add_single_shot_cmd(lss);
  add_single_shot_cmd(lss);

  // Arrow 3 -------------------------

  point_t start_of_line = {.x = 34.5, .y = 49};
  point_t end_of_line = {.x = 65, .y = 82};
  point_t out_of_way_point = {.x = 73, .y = 118};

  lss.add(new SpinRPMCommand(flywheel_sys, 3200)); // #40

  // go to line and collect line
  lss.add({
      START_INTAKE,
      // Start of line
      TURN_TO_POINT(start_of_line)->withTimeout(2.0), // #39
      // START_INTAKE,                       // #40
      DRIVE_TO_POINT_FAST_PT(start_of_line, fwd)->withTimeout(2.0), // #41

      // Drive to End of line
      TURN_TO_POINT(end_of_line)->withTimeout(2.0),               // #43
      DRIVE_TO_POINT_FAST_PT(end_of_line, fwd)->withTimeout(2.0), // $44

  });

  point_t hoop_point = {.x = 17, .y = 113};
  lss.add(TURN_TO_POINT(hoop_point), 1.0);
  lss.add(DRIVE_FORWARD_FAST(6, rev));

  lss.add(new FlapDownCommand());
  add_single_shot_cmd(lss);
  add_single_shot_cmd(lss);
  add_single_shot_cmd(lss);

  // Arrow 4 -------------------------
  point_t endgame_point = {.x = 116.36, .y = 106.23};

  lss.add(TURN_TO_POINT(endgame_point), 1.0);               // [measure]
  lss.add(DRIVE_TO_POINT_FAST_PT(endgame_point, fwd), 4.0); //[measure]
  lss.add(TURN_TO_HEADING(48), 3.0);

  lss.add(new PrintOdomCommand(odometry_sys), 400);

  lss.add(new EndgameCommand(endgame_solenoid));

  return lss;
}

/*
Auto loader side

Map from page 40 of the game manual

(R) = Red Hoop, Blue Zone
(B) = Blue Hoop, Red Zone
 *  = Starting position for this auto

           ^ 180 degrees
  +-------------------+
  |        |____| (R) |
  |___           |    |
  | * |          |    |
  |   |          |    |  --> 90 degrees
  |   |          |_*__|
  |___|  ____         |
  |(B)  |____|        |
  +-------------------+
           v 0 degrees

 Human Instructions:
 Align robot to specified place and angle using LOADER SIDE AUTO jig
*/

CommandController auto_loader_side()
{

  CommandController lsa;

  flap_down();

  pose_t start_pos = pose_t{.x = 36.0, .y = 12.2, .rot = -90};
  pose_t roller_in_pos = {.x = 36.0, .y = 5.16, .rot = -90};

  // Rollers =======================================================
  lsa.add(new OdomSetPosition(odometry_sys, start_pos));
  lsa.add(SPIN_FW_AT(3250));
  lsa.add(new SpinRollerCommand(roller_in_pos), 5.0);
  // lsa.add(new OdomSetPosition(odometry_sys, start_pos));
  lsa.add(DRIVE_FORWARD_FAST(2, rev));
  point_t roller_out_pos = {.x = 36, .y = 16.0};
  lsa.add(DRIVE_TO_POINT_FAST_PT(roller_out_pos, vex::reverse));

  point_t out_of_way_pos1 = {.x = 79.0, .y = 14.0};
  point_t shoot_point1 = {.x = 65, .y = 50};

  // First Shot =======================================================
  lsa.add({
      // Wa
      TURN_TO_POINT(out_of_way_pos1)->withTimeout(2.0),
      DRIVE_TO_POINT_FAST_PT(out_of_way_pos1, fwd)->withTimeout(2.0),

      TURN_TO_POINT(shoot_point1)->withTimeout(2.0),
      DRIVE_TO_POINT_FAST_PT(shoot_point1, fwd)->withTimeout(2.0),

  });

  lsa.add(TURN_TO_HEADING(126.6), 2.0);
  lsa.add(AUTO_AIM, 1.0);
  lsa.add(WAIT_FOR_FLYWHEEL, 1.0);
  lsa.add(SHOOT_DISK);

  // lsa.add(TURN_TO_HEADING(125.6), 2.0);
  lsa.add(AUTO_AIM, 1.0);
  lsa.add(WAIT_FOR_FLYWHEEL, 1.0);
  lsa.add(CLEAR_DISKS);

  auto lerp_point = [](point_t a, point_t b, float t)
  {
    return point_t{.x = a.x * (1 - t) + b.x * t, .y = a.y * (1 - t) + b.y * t};
  };

  point_t pre_stack3_pos = {.x = 70.0, .y = 48.0};
  point_t after_stack3_pos = {.x = 44.0, .y = 20.0};

  point_t mid_stack3_pos = lerp_point(pre_stack3_pos, after_stack3_pos, .6);

  // point_t shoot_point1 = {.x = 64, .y = 51};
  // double goal_pos1_deg = 120.0;

  // Third Shot =======================================================

  lsa.add(SPIN_FW_AT(3550));
  lsa.add({
      // line up to stack
      TURN_TO_POINT(pre_stack3_pos)->withTimeout(2.0),
      DRIVE_TO_POINT_FAST_PT(pre_stack3_pos, fwd)->withTimeout(2.0),

      TURN_TO_POINT(mid_stack3_pos)->withTimeout(2.0),
      DRIVE_TO_POINT_FAST_PT(mid_stack3_pos, fwd)->withTimeout(2.0),
      new DelayCommand(200),
      START_INTAKE,

      // grab stack
      TURN_TO_POINT(after_stack3_pos)->withTimeout(2.0),
      DRIVE_TO_POINT_SLOW_PT(after_stack3_pos, fwd)->withTimeout(2.0),

      // to shoot point
      TURN_TO_POINT(shoot_point1)->withTimeout(2.0),
      DRIVE_TO_POINT_FAST_PT(shoot_point1, fwd)->withTimeout(2.0),
      STOP_INTAKE,

  });

  lsa.add(TURN_TO_HEADING(128), 2.0);
  lsa.add(AUTO_AIM, 1.0);
  lsa.add(WAIT_FOR_FLYWHEEL, 1.0);
  lsa.add(SHOOT_DISK);

  lsa.add(AUTO_AIM, 1.0);
  lsa.add(WAIT_FOR_FLYWHEEL, 1.0);
  lsa.add(SHOOT_DISK);

  lsa.add(AUTO_AIM, 1.0);
  lsa.add(WAIT_FOR_FLYWHEEL, 1.0);
  lsa.add(CLEAR_DISKS);
  lsa.add_delay(600);

  point_t disk1_pos = {.x = 84.0, .y = 46.0};
  point_t disk2_pos = {.x = 83.0, .y = 36.0};
  point_t disk3_pos = {.x = 83.0, .y = 27.0};

  point_t lineup_disk2_pos = {.x = 70.0, .y = 41.0};
  point_t lineup_disk3_pos = {.x = 70.0, .y = 32.0};

  // Second Shot =======================================================
  lsa.add(SPIN_FW_AT(3500));
  lsa.add({

      // First
      TURN_TO_POINT(disk1_pos)->withTimeout(2.0),
      START_INTAKE,
      DRIVE_TO_POINT_FAST_PT(disk1_pos, fwd)->withTimeout(2.0),

      // Second
      DRIVE_TO_POINT_FAST_PT(lineup_disk2_pos, vex::reverse)->withTimeout(2.0),
      TURN_TO_POINT(disk2_pos)->withTimeout(2.0),
      DRIVE_TO_POINT_FAST_PT(disk2_pos, fwd)->withTimeout(2.0),

      // Third
      DRIVE_TO_POINT_FAST_PT(lineup_disk3_pos, vex::reverse)->withTimeout(2.0),
      TURN_TO_POINT(disk3_pos)->withTimeout(2.0),
      DRIVE_TO_POINT_FAST_PT(disk3_pos, fwd)->withTimeout(2.0),
  });

  // go shoot
  //  point_t disk3_pos = {.x = 86.0, .y = 27.0};
  point_t halfway = {.x = 75, .y = 39}; // turn here
  point_t shoot_point2 = {.x = 64, .y = 51};

  lsa.add({
      DRIVE_TO_POINT_FAST_PT(halfway, vex::reverse)->withTimeout(2.0),
      STOP_INTAKE,
      TURN_TO_POINT(shoot_point2)->withTimeout(2.0),
      DRIVE_TO_POINT_FAST_PT(shoot_point2, fwd)->withTimeout(2.0),
  });

  lsa.add(TURN_TO_HEADING(130), 2.0);
  lsa.add(AUTO_AIM, 1.0);
  lsa.add(WAIT_FOR_FLYWHEEL, 1.0);
  lsa.add(SHOOT_DISK);

  lsa.add(AUTO_AIM, 1.0);
  lsa.add(WAIT_FOR_FLYWHEEL, 1.0);
  lsa.add(SHOOT_DISK);
  lsa.add(WAIT_FOR_FLYWHEEL, 1.0);

  lsa.add(AUTO_AIM, 1.0);
  lsa.add(CLEAR_DISKS);
  lsa.add_delay(600);

  // Line up to 3 stack
  point_t pre_op_lineup = {.x = 43.6, .y = 24.05};
  lsa.add({
      TURN_TO_POINT(pre_op_lineup)->withTimeout(2.0),
      DRIVE_TO_POINT_FAST_PT(pre_op_lineup, fwd)->withTimeout(2.0),
      TURN_TO_HEADING(130)->withTimeout(2.0),
  });

  return lsa;
}

// start 84, 13
// turn to goal
// shooty 2
// turn to pt
// forward to 84, 46
// backward to start
// turn to point
// drive to 63, 31
// turn to goal
// shooty 3
// turn to 3 stack
// drive forward to intook 3
// torn to goal
// shooty 3
// turn to preroller
// turn to roller
// we do a little rolling.jpeg
#define ss_time 0.08
#define clear_time 0.5
#define SHOT_SPACING 1000

#define SHOOT_3(time_ms) new ShootCommand(intake, ss_time, 12), new DelayCommand(time_ms), new ShootCommand(intake, ss_time, 12), new DelayCommand(time_ms), new ShootCommand(intake, clear_time, 12), new DelayCommand(time_ms)
// #define SHOOT_3(time_ms) new ShootCommand(intake, time_ms, 12)

#define TURN_TO_AND_DRIVE_TO_FAST(pt, timeout) (TURN_TO_POINT(pt)->withTimeout(timeout)), (DRIVE_TO_POINT_FAST_PT(pt, fwd)->withTimeout(timeout))
CommandController auto_loader_side_disks_last()
{
#define point_t point_t

  point_t goal_point = {15, 125};
  pose_t start_point_odom = {.x = 83, .y = 13, .rot = 90.f};
  point_t start_point = {.x = start_point_odom.x, .y = start_point_odom.y};
  point_t disk_line_end = {83, 47};
  point_t pre_3_stack = {65, 45};
  point_t post_3_stack = {52, 30};
  point_t pre_roller_pt = {31, 15};

  CommandController lsdl;
  lsdl.add({
      new OdomSetPosition(odometry_sys, start_point_odom),
      TURN_TO_POINT(goal_point),
      SHOOT_3(1500),
      TURN_TO_POINT(disk_line_end),
      START_INTAKE,
      DRIVE_TO_POINT_FAST_PT(disk_line_end, vex::forward),
      DRIVE_TO_POINT_FAST_PT(start_point, vex::reverse),
      STOP_INTAKE,
      TURN_TO_AND_DRIVE_TO_FAST(pre_3_stack, 2.0),
      TURN_TO_POINT(goal_point),
      SHOOT_3(1500),
      START_INTAKE,
      TURN_TO_AND_DRIVE_TO_FAST(post_3_stack, 2.0),
      STOP_INTAKE,
      TURN_TO_POINT(goal_point)->withTimeout(2.0),
      SHOOT_3(1500),
      TURN_TO_AND_DRIVE_TO_FAST(pre_roller_pt, 2.0),
      TURN_TO_HEADING(270)->withTimeout(2.0),
      //  DO LE ROLLERS
  });
  return lsdl;

#undef point_t
}

// Skills rollers last

#define INTAKE_UP (new FunctionCommand([]() {intake_up(); return true; }))
#define INTAKE_DOWN (new FunctionCommand([]() {intake_down(); return true; }))
CommandController skills_rollers_last()
{
  auto_tmr.reset();
  flap_down();
  CommandController srl;
  srl.add({
              new FlapDownCommand(),

              // preload+1
              new OdomSetPosition(odometry_sys, {.x = 30.655172, .y = 13.034482, .rot = -180.0}),

              SPIN_FW_AT(3000),
              START_INTAKE,
              DRIVE_TO_POINT_FAST(12.551724, 13.5172415, fwd),
              TURN_TO_HEADING(90.0),
              STOP_INTAKE,
              DRIVE_TO_POINT_FAST(12.310345, 51.413795, fwd),
              TURN_TO_HEADING(89.5),
              SHOOT_3(SHOT_SPACING), // preload+1

              // 3 stack on line
              INTAKE_UP,
              SPIN_FW_AT(2750),
              TURN_TO_HEADING(-30.0),
              START_INTAKE,
              (new FunctionCommand([]()
                                   {
        auto dropper = [](){
          vexDelay(475);
          intake_down();
          return 0;
        };
        vex::thread mine(dropper);
        return true; })),

              DRIVE_TO_POINT_FAST(46.58621, 24.137932, fwd),
              new DelayCommand(500),
              TURN_TO_HEADING(0.0),
              STOP_INTAKE,

              SHOOT_3(SHOT_SPACING), // 3 stack on line

              // 3 stack not on line
              SPIN_FW_AT(2650),
              TURN_TO_HEADING(45.0),
              INTAKE_UP,
              START_INTAKE,

              DRIVE_TO_POINT_FAST(69.27586, 50.03448, fwd),
              (new FunctionCommand([]()
                                   {
        auto dropper = [](){
          vexDelay(275);
          intake_down();
          return 0;
        };
        vex::thread mine(dropper);
        return true; })),

              new DelayCommand(500),

              TURN_TO_HEADING(-25.0),
              // new FlapDownCommand(),
              STOP_INTAKE,

              SHOOT_3(SHOT_SPACING), // 3 stack not on line

              // side 3 in a row
              SPIN_FW_AT(2900),
              START_INTAKE,

              TURN_TO_HEADING(-320.0),
              DRIVE_TO_POINT_FAST(109.586205, 95.31035, fwd),
              TURN_TO_HEADING(-65.0),
              DRIVE_TO_POINT_FAST(128.06897, 61.5, fwd),
              TURN_TO_HEADING(-452.0),
              SHOOT_3(SHOT_SPACING), // side 3 in a row
                                     //
                                     // barrier side 1

              SPIN_FW_AT(2900),
              TURN_TO_HEADING(-175.0), // lesser to hit barrier more
              START_INTAKE,
              DRIVE_TO_POINT_FAST(82.0, 59.2, fwd),
              TURN_TO_HEADING(-39.0),

              STOP_INTAKE,
              SHOOT_3(SHOT_SPACING),

              // barrier side 2
              SPIN_FW_AT(2900),
              TURN_TO_HEADING(-80.0),
              START_INTAKE,
              DRIVE_TO_POINT_FAST(87.6, 18.6, fwd),
              TURN_TO_HEADING(11.0),
              DRIVE_FORWARD_FAST(8, rev),

              STOP_INTAKE,
              SHOOT_3(SHOT_SPACING),

              //
              // center line row of 3
              SPIN_FW_AT(3200),
              TURN_TO_HEADING(125.0),
              START_INTAKE,
              DRIVE_TO_POINT_FAST(60.48276, 64.344826, fwd),
              TURN_TO_HEADING(225.0),
              DRIVE_TO_POINT_FAST(26.034483, 29.758621, fwd),
              TURN_TO_HEADING(355.0),
              STOP_INTAKE,
              SHOOT_3(SHOT_SPACING), // center line row of 3

              new FunctionCommand([]()
                                  {printf("========  Before rollers at %f sec ========", auto_tmr.value());return true; }),
              TURN_TO_HEADING(270.0),

              new FunctionCommand([]()
                                  {
                            double seconds_left = (60.0-auto_tmr.value());
                            double needed = 5.0;
                            if (seconds_left > needed){
                              CommandController rollers;
                              rollers.add(
                                new SpinRollerCommand({0,0 ,0 }), 4.0); 
                              rollers.run();
                            }
                          return true; }),

              // endgaming
              new EndgameCommand(endgame_solenoid),

              DRIVE_FORWARD_FAST(4, fwd),
          },
          3.0);
  return srl;
}

// DRIVE_TO_POINT_FAST(28.4, 12.9, fwd),
// new OdomSetPosition(odometry_sys, {.x = 28.4, .y = 12.9, .rot = 270}),
// DRIVE_FORWARD_FAST(5, rev),
// DRIVE_TO_POINT_FAST(27.4, 12.9, fwd),
// new OdomSetPosition(odometry_sys, {.x = 28.4, .y = 12.9, .rot = 270}),
// DRIVE_FORWARD_FAST(16, rev),
// TURN_TO_HEADING(228.0),
//}

CommandController disk_rush_auto()
{
  flap_down();
  left_motors.setStopping(vex::brake);
  right_motors.setStopping(vex::brake);
  CommandController dra;
  // auto x = new OdomSetPosition()
  dra.add({
              new OdomSetPosition(odometry_sys, {.x = 53.896553, .y = 17.068966, .rot = 135.0}),
              START_INTAKE,
              new FlapDownCommand(),
              SPIN_FW_AT(3450),
              INTAKE_UP,
              (new FunctionCommand([]()
                                   {
        auto dropper = [](){
          vexDelay(700);
          intake_down();
          return 0;
        };
        vex::thread mine(dropper);
        return true; })),

              DRIVE_TO_POINT_FAST(37.482758, 34.448277, fwd),
              INTAKE_DOWN,
              new DelayCommand(750),
              DRIVE_TO_POINT_FAST(41.27586, 30.896551, rev),
              TURN_TO_HEADING(105.0),

              // AUTO_AIM,                     FIRST SHOT
              SHOOT_3(SHOT_SPACING),
              SPIN_FW_AT(3300),

              START_INTAKE,
              TURN_TO_HEADING(25.0),
              INTAKE_UP,
              (new FunctionCommand([]()
                                   {
        auto dropper = [](){
          vexDelay(450);
          intake_down();
          return 0;
        };

        
        vex::thread mine(dropper);


        return true; })),
              DRIVE_TO_POINT_FAST(56.413795, 42.5, fwd),
              INTAKE_DOWN,
              DRIVE_FORWARD_FAST(3, fwd),

              TURN_TO_HEADING(120.0),
              INTAKE_DOWN,

              // AUTO_AIM,                      SECOND SHOT
              SHOOT_3(SHOT_SPACING + 200),

              SPIN_FW_AT(3450),

              // DRIVE_TO_POINT_FAST(85.2931, 14.0, rev),
              //
              // TURN_TO_HEADING(90),
              //
              // START_INTAKE,
              // DRIVE_TO_POINT_SLOW(80.55172, 49.482758, fwd),
              // DRIVE_TO_POINT_FAST(79.06897, 14.0, rev),
              // TURN_TO_HEADING(120.0),
              // DRIVE_TO_POINT_FAST(62.275864, 45.37931, fwd),
              //
              // TURN_TO_HEADING(121.0),
              // AUTO_AIM,                      THIRD SHOT
              // SHOOT_3(SHOT_SPACING),
              //
              START_INTAKE,
              TURN_TO_HEADING(225.0),
              DRIVE_TO_POINT_SLOW(31.9, 18.0, fwd),
              DRIVE_FORWARD_FAST(3, fwd),
              DRIVE_FORWARD_FAST(3, rev),
              TURN_TO_HEADING(270.0),
              //

              (new SpinRollerCommand({0, 0, 0}))->withTimeout(10),

              // SPIN_FW_AT(3500),

              // TURN_TO_HEADING(98.0),
              // AUTO_AIM,
              // SHOOT_3(SHOT_SPACING),

              // 225
              // 31.9, 18.0
              // 270

              // bonk 2
          },
          3.0);
  for (int i = 0; i < num_roller_fallback; i++)
  {
    dra.add({
        DRIVE_FORWARD_FAST(8, fwd),
        new OdomSetPosition(odometry_sys, {.x = 31.5, .y = 10.5, .rot = 90}),
        DRIVE_FORWARD_FAST(5, rev),
    });
  }

  return dra;
}

CommandController only_roller_auto()
{
  CommandController only_r;
  for (int i = 0; i < num_roller_fallback; i++)
  {
    only_r.add({
        DRIVE_FORWARD_FAST(8, fwd),
        new OdomSetPosition(odometry_sys, {.x = 31.5, .y = 10.5, .rot = 90}),
        DRIVE_FORWARD_FAST(5, rev),
    });
  }

  only_r.add({
                 DRIVE_FORWARD_FAST(2, rev),
                 TURN_TO_HEADING(-90),
             },
             3.0);
  return only_r;
}

CommandController only_roller_auto_fancy()
{
  CommandController only_r;
  only_r.add(new SpinRollerCommand({.x = 32.3, .y = 5.6, .rot = -90}), 35);
  only_r.add({
                 DRIVE_FORWARD_FAST(2, rev),
                 TURN_TO_HEADING(-90),
             },
             3.0);
  return only_r;
}

CommandController safe_auto()
{
  CommandController sa;
  flap_down();
  sa.add({
             new OdomSetPosition(odometry_sys, {.x = 84.48276, .y = 16.172415, .rot = 90.0}),
             SPIN_FW_AT(3500),
             START_INTAKE,
             DRIVE_TO_POINT_SLOW(83.75862, 42.0, fwd),
             DRIVE_TO_POINT_FAST(84.48276, 16.655172, rev),
             TURN_TO_HEADING(120.0),
             DRIVE_TO_POINT_FAST(69.586205, 43.689655, fwd),

             TURN_TO_HEADING(121.0),
             SHOOT_3(800),

             TURN_TO_HEADING(228.0),

             START_INTAKE,
             INTAKE_UP,
             SPIN_FW_AT(3600),

             (new FunctionCommand([]()
                                  {
        auto dropper = [](){
          vexDelay(850);
          intake_down();
          return 0;
        };
        vex::thread mine(dropper);

        return true; })),

             DRIVE_TO_POINT_SLOW(55.517242, 35.827587, fwd),
             new DelayCommand(1000),
             TURN_TO_HEADING(108.0),
             SHOOT_3(800),
            //  DRIVE_TO_POINT_SLOW(34.75862, 18.689655, fwd),
            //  TURN_TO_HEADING(-90.0),
            //  new SpinRollerCommand({0, 0, 0}),
         },
         3.0);
         
         return sa;
}
