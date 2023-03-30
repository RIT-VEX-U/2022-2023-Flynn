#include "competition/autonomous_flynn.h"
#include "vision.h"
#include "tuning.h"
#include "../core/include/splash.h"

#define CALIBRATE_IMU()       \
  while (imu.isCalibrating()) \
  {                           \
  }

const double TURN_SPEED = 0.6;
const double INTAKE_VOLT =  12;
const double SHOOTING_RPM = 3200;
const double SINGLE_SHOT_TIME  = 0.2;
const double SINGLE_SHOT_VOLT =  6;
const double SINGLE_SHOT_RECOVER_DELAY_MS = 1000;
int glbl_vision_center = 135;

#define DRIVE_TO_POINT_SLOW(x, y, dir) (new DriveToPointCommand(drive_sys, drive_slow_mprofile, x, y, directionType::dir))
#define DRIVE_TO_POINT_SLOW_PT(pt, dir) (new DriveToPointCommand(drive_sys, drive_slow_mprofile, pt, directionType::dir))

#define DRIVE_TO_POINT_FAST(x, y, dir) (new DriveToPointCommand(drive_sys, drive_fast_mprofile, x, y, directionType::dir))
#define DRIVE_TO_POINT_FAST_PT(pt, dir) (new DriveToPointCommand(drive_sys, drive_fast_mprofile, pt, dir))

#define DRIVE_FORWARD_FAST(in, dir) (new DriveForwardCommand(drive_sys, drive_fast_mprofile, in, directionType::dir))
#define TURN_TO_HEADING(heading_deg) (new TurnToHeadingCommand(drive_sys, *config.turn_feedback, heading_deg, TURN_SPEED))

#define TURN_TO_POINT(point) (new TurnToPointCommand(drive_sys, odometry_sys, *config.turn_feedback, point))

#define START_INTAKE (new StartIntakeCommand(intake, INTAKE_VOLT))
#define STOP_INTAKE (new StopIntakeCommand(intake))


#define ShootDisk (new ShootCommand(intake, SINGLE_SHOT_TIME, SINGLE_SHOT_VOLT))
#define SpinFWAt(rpm) (new SpinRPMCommand(flywheel_sys, rpm))

#define PrintOdom (new PrintOdomCommand(odometry_sys))
#define PrintOdomContinous (new PrintOdomContinousCommand(odometry_sys))

const double TRI_SHOT_TIME = 1.0;
const double TRI_SHOT_VOLT = 2;
const double TRI_SHOT_RECOVER_DELAY_MS = 200;

#define AUTO_AIM (new VisionAimCommand(true, glbl_vision_center, 10))
#define WAIT_FOR_FLYWHEEL (new WaitUntilUpToSpeedCommand(flywheel_sys, 150))
#define TRI_SHOT_DISK (new ShootCommand(intake, TRI_SHOT_TIME, TRI_SHOT_VOLT))

#define SHOOT_DISK (new ShootCommand(intake, SINGLE_SHOT_TIME, SINGLE_SHOT_VOLT))
#define CLEAR_DISKS (new ShootCommand(intake, .75, SINGLE_SHOT_VOLT))

static void add_single_shot_cmd(CommandController &controller, double vis_timeout = 1.0)
{
  controller.add(WAIT_FOR_FLYWHEEL, vis_timeout);
  if (vis_timeout == 0.0)
    controller.add(AUTO_AIM);
  else
    controller.add(AUTO_AIM, vis_timeout);
  controller.add(ShootDisk);
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
    position_t pos = odometry_sys.get_position();
    printf("%.2f, %.2f, %.2f\n", pos.x, pos.y, pos.rot);
    vexDelay(1000);
  }
  return 0;
}

void test_stuff()
{
  CALIBRATE_IMU();

  vex::task odom_print(print_odom);

  CommandController mine = auto_loader_side();
  mine.run();
  return;
  //
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
                                   { intake.spin(reverse, 12, volt); }); // Intake
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

  int i = 0;

  VisionAimCommand visaim(false, 145, 5);

  timer loop_timer;
  loop_timer.reset();
  double delay_time = 0.0;
  double loop_time = 0.0;
  // Periodic
  while (true)
  {
    i++;

    // ========== DRIVING CONTROLS ==========
    if (main_controller.ButtonA.pressing())
      visaim.run();
    else
      drive_sys.drive_arcade(main_controller.Axis3.position(pct) / 100.0, main_controller.Axis1.position(pct) / 300.0);

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
  flapup_solenoid.set(false);
  position_t start_pos = position_t{.x = 36.0, .y = 12.2, .rot = -90};

  position_t roller_in_pos = {.x = 36.0, .y = 4.16, .rot = -90};

  CommandController lss;
  lss.add(new OdomSetPosition(odometry_sys, start_pos)); // #1

  // spin -90 degree roller
  lss.add(new SpinRollerCommand(roller_in_pos), 5.0);
  lss.add(DRIVE_FORWARD_FAST(6, rev));

  Vector2D::point_t corner_disk_point = {.x = 10, .y = 12};

  // intake corner disk
  lss.add({
      TURN_TO_POINT(corner_disk_point)->withTimeout(1.5),                 // #5
      START_INTAKE,                                                      // #6
      DRIVE_TO_POINT_SLOW_PT(corner_disk_point, fwd)->withTimeout(1.5), // #7
      DRIVE_FORWARD_FAST(4, rev)->withTimeout(1.5),                     // #8
  });

  Vector2D::point_t shoot_point = {.x = 10.5, .y = 93};

  // align to 180 degree roller
  lss.add(new SpinRPMCommand(flywheel_sys, 2900)); // #21

  Vector2D::point_t roller_out_pos2 = {.x = 14, .y = 33};
  position_t roller_in_pos2 = {.x = 4.20, .y = 33, .rot = 180};

  lss.add({
      TURN_TO_POINT(roller_out_pos2)->withTimeout(1.5),
      DRIVE_TO_POINT_FAST_PT(roller_out_pos2, fwd)->withTimeout(1.5), // #12
      TURN_TO_HEADING(180)->withTimeout(1.5),                           // #13

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
  lss.add(ShootDisk);
  lss.add_delay(1000);

  lss.add(WAIT_FOR_FLYWHEEL, 1.0);
  lss.add(ShootDisk);
  lss.add_delay(1000);

  lss.add(WAIT_FOR_FLYWHEEL, 1.0);
  lss.add(CLEAR_DISKS);

  // lss.add(PrintOdomContinous); /// the guy youre looking for =================================================================================> :)

  // DISKS AGAINST L PIECE
  Vector2D::point_t disk_pos1 = {.x = 24.0, .y = 81.5};
  Vector2D::point_t disk_pos2 = {.x = 32.0, .y = 81.5};
  Vector2D::point_t disk_pos3 = {.x = 42.0, .y = 81.5};

  Vector2D::point_t disk_prep_pos1 = {.x = 25, .y = 70};
  Vector2D::point_t disk_prep_pos2 = {.x = 28, .y = 70};
  Vector2D::point_t disk_prep_pos3 = {.x = 33, .y = 70};

  // disks against L piece
  lss.add({
      // farthest
      DRIVE_TO_POINT_FAST_PT(disk_prep_pos1, rev)->withTimeout(2.0), // #27
      START_INTAKE,                                                   // #26
      TURN_TO_POINT(disk_pos1)->withTimeout(1.5),                      // #28
      DRIVE_TO_POINT_SLOW_PT(disk_pos1, fwd)->withTimeout(2.0),               // #29

      // middle
      DRIVE_TO_POINT_FAST_PT(disk_prep_pos2, rev)->withTimeout(2.0), // #30
      TURN_TO_POINT(disk_pos2)->withTimeout(1.5),                      // #31
      DRIVE_TO_POINT_SLOW_PT(disk_pos2, fwd)->withTimeout(2.0),               // #32

      // closest disk
      DRIVE_TO_POINT_FAST_PT(disk_prep_pos3, rev)->withTimeout(2.0), // #33
      TURN_TO_POINT(disk_pos3)->withTimeout(1.5),                      // #34
      DRIVE_TO_POINT_SLOW_PT(disk_pos3, fwd)->withTimeout(2.0),               // #35
      DRIVE_TO_POINT_FAST_PT(disk_prep_pos3, rev)->withTimeout(2.0), // #36
  });
  lss.add(new SpinRPMCommand(flywheel_sys, 3100)); // #40

  // Vector2D::point_t pre_shoot_point_other = {.x = 10.5, .y = 70};
  // Vector2D::point_t shoot_point_other = {.x = 10.5, .y = 90};

  // lss.add(TURN_TO_POINT(pre_shoot_point_other), 1.5);        // #37
  // lss.add(DriveToPointFastPt(pre_shoot_point_other), 4.0); // #38
  // lss.add(TURN_TO_POINT(shoot_point_other), 1.5);            // #37
  // lss.add(DriveToPointFastPt(shoot_point_other), 4.0);     // #38
  lss.add(new FlapDownCommand());

  // lss.add(TURN_TO_HEADING(75), 0.5); // #39
  Vector2D::point_t hoop_point_early = {.x = 17, .y = 123};
  lss.add(TURN_TO_POINT(hoop_point_early));

  add_single_shot_cmd(lss);
  add_single_shot_cmd(lss);
  add_single_shot_cmd(lss);

  // Arrow 3 -------------------------

  Vector2D::point_t start_of_line = {.x = 34.5, .y = 49};
  Vector2D::point_t end_of_line = {.x = 65, .y = 82};
  Vector2D::point_t out_of_way_point = {.x = 73, .y = 118};

  lss.add(new SpinRPMCommand(flywheel_sys, 3200)); // #40

  // go to line and collect line
  lss.add({
      START_INTAKE,
      // Start of line
      TURN_TO_POINT(start_of_line)->withTimeout(2.0), // #39
      // START_INTAKE,                       // #40
      DRIVE_TO_POINT_FAST_PT(start_of_line, fwd)->withTimeout(2.0), // #41

      // Drive to End of line
      TURN_TO_POINT(end_of_line)->withTimeout(2.0),                 // #43
      DRIVE_TO_POINT_FAST_PT(end_of_line, fwd)->withTimeout(2.0), // $44

  });

  Vector2D::point_t hoop_point = {.x = 17, .y = 113};
  lss.add(TURN_TO_POINT(hoop_point), 1.0);
  lss.add(DRIVE_FORWARD_FAST(6, rev));

  lss.add(new FlapDownCommand());
  add_single_shot_cmd(lss);
  add_single_shot_cmd(lss);
  add_single_shot_cmd(lss);

  // Arrow 4 -------------------------
  Vector2D::point_t endgame_point = {.x = 116.36, .y = 106.23};

  lss.add(TURN_TO_POINT(endgame_point), 1.0);                 // [measure]
  lss.add(DRIVE_TO_POINT_FAST_PT(endgame_point, fwd), 4.0); //[measure]
  lss.add(TURN_TO_HEADING(48), 3.0);

  lss.add(new EndgameCommand(endgame_solenoid));
  lss.add(PrintOdom);

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

  position_t start_pos = position_t{.x = 36.0, .y = 12.2, .rot = -90};
  position_t roller_in_pos = {.x = 36.0, .y = 5.16, .rot = -90};

  // Rollers =======================================================
  lsa.add(new OdomSetPosition(odometry_sys, start_pos));
  lsa.add(SpinFWAt(3250));
  lsa.add(new SpinRollerCommand(roller_in_pos), 5.0);
  // lsa.add(new OdomSetPosition(odometry_sys, start_pos));
  lsa.add(DRIVE_FORWARD_FAST(2, rev));
  Vector2D::point_t roller_out_pos = {.x = 36, .y = 16.0};
  lsa.add(DRIVE_TO_POINT_FAST_PT(roller_out_pos, rev));

  Vector2D::point_t out_of_way_pos1 = {.x = 79.0, .y = 14.0};
  Vector2D::point_t shoot_point1 = {.x = 65, .y = 50};

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
  lsa.add(ShootDisk);

  // lsa.add(TURN_TO_HEADING(125.6), 2.0);
  lsa.add(AUTO_AIM, 1.0);
  lsa.add(WAIT_FOR_FLYWHEEL, 1.0);
  lsa.add(CLEAR_DISKS);

  auto lerp_point = [](Vector2D::point_t a, Vector2D::point_t b, float t)
  {
    return Vector2D::point_t{.x = a.x * (1 - t) + b.x * t, .y = a.y * (1 - t) + b.y * t};
  };

  Vector2D::point_t pre_stack3_pos = {.x = 70.0, .y = 48.0};
  Vector2D::point_t after_stack3_pos = {.x = 44.0, .y = 20.0};

  Vector2D::point_t mid_stack3_pos = lerp_point(pre_stack3_pos, after_stack3_pos, .6);

  // Vector2D::point_t shoot_point1 = {.x = 64, .y = 51};
  // double goal_pos1_deg = 120.0;

  // Third Shot =======================================================

  lsa.add(SpinFWAt(3550));
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
  lsa.add(ShootDisk);

  lsa.add(AUTO_AIM, 1.0);
  lsa.add(WAIT_FOR_FLYWHEEL, 1.0);
  lsa.add(ShootDisk);

  lsa.add(AUTO_AIM, 1.0);
  lsa.add(WAIT_FOR_FLYWHEEL, 1.0);
  lsa.add(CLEAR_DISKS);
  lsa.add_delay(600);

  Vector2D::point_t disk1_pos = {.x = 84.0, .y = 46.0};
  Vector2D::point_t disk2_pos = {.x = 83.0, .y = 36.0};
  Vector2D::point_t disk3_pos = {.x = 83.0, .y = 27.0};

  Vector2D::point_t lineup_disk2_pos = {.x = 70.0, .y = 41.0};
  Vector2D::point_t lineup_disk3_pos = {.x = 70.0, .y = 32.0};

  // Second Shot =======================================================
  lsa.add(SpinFWAt(3500));
  lsa.add({

      // First
      TURN_TO_POINT(disk1_pos)->withTimeout(2.0),
      START_INTAKE,
      DRIVE_TO_POINT_FAST_PT(disk1_pos, fwd)->withTimeout(2.0),

      // Second
      DRIVE_TO_POINT_FAST_PT(lineup_disk2_pos, rev)->withTimeout(2.0),
      TURN_TO_POINT(disk2_pos)->withTimeout(2.0),
      DRIVE_TO_POINT_FAST_PT(disk2_pos, fwd)->withTimeout(2.0),

      // Third
      DRIVE_TO_POINT_FAST_PT(lineup_disk3_pos, rev)->withTimeout(2.0),
      TURN_TO_POINT(disk3_pos)->withTimeout(2.0),
      DRIVE_TO_POINT_FAST_PT(disk3_pos, fwd)->withTimeout(2.0),
  });

  // go shoot
  //  Vector2D::point_t disk3_pos = {.x = 86.0, .y = 27.0};
  Vector2D::point_t halfway = {.x = 75, .y = 39}; // turn here
  Vector2D::point_t shoot_point2 = {.x = 64, .y = 51};

  lsa.add({
      DRIVE_TO_POINT_FAST_PT(halfway, rev)->withTimeout(2.0),
      STOP_INTAKE,
      TURN_TO_POINT(shoot_point2)->withTimeout(2.0),
      DRIVE_TO_POINT_FAST_PT(shoot_point2, fwd)->withTimeout(2.0),
  });

  lsa.add(TURN_TO_HEADING(130), 2.0);
  lsa.add(AUTO_AIM, 1.0);
  lsa.add(WAIT_FOR_FLYWHEEL, 1.0);
  lsa.add(ShootDisk);

  lsa.add(AUTO_AIM, 1.0);
  lsa.add(WAIT_FOR_FLYWHEEL, 1.0);
  lsa.add(ShootDisk);
  lsa.add(WAIT_FOR_FLYWHEEL, 1.0);

  lsa.add(AUTO_AIM, 1.0);
  lsa.add(CLEAR_DISKS);
  lsa.add_delay(600);

  // Line up to 3 stack
  Vector2D::point_t pre_op_lineup = {.x = 43.6, .y = 24.05};
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
#define SHOOT_3 SHOOT_DISK, new DelayCommand(1000), STOP_INTAKE
#define TURN_TO_AND_DRIVE_TO_FAST(pt, timeout) (TURN_TO_POINT(pt)->withTimeout(timeout)),(DRIVE_TO_POINT_FAST_PT(pt, fwd)->withTimeout(timeout))
CommandController auto_loader_side_disks_last()
{
  #define point_t Vector2D::point_t
  
  point_t goal_point = {15, 125};
  position_t start_point_odom = {.x = 83, .y = 13, .rot = 90.f};
  point_t start_point = {.x = start_point_odom.x, .y = start_point_odom.y};
  point_t disk_line_end = {83, 47};
  point_t pre_3_stack = {65, 45};
  point_t post_3_stack = {52, 30};
  point_t pre_roller_pt = {31, 10};

  CommandController lsdl;
  lsdl.add({
    new OdomSetPosition(odometry_sys, start_point_odom),
    TURN_TO_POINT(goal_point),
    SHOOT_3,
    TURN_TO_POINT(disk_line_end),
    START_INTAKE,
    DRIVE_TO_POINT_FAST_PT(disk_line_end, vex::forward),
    DRIVE_TO_POINT_FAST_PT(start_point, vex::reverse),
    STOP_INTAKE,
    TURN_TO_AND_DRIVE_TO_FAST(pre_3_stack, 2.0),
    TURN_TO_POINT(goal_point),
    SHOOT_3,
    START_INTAKE,
    TURN_TO_AND_DRIVE_TO_FAST(post_3_stack, 2.0),
    STOP_INTAKE,
    TURN_TO_POINT(goal_point),
    SHOOT_3,
    TURN_TO_AND_DRIVE_TO_FAST(pre_roller_pt, 2.0),
    TURN_TO_HEADING(270),
    // DO LE ROLLERS
  });

  #undef point_t
}