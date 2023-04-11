#include "competition/comp_screen.h"

void page_one(vex::brain::lcd &screen, int x, int y, int width, int height, bool first_run)
{
    screen.setFillColor(vex::black);
    screen.setPenColor(vex::black);
    screen.drawRectangle(x, y, width, height);
    // Left Half - motors

    // Column header
    draw_mot_header(screen, x, 0, width / 2);

    static int animation_tick = 0;
    animation_tick++;

    const int line_height = 20;
    int row_num = 1;
    // motors and their information
    for (auto const &name_and_motor : motor_names)
    {
        draw_mot_stats(screen, x, row_num * line_height, width / 2, name_and_motor.first.c_str(), name_and_motor.second, animation_tick);
        row_num++;
    }

    for (auto const &name_and_device : device_names)
    {
        auto device = name_and_device.second;
        draw_dev_stats(screen, x, row_num * line_height, width / 2, name_and_device.first.c_str(), name_and_device.second, animation_tick);
        row_num++;
    }

    // Battery stuff
    double bat_voltage = Brain.Battery.voltage();
    double bat_percentage = Brain.Battery.capacity();
    draw_battery_stats(screen, x, row_num * line_height, bat_voltage, bat_percentage);

    // Program Type

    // Right Top
    screen.drawImageFromBuffer(splash_little, 44 + width / 2, 0, splash_little_width, 100);
}

void page_two(vex::brain::lcd &screen, int x, int y, int width, int height, bool first_run)
{
    screen.setPenColor(vex::black);
    screen.setFillColor(vex::black);

    screen.drawRectangle(x, y, width, height);
    position_t pos = odometry_sys.get_position();
    screen.setFont(vex::mono20);
    screen.setPenColor(vex::white);
    screen.setFillColor(vex::transparent);
    screen.printAt(100, 100, "Pos: (%.2f, %.2f) : %.2f", pos.x, pos.y, pos.rot);
    screen.printAt(100, 140, "FW setpt: %.2f\t  %.2f", flywheel_sys.getDesiredRPM(), flywheel_sys.getRPM());
}

void page_three(vex::brain::lcd &screen, int x, int y, int width, int height, bool first_run)
{
    static GraphDrawer setpt(screen, 30, "time", "rpm", vex::red, true, 0, 4000);
    static GraphDrawer rpm(screen, 30, "time", "rpm", vex::blue, true, 0, 4000);
    static bool func_initialized = false;
    static vex::timer tmr;
    if (!func_initialized)
    {
        tmr.reset();
        func_initialized = true;
    }
    screen.clearScreen();
    double t = tmr.time(seconds);
    setpt.add_sample({.x = t, .y = flywheel_sys.getDesiredRPM()});
    rpm.add_sample({.x = t, .y = flywheel_sys.getRPM()});

    setpt.draw(x + 4, y, width - 8, height - 40);
    rpm.draw(x + 4, y, width - 8, height - 40);
    screen.printAt(x + width / 2, y + height - 30, "setpt: %.0f, rpm: %.0f", flywheel_sys.getDesiredRPM(), flywheel_sys.getRPM());
}

static bool inRectangle(int x, int y, int width, int height, int test_x, int test_y)
{
    // too left / low
    if (test_x < x || test_y < y)
    {
        return false;
    }

    // too right / high
    if (test_x > x + width || test_y > y + height)
    {
        return false;
    }

    // gotta be in
    return true;
}

void page_four(vex::brain::lcd &screen, int x, int y, int width, int height, bool first_run)
{
    screen.setPenColor(vex::black);
    screen.drawRectangle(x, y, width, height, vex::black);
    int padding = 20;
    int button_width = (width / 2) - (padding * 2);
    int button_height = (height / 2) - (padding * 2);
    // Color Picker
    int bluex = x + padding;
    int bluey = y + padding;
    int redx = (x + width / 2) + padding;
    int redy = y + padding;

    screen.setPenColor(vex::white);
    screen.setFont(mono20);
    // Blue Button
    screen.setFillColor(vex::blue);
    if (target_red == false)
    {
        screen.setPenWidth(4);
    }
    else
    {
        screen.setPenWidth(1);
    }
    screen.drawRectangle(bluex, bluey, button_width, button_height);
    screen.setPenWidth(1);
    screen.setPenColor(vex::black);

    screen.printAt(bluex + padding, bluey + padding + padding, "Blue");

    // Red Button
    screen.setFillColor(vex::red);
    if (target_red == true)
    {
        screen.setPenWidth(4);
    }
    else
    {
        screen.setPenWidth(1);
    }
    screen.setPenColor(vex::white);
    screen.drawRectangle(redx, redy, button_width, button_height);
    screen.setPenWidth(1);
    screen.setPenColor(vex::black);

    screen.printAt(redx + padding, redy + padding + padding, "Red");

    // Enabler
    int onx = x + padding;
    int ony = (y + height / 2) + padding;
    int offx = (x + width / 2) + padding;
    int offy = (y + height / 2) + padding;

    screen.setPenColor(vex::white);

    // On
    screen.setFillColor(vex::color(50, 168, 82));
    if (vision_enabled == true)
    {
        screen.setPenWidth(4);
    }
    else
    {
        screen.setPenWidth(1);
    }
    screen.drawRectangle(onx, ony, button_width, button_height);
    screen.setPenWidth(1);
    screen.setPenColor(vex::black);
    screen.printAt(onx + padding, ony + padding + padding, "On");

    // Off
    screen.setFillColor(vex::red);
    if (vision_enabled == false)
    {
        screen.setPenWidth(4);
    }
    else
    {
        screen.setPenWidth(1);
    }
    screen.setPenColor(vex::white);
    screen.drawRectangle(offx, offy, button_width, button_height);
    screen.setPenWidth(1);
    screen.setPenColor(vex::black);
    screen.printAt(offx + padding, offy + padding + padding, "Off");

    if (screen.pressing())
    {
        int test_x = screen.xPosition();
        int test_y = screen.yPosition();
        // clicked on blue
        if (inRectangle(bluex, bluey, button_width, button_height, test_x, test_y))
        {
            target_red = false;
        }
        // clicked on red
        if (inRectangle(redx, redy, button_width, button_height, test_x, test_y))
        {
            target_red = true;
        }

        // clicked on on
        if (inRectangle(onx, ony, button_width, button_height, test_x, test_y))
        {
            vision_enabled = true;
        }
        // clicked on off
        if (inRectangle(offx, offy, button_width, button_height, test_x, test_y))
        {
            vision_enabled = false;
        }
    }
}

bool keep_collecting = true;
const float paddingx = 50;
const float paddingy = 10;
Vector2D::point_t offset_from_corner_px = {20, 20};
const float field_width_on_screen = 200;
const float tile_width_in = 23.5;
const float field_width_inches = 6 * tile_width_in;
const float inch_to_pixel = field_width_on_screen / field_width_inches;
const float rob_width = 16;
const float rob_height = 16;

Vector2D::point_t field_to_screen(Vector2D::point_t p)
{
    Vector2D::point_t ret = {.x = p.y * inch_to_pixel, .y = p.x * inch_to_pixel};
    ret = ret + offset_from_corner_px;
    return ret;
}

int pixel_x(Vector2D::point_t p)
{
    return (int)field_to_screen(p).x;
}
int pixel_y(Vector2D::point_t p)
{
    return (int)field_to_screen(p).y;
}
void drawInchLine(vex::brain::lcd &screen, Vector2D::point_t a, Vector2D::point_t b)
{
    screen.drawLine(pixel_x(a), pixel_y(a), pixel_x(b), pixel_y(b));
}

Vector2D::point_t rotate2D(Vector2D::point_t p, double dir_radians)
{
    double c = cos(dir_radians);
    double s = sin(dir_radians);
    // clang-format off
    return {.x = p.x * c - p.y * s
           ,.y = p.x * s + p.y * c };
    // clang-format on
}
void page_five(vex::brain::lcd &screen, int x, int y, int width, int height, bool first_run)
{
#define point_t Vector2D::point_t
    const int num_points = 40;
    static int points_index = 0;
    static std::array<point_t, num_points> points = {};
    auto add_point_to_path = [](point_t pt)
    {
        points[points_index] = pt;
        points_index = (points_index + 1) % num_points;
    };
    auto get_point_from_path = [](int index)
    {
        return points[(index + points_index) % num_points];
    };

    // count up and only save points on 0
    static int count = 0;
    const int save_every = 5; // iterations

    position_t pose = odometry_sys.get_position();

    bool isZeroPos = pose.x==odometry_sys.zero_pos.x && pose.y==odometry_sys.zero_pos.y && pose.rot==odometry_sys.zero_pos.rot;

    if (keep_collecting && count == 0 && !isZeroPos)
    {
        add_point_to_path({pose.x, pose.y});
    }

    offset_from_corner_px = {x + paddingx, y + paddingy};
    point_t origin = {0, 0};
    point_t plus_y = {0, field_width_inches};
    point_t plus_x = {field_width_inches, 0};
    point_t plus_both = {field_width_inches, field_width_inches};

    point_t unit_y = {0, tile_width_in};
    point_t unit_x = {tile_width_in, 0};

    point_t red_goal = {17, field_width_inches - 17};
    point_t blue_goal = {field_width_inches - 17, 17};
    float goal_diam = 18;

    point_t blue_barrier_point = {2 * tile_width_in, field_width_inches - 2 * tile_width_in};
    point_t red_barrier_point = {field_width_inches - 2 * tile_width_in, 2 * tile_width_in};

    // drawing
    screen.clearScreen(vex::white);
    screen.setPenColor(vex::black);
    screen.setPenWidth(2);
    // barriers
    drawInchLine(screen, origin, plus_y);
    drawInchLine(screen, origin, plus_x);
    drawInchLine(screen, plus_x, plus_both);
    drawInchLine(screen, plus_y, plus_both);

    screen.setPenWidth(1);
    // between tile lines
    for (int x = 1; x <= 5; x++)
    {
        drawInchLine(screen, {x * tile_width_in, 0}, {x * tile_width_in, field_width_inches});
    }
    for (int y = 1; y <= 5; y++)
    {
        drawInchLine(screen, {0, y * tile_width_in}, {field_width_inches, y * tile_width_in});
    }
    // red goal
    screen.setFillColor(vex::red);
    screen.setPenColor(vex::red);
    screen.drawCircle(pixel_x(red_goal), pixel_y(red_goal), goal_diam / 2);

    // blue goal
    screen.setFillColor(vex::blue);
    screen.setPenColor(vex::blue);
    screen.drawCircle(pixel_x(blue_goal), pixel_y(blue_goal), goal_diam / 2);

    screen.setPenWidth(4);
    // red barrier
    screen.setFillColor(vex::red);
    screen.setPenColor(vex::red);
    drawInchLine(screen, red_barrier_point, red_barrier_point + unit_x);
    drawInchLine(screen, red_barrier_point, red_barrier_point - unit_y);
    // blue barrier
    screen.setFillColor(vex::blue);
    screen.setPenColor(vex::blue);
    drawInchLine(screen, blue_barrier_point, blue_barrier_point + unit_y);
    drawInchLine(screen, blue_barrier_point, blue_barrier_point - unit_x);

    // Robot
    screen.setPenWidth(3);
    screen.setFillColor(vex::purple);
    screen.setPenColor(vex::purple);
    point_t pos = {pose.x, pose.y};
    point_t fl = (rotate2D({-rob_width / 2, -rob_height / 2}, deg2rad(pose.rot)) + pos);
    point_t fr = (rotate2D({rob_width / 2, -rob_height / 2}, deg2rad(pose.rot)) + pos);
    point_t bl = (rotate2D({-rob_width / 2, rob_height / 2}, deg2rad(pose.rot)) + pos);
    point_t br = (rotate2D({rob_width / 2, rob_height / 2}, deg2rad(pose.rot)) + pos);
    // outline
    drawInchLine(screen, fl, fr);
    drawInchLine(screen, fr, br);
    drawInchLine(screen, br, bl);
    drawInchLine(screen, bl, fl);
    // draw direction indicator
    point_t front_pos = {(fr + br).x / 2.0, (fr + br).y / 2.0}; // we should get lerp and division by scalar on point_t. lerp is so cool
    point_t fronter_pos = {front_pos.x + cos(deg2rad(pose.rot)) * 100, front_pos.y + sin(deg2rad(pose.rot)) * 100};

    drawInchLine(screen, front_pos, fronter_pos);

    screen.setFillColor(vex::black);
    screen.setPenColor(vex::black);
    screen.drawCircle(pixel_x(front_pos), pixel_y(front_pos), 4);

    // Save last 10 reading
    // Only update that list once every 1/4 second or so
    // draw path that it took
    // maybe dont update point if we're now stopped. that way we can walk over and see what it did yk
    // things to ponder...
    // perhaps a replay button

    //
    screen.setFillColor(white);
    screen.setPenColor(black);
    screen.setFont(mono20);

    screen.printAt(300, 80, "(%.1f, %.1f)", pose.x, pose.y);
    screen.printAt(300, 100, "rot = %.f deg", pose.rot);
    screen.printAt(300, 120, "%.3f : %.3f", left_enc.position(rotationUnits::rev), right_enc.position(rotationUnits::rev));

    // Draw Path
    screen.setFillColor(vex::orange);
    screen.setPenColor(vex::orange);
    for (int i = 0; i < num_points - 1; i++)
    {
        auto p1 = get_point_from_path(i);
        auto p2 = get_point_from_path(i + 1);
        drawInchLine(screen, p1, p2);
    }

    count++;
    count %= save_every;

#undef point_t
}

// num fall back rollers
void page_six(vex::brain::lcd &screen, int x, int y, int width, int height, bool first_run)
{
    screen.setFillColor(vex::black);
    screen.setPenColor(vex::black);
    screen.drawRectangle(x, y, width, height);

    screen.setPenColor(vex::color::white);
    screen.setFont(mono60);
    screen.printAt(x + width / 2, y + height / 2, "%d", num_roller_fallback);
    screen.setFont(mono30);
    screen.printAt(x + width / 2 - (screen.getStringWidth("Roller Hits") / 2), y + 190, "Roller Hits");

    int test_x = screen.xPosition();
    int test_y = screen.yPosition();

    int mod_width = 60;

    screen.setPenColor(vex::color::black);
    screen.setFillColor(vex::color::black);
    screen.drawRectangle(x, y, mod_width, height);
    screen.drawRectangle(x + width - mod_width, y, mod_width, height);
    screen.drawLine(x + mod_width, 0, x + mod_width, height);
    screen.drawLine(x + width - mod_width, 0, x + width - mod_width, height);

    screen.setFillColor(vex::black);
    screen.setPenColor(vex::color::white);
    screen.setFont(mono60);

    screen.printAt(x + (mod_width / 4), y + height / 2, "-");
    screen.printAt(x + width - mod_width + (mod_width / 4), y + height / 2, "+");

    static bool was_pressing = false;
    bool pressing = screen.pressing();
    if (pressing & !was_pressing)
    {
        if (inRectangle(x, y, x + mod_width, height, test_x, test_y))
        {
            num_roller_fallback--;
        }
        if (inRectangle(x + width - mod_width, y, mod_width, height, test_x, test_y))
        {
            num_roller_fallback++;
        }
    }
    was_pressing = pressing;
}
