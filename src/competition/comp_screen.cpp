#include "competition/comp_screen.h"

void page_one(vex::brain::lcd &screen, int x, int y, int width, int height, bool first_run)
{
    screen.setFillColor(vex::black);
    screen.setPenColor(vex::black);
    screen.drawRectangle(x, y, width, height);
    // Left Half - motors

    // Column header
    draw_mot_header(screen, x, 0, width / 2);

    const int line_height = 20;
    int row_num = 1;
    // motors and their information
    for (auto const &name_and_motor : motor_names)
    {
        draw_mot_stats(screen, x, row_num * line_height, width / 2, name_and_motor.first.c_str(), name_and_motor.second);
        row_num++;
    }

    // Battery stuff
    double bat_voltage = Brain.Battery.voltage();
    double bat_percentage = Brain.Battery.capacity();
    draw_battery_stats(screen, x, 180, bat_voltage, bat_percentage);

    // Program Type

    // Right Top
    screen.drawImageFromBuffer(splash_little, 40 + width / 2, 0, width / 2, 100);
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

    setpt.draw(x, y, width, height - 40);
    rpm.draw(x, y, width, height - 40);
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
