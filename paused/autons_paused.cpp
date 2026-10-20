// Paused auton code, moved out of src/main.cpp. NOT compiled.
// To bring something back, paste it into main.cpp.

// ---- grab_pin(): line up, turn, creep forward, close on the distance sensor.
/**
 * Grabs a pin from a known robot pose: (cx, cy) is where the robot's CENTRE
 * is, and `heading` the way it faces, with the claw lined up on the pin
 * (read off the FIELD page).
 *
 *   P1  drive to a line-up spot LINEUP_IN back along `heading` -- the
 *       estimate
 *   P2  turn to `heading` there, with nothing in front of the claw
 *   P3  drive the claw straight forward; the moment the distance sensor
 *       reads GRAB_MM or closer, STOP and close
 *
 * If the sensor never reads GRAB_MM (it goes blind up close), it closes at
 * the end of the drive-in anyway. Auto-clamp is off for all of it, so it
 * cannot fire early and grab the pin by the edge. Returns true if the
 * sensor triggered the close.
 */
bool grab_pin(double cx, double cy, double heading) {
    constexpr double LINEUP_IN = 10;     // start this far back; bigger = longer straight run-in
    constexpr double OVERRUN_IN = 3;     // may drive this far past (cx, cy) looking for the pin
    constexpr int GRAB_MM = 65;          // close here; bigger = closes earlier (45 never triggered)
    constexpr int CREEP_POWER = 25;      // out of 127, while driving in; slower = more time to close (was 35)

    robot::claw::set_armed(false);
    robot::claw::open();

    const double h = heading * M_PI / 180.0;
    const double lx = cx - LINEUP_IN * std::sin(h);
    const double ly = cy - LINEUP_IN * std::cos(h);

    step("P1 drive to estimate");
    chassis.turnToPoint(lx, ly, 1000);
    chassis.moveToPoint(lx, ly, 2500, {.maxSpeed = 50});
    step("P2 turn to face pin");
    chassis.turnToHeading(heading, 1000);
    chassis.waitUntilDone();

    step("P3 claw forward");
    const lemlib::Pose start = chassis.getPose();
    const std::uint32_t t0 = pros::millis();
    bool triggered = false;
    int closest_mm = 9999;
    while (pros::millis() - t0 < 3000) {
        const lemlib::Pose now = chassis.getPose();
        if (std::hypot(now.x - start.x, now.y - start.y) > LINEUP_IN + OVERRUN_IN) break;
        const int mm = robot::claw::distance_mm();
        if (mm > 0 && mm < closest_mm) closest_mm = mm;
        if (mm > 0 && mm <= GRAB_MM) {
            triggered = true;
            break;
        }
        chassis.arcade(CREEP_POWER, 0, true);
        pros::delay(10);
    }
    chassis.arcade(0, 0, true);  // STOP, then close
    robot::claw::close();
    pros::delay(200);  // let the piston shut before anything moves
    if (triggered) {
        ui::log("grab_pin: closed at %d mm", closest_mm);
    } else {
        ui::warn("grab_pin: never read %d mm (closest %d) -- closed anyway", GRAB_MM, closest_mm);
    }
    robot::claw::set_armed(true);  // back on for driver control
    return triggered;
}

// ---- Red Left after the first goal: pin 2 stack, back-to-start toggle, old mirrored grab.
#if 0
    // ---- Practice: first goal -> next pin (same column, one row down).
    // ---- Target is the pose read off the FIELD page with the claw lined up
    // ---- on the pin. Moving only -- no grab yet.
    step("6 back away from goal");
    drive_straight(-15.0 / 2.54, 1000);  // 15 cm, so the claw clears the goal
    // Robot CENTRE + heading with the claw lined up on pin 2 (FIELD page).
    // This approach grabbed reliably before the toggle moved to the start.
    constexpr double PIN2_CX = 19.1, PIN2_CY = 33.2, PIN2_HEADING = 135.7;
    step("7 grab pin 2");
    grab_pin(PIN2_CX, PIN2_CY, PIN2_HEADING);  // always closes the claw; carry on and score
    step("at pin");

    // ---- Pin 2 onto the first goal, on top of pin 1. Same approach as the
    // ---- first goal (drive-out spot, face 90, drive in, push, back off,
    // ---- lower, release), just carried higher to clear pin 1.
    constexpr double STACK_EXTRA = 20;  // how much higher than pin 1 was carried; bigger = higher
    step("11 lift for stack");
    lift_to(goal_lift + STACK_EXTRA, 1500);

    // Reverse back to the drive-out spot: the pin is behind the robot's
    // heading from here, so backing up is a small turn instead of a U-turn.
    // Turns are slower with a raised pin, so it does not swing.
    step("12 back to drive-out spot");
    chassis.turnToPoint(0, 46.308, 1000, {.forwards = false, .maxSpeed = 60});
    chassis.moveToPoint(0, 46.308, 2500, {.forwards = false, .maxSpeed = 50});
    pros::delay(1000);  // settle, exactly like the first goal
    step("13 face 90");
    chassis.turnToHeading(90, 1000);
    pros::delay(1000);  // settle, exactly like the first goal

    // OVERSHOOT: aim the drive itself OVERSHOOT_IN past the goal point, so
    // it does not stop short and leave the push nothing to reach. The goal
    // stops the robot and squares it up; the timeout ends the drive.
    constexpr double GOAL_X = 16.523, GOAL_Y = 51.972, OVERSHOOT_IN = 4;
    step("14 drive INTO goal (overshoot)");
    {
        const lemlib::Pose at = chassis.getPose();
        const double gx = GOAL_X - at.x, gy = GOAL_Y - at.y;
        const double len = std::hypot(gx, gy);
        const double k = len > 0 ? (len + OVERSHOOT_IN) / len : 1.0;
        chassis.moveToPoint(at.x + gx * k, at.y + gy * k, 2000, {.maxSpeed = 50});
        chassis.waitUntilDone();
    }
    step("14b push into goal");
    chassis.arcade(60, 0, true);
    pros::delay(800);  // longer = further in
    chassis.arcade(0, 0, true);
    step("15 back up 4.5 cm");
    drive_straight(-4.5 / 2.54, 1000);
    step("15 lower lift");
    robot::lift::manual(-60);
    pros::delay(400);  // longer = lower
    robot::lift::manual(0);
    step("15 release");
    robot::claw::open();

    // ---- PAUSED: drive back to the start and toggle. The toggle now runs
    // ---- FIRST instead; this is kept in case the order goes back.
#if 0
    // ---- Back to the start pose, where the toggle works from. Reversing
    // ---- again keeps the turns small.
    step("16 back away from goal");
    drive_straight(-15.0 / 2.54, 1000);
    step("17 back to start");
    chassis.turnToPoint(0, 64.065, 1000, {.forwards = false});
    chassis.moveToPoint(0, 64.065, 2500, {.forwards = false, .maxSpeed = 50});
    step("18 face 180");
    chassis.turnToHeading(180, 1000);
    chassis.waitUntilDone();

    // Overshoot: back up MORE than needed, into the wall, so the wall squares
    // the robot up for the toggle. SLOW on purpose: at speed the impact
    // bounces the robot back off the wall. Slow and long lets it settle flat.
    step("18b back slowly into wall to align");
    chassis.arcade(-30, 0, true);
    pros::delay(1200);
    chassis.arcade(0, 0, true);
    pros::delay(200);  // let it settle before the toggle

    do_toggle();  // one flip; red needs two from yellow -- call again when wanted
#endif
    step("done");

#if 0
    constexpr double FIRST_GOAL_DROP_TO = 2.0;
    step("5 drop");
    lift_to(FIRST_GOAL_DROP_TO, 2500);

    // Back up 15 cm, straight back from wherever it actually stopped, to
    // clear the goal. Heading 0 is +Y and grows clockwise, so "forward" is
    // (sin, cos) of the heading.
    constexpr double BACK_UP_IN = 15.0 / 2.54;  // cm -> inches
    step("6 back up");
    const lemlib::Pose here = chassis.getPose(true);  // true = radians
    chassis.moveToPoint(here.x - BACK_UP_IN * std::sin(here.theta),
                        here.y - BACK_UP_IN * std::cos(here.theta), 1200,
                        {.forwards = false, .maxSpeed = 50});

    // Then aim at (28.7, -18.6) -- swung EXTRA_TURN_DEG -- and drive at the
    // cone, slowing as it gets close and stopping as soon as the claw grabs
    // it. Positive turns further left, negative further right. Red Right
    // swings 8 left; mirrored, that is 8 right, so -8 here.
    constexpr double EXTRA_TURN_DEG = -8;
    chassis.waitUntilDone();  // finish backing up before measuring from here
    const lemlib::Pose from = chassis.getPose();
    const double dx = 28.7 - from.x;
    const double dy = -18.6 - from.y;
    const double reach = std::hypot(dx, dy);
    const double aim = std::atan2(dx, dy) - EXTRA_TURN_DEG * M_PI / 180.0;
    const double aim_x = from.x + reach * std::sin(aim);
    const double aim_y = from.y + reach * std::cos(aim);

    // Slower than the rest of the routine: this is the grab. Lower = gentler.
    constexpr int GRAB_SPEED = 35;
    step("7 aim at cone");
    chassis.turnToPoint(aim_x, aim_y, 1000);
    chassis.moveToPoint(aim_x, aim_y, 2500, {.maxSpeed = GRAB_SPEED});
    step("8 slow grab");
    approach_and_grab(GRAB_SPEED, 4000);
    pros::delay(500);  // let the claw settle on the cone before moving off

    // Stack it: turn round to face the cone at (47, -24), lift, drive up to
    // it, then lower and let go.
    //
    // Positions are the robot's CENTRE, and the claw sits ahead of it, so
    // stop well short: 5 cm, plus ~9 in for centre-to-claw. Bigger number =
    // shorter drive.
    constexpr double STACK_X = 47;
    constexpr double STACK_Y = -24;
    constexpr double STOP_SHORT_IN = 8.0 / 2.54 + 9.0;

    // Lift first, so the held cone is already up when the robot swings round.
    step("9 lift for stack");
    robot::lift::manual(80);
    pros::delay(450);  // longer = higher; the held cone has to clear the other one
    robot::lift::manual(0);

    // Turn slowly: a raised lift with a cone on it tips and swings if the
    // robot spins hard. Lower maxSpeed = gentler (out of 127).
    step("10 turn to stack");
    chassis.turnToPoint(STACK_X, STACK_Y, 2500, {.maxSpeed = 60});
    chassis.waitUntilDone();

    step("11 drive to stack");
    const lemlib::Pose at = chassis.getPose();
    const double sx = STACK_X - at.x;
    const double sy = STACK_Y - at.y;
    const double gap = std::hypot(sx, sy);
    if (gap > STOP_SHORT_IN) {
        const double k = (gap - STOP_SHORT_IN) / gap;  // fraction of the way there
        chassis.moveToPoint(at.x + sx * k, at.y + sy * k, 2500, {.maxSpeed = 50});
        chassis.waitUntilDone();
    }

    step("12 drop + release");
    robot::lift::manual(-60);
    pros::delay(400);
    robot::lift::manual(0);
    robot::claw::open();
#endif
#endif  // one-pin

// ---- Red Left opener (was before the toggle): forward 5 cm, lift +92, back 7 cm.
#if 0
    // Opener: forward 5 cm, raise the lift BY 92 from wherever it is, back
    // up 7 cm. The lift stays up.
    constexpr double OPENER_RAISE = 92;
    step("0a forward 5 cm");
    drive_straight(5.0, 1000);
    step("0b raise lift by 92");
    lift_to(robot::lift::height() + OPENER_RAISE, 2000);
    step("0c back up 7 cm");
    drive_straight(-7.0, 1000);
#endif
