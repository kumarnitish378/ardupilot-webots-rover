/*
 * File:          ugv_teleop.c
 * Description:   Keyboard teleop for the ugv_rover world: front wheels steer
 *                (position control), rear wheels drive (velocity control).
 *                Webots has no mechanical differential node, so the rear
 *                wheel speeds are derived kinematically from a bicycle-model
 *                turn radius (steering angle + wheelbase/track), giving the
 *                inner wheel a lower speed and the outer wheel a higher one
 *                while turning, instead of commanding both wheels identically.
 *                GPS/IMU/distance-sensor readings, plus
 *                the commanded steering angle, the *actual* steering angle
 *                measured by the steering-joint position sensors, and both
 *                rear wheel speeds are printed once per second and logged
 *                every simulation step to ugv_teleop_log.csv (next to this
 *                controller) so oscillation/overshoot can be inspected after
 *                a driving session, not just guessed at from the console.
 *
 *                Like a real car, this only turns while moving - there is
 *                no in-place pivot the way skid-steer had.
 *
 *                Sensor reading and motor writing are kept in their own
 *                functions so the keyboard layer can later be swapped for
 *                an ArduPilot SITL socket bridge without touching the rest.
 *
 *                The CSV log carries the full rover state every step (not
 *                just steering/wheel speed) so events like driving into an
 *                obstacle and getting stuck show up as a frozen pos/heading
 *                trace, not just something you had to catch live in the
 *                console.
 *
 *                Three drive modes, switched with dedicated keys (edge
 *                triggered, so holding the key doesn't spam the switch):
 *                  'M' - MANUAL: arrow keys drive directly.
 *                  'A' - AUTONOMOUS: autonomous_decide() closes the loop
 *                        itself from ds_left/ds_right telemetry (drive
 *                        forward when clear, steer away from whichever
 *                        side reads closer, reverse-and-turn if boxed in),
 *                        with a stuck-detection fallback that watches
 *                        actual GPS displacement and forces a reverse/turn
 *                        recovery if it isn't moving despite trying to -
 *                        this runs at full simulation rate inside the
 *                        controller, so unlike REMOTE mode there is no
 *                        external round-trip delay.
 *                  'R' - REMOTE: drive/steer are read from a plain text
 *                        command file (rover_command.txt, next to this
 *                        controller) instead of the keyboard. An external
 *                        process - or a human editing the file by hand -
 *                        can write "drive steer" (each in [-1,1]) to it at
 *                        any time; the last value written just holds until
 *                        a new one is written. Combined with reading
 *                        ugv_teleop_log.csv back, this is a real (if
 *                        turn-based rather than continuous) send-command /
 *                        read-telemetry loop.
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/distance_sensor.h>
#include <webots/radar.h>
#include <webots/keyboard.h>
#include <math.h>
#include <stdio.h>

#define TIME_STEP 16
#define MAX_WHEEL_SPEED 8.0   // rad/s, rear drive wheels (motors' maxVelocity is 15)
#define MAX_STEER_ANGLE 0.575  // rad (~33 deg, i.e. the old 23 deg + 10), matches the front HingeJoints' minStop/maxStop
#define PRINT_PERIOD_MS 1000
#define RAD2DEG (180.0 / M_PI)

// Must match the wheel anchor positions in ugv_rover.wbt: front anchors at
// x=+-0.2 and rear at x=-0.2 -> wheelbase 0.4; left/right anchors at
// y=+-0.15 -> track 0.3.
#define WHEELBASE 0.4
#define TRACK_WIDTH 0.3

// Autonomous obstacle-avoidance thresholds, in meters (ds_left/ds_right read
// distance directly, up to their 3 m max range - see the .wbt lookupTable).
#define OBSTACLE_NEAR_M 0.6   // start steering away from the closer side
#define OBSTACLE_STOP_M 0.25  // too close on both sides - reverse and turn instead

#define RADAR_MAX_RANGE 3.0  // must match maxRange on the corner Radar nodes in the .wbt

// Stuck detection: distance-sensor thresholds alone don't catch "wedged
// against/on top of an obstacle and spinning without actually moving" (the
// steering-joint/rock incident) - a rock's irregular mesh can grab a wheel
// without ever showing up as "too close" on either sensor. So we also track
// actual GPS displacement over a rolling window and force a recovery
// maneuver if it isn't moving despite trying to drive.
#define STUCK_CHECK_PERIOD_S 1.0   // how often to sample position
#define STUCK_DISPLACEMENT_M 0.05  // moved less than this while driving = stuck
#define STUCK_RECOVERY_S 1.5       // how long to force reverse-and-turn recovery

#define COMMAND_FILE "rover_command.txt"

typedef enum { MODE_MANUAL, MODE_AUTONOMOUS, MODE_REMOTE } DriveMode;

static const char *mode_name(DriveMode mode) {
  switch (mode) {
    case MODE_AUTONOMOUS:
      return "AUTONOMOUS";
    case MODE_REMOTE:
      return "REMOTE";
    default:
      return "MANUAL";
  }
}

typedef struct {
  WbDeviceTag steer_fl;
  WbDeviceTag steer_fr;
  WbDeviceTag wheel_rl;
  WbDeviceTag wheel_rr;
} Motors;

typedef struct {
  WbDeviceTag gps;
  WbDeviceTag imu;
  WbDeviceTag ds_left;
  WbDeviceTag ds_right;
  WbDeviceTag steer_fl_sensor;
  WbDeviceTag steer_fr_sensor;
  WbDeviceTag radar_fl;
  WbDeviceTag radar_fr;
  WbDeviceTag radar_rl;
  WbDeviceTag radar_rr;
} Sensors;

typedef struct {
  double pos[3];
  double heading_deg;
  double ds_left;
  double ds_right;
  double radar_fl;
  double radar_fr;
  double radar_rl;
  double radar_rr;
} RoverState;

typedef struct {
  int initialized;
  double last_check_time;
  double last_check_pos[2];
  double recovery_until_time;
  double recovery_steer_sign;
} AutonomousState;

static void autonomous_state_init(AutonomousState *as) {
  as->initialized = 0;
  as->last_check_time = 0.0;
  as->last_check_pos[0] = 0.0;
  as->last_check_pos[1] = 0.0;
  as->recovery_until_time = -1.0;
  as->recovery_steer_sign = 1.0;
}

static void motors_init(Motors *m) {
  m->steer_fl = wb_robot_get_device("steer_fl");
  m->steer_fr = wb_robot_get_device("steer_fr");
  m->wheel_rl = wb_robot_get_device("wheel_rl");
  m->wheel_rr = wb_robot_get_device("wheel_rr");

  // rear drive wheels: infinite target position puts them in pure velocity mode
  wb_motor_set_position(m->wheel_rl, INFINITY);
  wb_motor_set_position(m->wheel_rr, INFINITY);
  wb_motor_set_velocity(m->wheel_rl, 0.0);
  wb_motor_set_velocity(m->wheel_rr, 0.0);

  // front steering motors stay in the default position-control mode
  wb_motor_set_position(m->steer_fl, 0.0);
  wb_motor_set_position(m->steer_fr, 0.0);
}

static void sensors_init(Sensors *s) {
  s->gps = wb_robot_get_device("gps");
  s->imu = wb_robot_get_device("imu");
  s->ds_left = wb_robot_get_device("ds_left");
  s->ds_right = wb_robot_get_device("ds_right");
  s->steer_fl_sensor = wb_robot_get_device("steer_fl_sensor");
  s->steer_fr_sensor = wb_robot_get_device("steer_fr_sensor");
  s->radar_fl = wb_robot_get_device("radar_fl");
  s->radar_fr = wb_robot_get_device("radar_fr");
  s->radar_rl = wb_robot_get_device("radar_rl");
  s->radar_rr = wb_robot_get_device("radar_rr");

  wb_gps_enable(s->gps, TIME_STEP);
  wb_inertial_unit_enable(s->imu, TIME_STEP);
  wb_distance_sensor_enable(s->ds_left, TIME_STEP);
  wb_distance_sensor_enable(s->ds_right, TIME_STEP);
  wb_position_sensor_enable(s->steer_fl_sensor, TIME_STEP);
  wb_position_sensor_enable(s->steer_fr_sensor, TIME_STEP);
  wb_radar_enable(s->radar_fl, TIME_STEP);
  wb_radar_enable(s->radar_fr, TIME_STEP);
  wb_radar_enable(s->radar_rl, TIME_STEP);
  wb_radar_enable(s->radar_rr, TIME_STEP);
}

// Closest target's distance on a radar, or RADAR_MAX_RANGE if it sees nothing
// (mirrors how ds_left/ds_right read "3.00" - their max range - when clear).
static double read_radar_min_distance(WbDeviceTag radar) {
  int n = wb_radar_get_number_of_targets(radar);
  if (n <= 0)
    return RADAR_MAX_RANGE;

  const WbRadarTarget *targets = wb_radar_get_targets(radar);
  double min_distance = targets[0].distance;
  for (int i = 1; i < n; i++) {
    if (targets[i].distance < min_distance)
      min_distance = targets[i].distance;
  }
  return min_distance;
}

// Reads all pending key events this step and turns them into a manual
// drive/steer command, each in [-1, 1], plus an edge-triggered mode
// request: *requested_mode is set to the newly-pressed mode key's mode
// (M/A/R just went down this step, not just "is held"), or left unchanged
// (still whatever it was) if no mode key was pressed this step. Callers
// should check requested_mode against the current mode before switching.
static void read_keyboard_command(double *drive, double *steer, DriveMode *requested_mode, int *mode_requested) {
  static int m_was_down = 0, a_was_down = 0, r_was_down = 0;
  int m_down = 0, a_down = 0, r_down = 0;

  *drive = 0.0;
  *steer = 0.0;
  *mode_requested = 0;

  int key = wb_keyboard_get_key();
  while (key >= 0) {
    switch (key) {
      case WB_KEYBOARD_UP:
        *drive += 1.0;
        break;
      case WB_KEYBOARD_DOWN:
        *drive -= 1.0;
        break;
      case WB_KEYBOARD_LEFT:
        *steer += 1.0;
        break;
      case WB_KEYBOARD_RIGHT:
        *steer -= 1.0;
        break;
      case 'M':
        m_down = 1;
        break;
      case 'A':
        a_down = 1;
        break;
      case 'R':
        r_down = 1;
        break;
      default:
        break;
    }
    key = wb_keyboard_get_key();
  }

  if (m_down && !m_was_down) {
    *requested_mode = MODE_MANUAL;
    *mode_requested = 1;
  } else if (a_down && !a_was_down) {
    *requested_mode = MODE_AUTONOMOUS;
    *mode_requested = 1;
  } else if (r_down && !r_was_down) {
    *requested_mode = MODE_REMOTE;
    *mode_requested = 1;
  }
  m_was_down = m_down;
  a_was_down = a_down;
  r_was_down = r_down;

  if (*drive > 1.0)
    *drive = 1.0;
  if (*drive < -1.0)
    *drive = -1.0;
  if (*steer > 1.0)
    *steer = 1.0;
  if (*steer < -1.0)
    *steer = -1.0;
}

// Reads "drive steer" (each in [-1,1]) from COMMAND_FILE. If the file is
// missing or unparsable, the previous command just holds - an external
// commander writes a value once and it stays in effect until overwritten,
// rather than needing to be refreshed every step.
static void read_remote_command(double *drive, double *steer) {
  static double last_drive = 0.0, last_steer = 0.0;

  FILE *f = fopen(COMMAND_FILE, "r");
  if (f != NULL) {
    double d, s;
    if (fscanf(f, "%lf %lf", &d, &s) == 2) {
      if (d > 1.0)
        d = 1.0;
      if (d < -1.0)
        d = -1.0;
      if (s > 1.0)
        s = 1.0;
      if (s < -1.0)
        s = -1.0;
      last_drive = d;
      last_steer = s;
    }
    fclose(f);
  }

  *drive = last_drive;
  *steer = last_steer;
}

// Reactive obstacle avoidance: drive forward when the path is clear, steer
// away from whichever side reads closer once something is near, and reverse
// while turning toward the more-open side if boxed in close on both sides.
// Positive steer turns left (see mix_ackermann_differential). Each side's
// proximity is the closest of that side's front distance sensor and its
// front/rear corner radars, so a wall or rock near a corner (not just
// straight ahead) is caught too - this is what actually avoids boundary
// collisions, not just head-on ones.
// Also tracks actual GPS displacement: if it isn't moving despite trying to
// drive - e.g. wedged on a rock, spinning in place - a plain range-sensor
// check wouldn't necessarily catch that (the obstacle might not read as
// "too close" on any sensor), so this forces a reverse-and-turn recovery
// instead, alternating direction each time so it doesn't just rock against
// the same obstacle repeatedly.
static void autonomous_decide(AutonomousState *as, double now_s, const double *pos, const RoverState *state,
                               double *drive, double *steer) {
  if (!as->initialized) {
    as->last_check_time = now_s;
    as->last_check_pos[0] = pos[0];
    as->last_check_pos[1] = pos[1];
    as->initialized = 1;
  }

  if (now_s - as->last_check_time >= STUCK_CHECK_PERIOD_S) {
    double dx = pos[0] - as->last_check_pos[0];
    double dy = pos[1] - as->last_check_pos[1];
    double displacement = sqrt(dx * dx + dy * dy);

    if (displacement < STUCK_DISPLACEMENT_M) {
      as->recovery_until_time = now_s + STUCK_RECOVERY_S;
      as->recovery_steer_sign = -as->recovery_steer_sign;  // alternate side each time
    }

    as->last_check_time = now_s;
    as->last_check_pos[0] = pos[0];
    as->last_check_pos[1] = pos[1];
  }

  if (now_s < as->recovery_until_time) {
    *drive = -0.6;
    *steer = as->recovery_steer_sign;
    return;
  }

  double left_proximity = fmin(state->ds_left, fmin(state->radar_fl, state->radar_rl));
  double right_proximity = fmin(state->ds_right, fmin(state->radar_fr, state->radar_rr));

  if (left_proximity < OBSTACLE_STOP_M && right_proximity < OBSTACLE_STOP_M) {
    *drive = -0.6;
    *steer = (left_proximity > right_proximity) ? 1.0 : -1.0;
    return;
  }
  if (left_proximity < OBSTACLE_NEAR_M || right_proximity < OBSTACLE_NEAR_M) {
    *drive = 0.5;
    *steer = (left_proximity < right_proximity) ? -1.0 : 1.0;
    return;
  }
  *drive = 1.0;
  *steer = 0.0;
}

// Kinematic stand-in for a mechanical differential: given the commanded
// center-line wheel speed and steering angle, derive the left/right rear
// wheel speeds from the bicycle-model turn radius so the inner wheel goes
// slower and the outer wheel goes faster while turning.
static void mix_ackermann_differential(double wheel_speed, double steer_angle, double *left_speed,
                                        double *right_speed) {
  const double MIN_STEER_ANGLE = 0.01;  // rad; avoid a near-zero-angle blow-up
  if (fabs(steer_angle) < MIN_STEER_ANGLE) {
    *left_speed = wheel_speed;
    *right_speed = wheel_speed;
    return;
  }

  // Turn radius at the rear axle. Positive steer_angle turns left, putting
  // the turn center to the vehicle's left, so turn_radius comes out positive
  // and the sign convention below handles both turn directions on its own.
  double turn_radius = WHEELBASE / tan(steer_angle);
  *left_speed = wheel_speed * (turn_radius - TRACK_WIDTH / 2.0) / turn_radius;
  *right_speed = wheel_speed * (turn_radius + TRACK_WIDTH / 2.0) / turn_radius;
}

static void read_rover_state(const Sensors *s, RoverState *state) {
  const double *pos = wb_gps_get_values(s->gps);
  state->pos[0] = pos[0];
  state->pos[1] = pos[1];
  state->pos[2] = pos[2];

  const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(s->imu);
  state->heading_deg = rpy[2] * RAD2DEG;  // yaw about Z (up) in this ENU world

  state->ds_left = wb_distance_sensor_get_value(s->ds_left);
  state->ds_right = wb_distance_sensor_get_value(s->ds_right);

  state->radar_fl = read_radar_min_distance(s->radar_fl);
  state->radar_fr = read_radar_min_distance(s->radar_fr);
  state->radar_rl = read_radar_min_distance(s->radar_rl);
  state->radar_rr = read_radar_min_distance(s->radar_rr);
}

static void motors_write(const Motors *m, double left_speed, double right_speed, double steer_angle) {
  wb_motor_set_velocity(m->wheel_rl, left_speed);
  wb_motor_set_velocity(m->wheel_rr, right_speed);

  wb_motor_set_position(m->steer_fl, steer_angle);
  wb_motor_set_position(m->steer_fr, steer_angle);
}

static FILE *log_open(void) {
  FILE *f = fopen("ugv_teleop_log.csv", "w");
  if (f != NULL) {
    fprintf(f,
            "time_s,mode,pos_x,pos_y,pos_z,heading_deg,ds_left_m,ds_right_m,"
            "radar_fl_m,radar_fr_m,radar_rl_m,radar_rr_m,"
            "steer_cmd_deg,steer_fl_actual_deg,steer_fr_actual_deg,wheel_rl_speed_rad_s,wheel_rr_speed_rad_s\n");
    fflush(f);
  }
  return f;
}

static void log_write(FILE *f, double time_s, DriveMode mode, const RoverState *state, double steer_cmd_deg,
                       double steer_fl_deg, double steer_fr_deg, double wheel_rl_speed, double wheel_rr_speed) {
  if (f == NULL)
    return;
  fprintf(f, "%.3f,%s,%.3f,%.3f,%.3f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f\n", time_s,
          mode_name(mode), state->pos[0], state->pos[1], state->pos[2], state->heading_deg, state->ds_left,
          state->ds_right, state->radar_fl, state->radar_fr, state->radar_rl, state->radar_rr, steer_cmd_deg,
          steer_fl_deg, steer_fr_deg, wheel_rl_speed, wheel_rr_speed);
}

static void print_status(DriveMode mode, const RoverState *state, double steer_cmd_deg, double steer_fl_deg,
                          double steer_fr_deg, double wheel_rl_speed, double wheel_rr_speed) {
  printf(
      "[%s] pos=(%.2f, %.2f, %.2f) m  heading=%.1f deg  ds_left=%.2f m  ds_right=%.2f m\n"
      "  radar fl=%.2f fr=%.2f rl=%.2f rr=%.2f m\n"
      "  steer_cmd=%.1f deg  steer_fl=%.1f deg  steer_fr=%.1f deg  wheel_rl=%.2f rad/s  wheel_rr=%.2f rad/s\n",
      mode_name(mode), state->pos[0], state->pos[1], state->pos[2], state->heading_deg, state->ds_left, state->ds_right,
      state->radar_fl, state->radar_fr, state->radar_rl, state->radar_rr, steer_cmd_deg, steer_fl_deg, steer_fr_deg,
      wheel_rl_speed, wheel_rr_speed);
  fflush(stdout);
}

int main(int argc, char **argv) {
  wb_robot_init();

  Motors motors;
  Sensors sensors;
  motors_init(&motors);
  sensors_init(&sensors);
  wb_keyboard_enable(TIME_STEP);

  FILE *log_file = log_open();
  if (log_file == NULL) {
    printf("ugv_teleop: warning - could not open ugv_teleop_log.csv for writing\n");
    fflush(stdout);
  }

  printf(
      "ugv_teleop: click the 3D view then use arrow keys (Up/Down = drive, Left/Right = steer).\n"
      "Press 'M' for MANUAL, 'A' for AUTONOMOUS, 'R' for REMOTE (reads %s). Starting in MANUAL.\n",
      COMMAND_FILE);
  fflush(stdout);

  DriveMode mode = MODE_MANUAL;
  AutonomousState auto_state;
  autonomous_state_init(&auto_state);
  int since_last_print_ms = 0;

  while (wb_robot_step(TIME_STEP) != -1) {
    double drive, steer;
    DriveMode requested_mode;
    int mode_requested;
    read_keyboard_command(&drive, &steer, &requested_mode, &mode_requested);

    if (mode_requested && requested_mode != mode) {
      mode = requested_mode;
      if (mode == MODE_AUTONOMOUS)
        autonomous_state_init(&auto_state);  // fresh stuck-detection baseline on (re-)entry
      printf("ugv_teleop: mode -> %s\n", mode_name(mode));
      fflush(stdout);
    }

    // Telemetry has to be read before the drive decision: autonomous/remote
    // modes need this step's state to decide this step's command.
    RoverState state;
    read_rover_state(&sensors, &state);

    if (mode == MODE_AUTONOMOUS)
      autonomous_decide(&auto_state, wb_robot_get_time(), state.pos, &state, &drive, &steer);
    else if (mode == MODE_REMOTE)
      read_remote_command(&drive, &steer);

    double wheel_speed = drive * MAX_WHEEL_SPEED;
    double steer_angle = steer * MAX_STEER_ANGLE;

    double left_speed, right_speed;
    mix_ackermann_differential(wheel_speed, steer_angle, &left_speed, &right_speed);
    motors_write(&motors, left_speed, right_speed, steer_angle);

    double steer_fl_actual_deg = wb_position_sensor_get_value(sensors.steer_fl_sensor) * RAD2DEG;
    double steer_fr_actual_deg = wb_position_sensor_get_value(sensors.steer_fr_sensor) * RAD2DEG;
    double steer_cmd_deg = steer_angle * RAD2DEG;

    log_write(log_file, wb_robot_get_time(), mode, &state, steer_cmd_deg, steer_fl_actual_deg, steer_fr_actual_deg,
              left_speed, right_speed);

    since_last_print_ms += TIME_STEP;
    if (since_last_print_ms >= PRINT_PERIOD_MS) {
      print_status(mode, &state, steer_cmd_deg, steer_fl_actual_deg, steer_fr_actual_deg, left_speed, right_speed);
      since_last_print_ms = 0;
    }
  }

  if (log_file != NULL)
    fclose(log_file);

  wb_robot_cleanup();
  return 0;
}
