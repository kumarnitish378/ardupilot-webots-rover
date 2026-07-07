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
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/distance_sensor.h>
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
} Sensors;

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

  wb_gps_enable(s->gps, TIME_STEP);
  wb_inertial_unit_enable(s->imu, TIME_STEP);
  wb_distance_sensor_enable(s->ds_left, TIME_STEP);
  wb_distance_sensor_enable(s->ds_right, TIME_STEP);
  wb_position_sensor_enable(s->steer_fl_sensor, TIME_STEP);
  wb_position_sensor_enable(s->steer_fr_sensor, TIME_STEP);
}

// Reads all pending key events this step and turns them into a
// drive/steer command, each in [-1, 1].
static void read_keyboard_command(double *drive, double *steer) {
  *drive = 0.0;
  *steer = 0.0;

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
      default:
        break;
    }
    key = wb_keyboard_get_key();
  }

  if (*drive > 1.0)
    *drive = 1.0;
  if (*drive < -1.0)
    *drive = -1.0;
  if (*steer > 1.0)
    *steer = 1.0;
  if (*steer < -1.0)
    *steer = -1.0;
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

static void motors_write(const Motors *m, double left_speed, double right_speed, double steer_angle) {
  wb_motor_set_velocity(m->wheel_rl, left_speed);
  wb_motor_set_velocity(m->wheel_rr, right_speed);

  wb_motor_set_position(m->steer_fl, steer_angle);
  wb_motor_set_position(m->steer_fr, steer_angle);
}

static FILE *log_open(void) {
  FILE *f = fopen("ugv_teleop_log.csv", "w");
  if (f != NULL) {
    fprintf(f, "time_s,steer_cmd_deg,steer_fl_actual_deg,steer_fr_actual_deg,wheel_rl_speed_rad_s,wheel_rr_speed_rad_s\n");
    fflush(f);
  }
  return f;
}

static void log_write(FILE *f, double time_s, double steer_cmd_deg, double steer_fl_deg, double steer_fr_deg,
                       double wheel_rl_speed, double wheel_rr_speed) {
  if (f == NULL)
    return;
  fprintf(f, "%.3f,%.2f,%.2f,%.2f,%.3f,%.3f\n", time_s, steer_cmd_deg, steer_fl_deg, steer_fr_deg, wheel_rl_speed,
          wheel_rr_speed);
}

static void print_status(const Sensors *s, double steer_cmd_deg, double steer_fl_deg, double steer_fr_deg,
                          double wheel_rl_speed, double wheel_rr_speed) {
  const double *pos = wb_gps_get_values(s->gps);
  const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(s->imu);
  double heading_deg = rpy[2] * RAD2DEG;  // yaw about Z (up) in this ENU world
  double ds_left = wb_distance_sensor_get_value(s->ds_left);
  double ds_right = wb_distance_sensor_get_value(s->ds_right);

  printf(
      "pos=(%.2f, %.2f, %.2f) m  heading=%.1f deg  ds_left=%.2f m  ds_right=%.2f m\n"
      "  steer_cmd=%.1f deg  steer_fl=%.1f deg  steer_fr=%.1f deg  wheel_rl=%.2f rad/s  wheel_rr=%.2f rad/s\n",
      pos[0], pos[1], pos[2], heading_deg, ds_left, ds_right, steer_cmd_deg, steer_fl_deg, steer_fr_deg, wheel_rl_speed,
      wheel_rr_speed);
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

  printf("ugv_teleop: click the 3D view then use arrow keys (Up/Down = drive, Left/Right = steer)\n");
  fflush(stdout);

  int since_last_print_ms = 0;

  while (wb_robot_step(TIME_STEP) != -1) {
    double drive, steer;
    read_keyboard_command(&drive, &steer);

    double wheel_speed = drive * MAX_WHEEL_SPEED;
    double steer_angle = steer * MAX_STEER_ANGLE;

    double left_speed, right_speed;
    mix_ackermann_differential(wheel_speed, steer_angle, &left_speed, &right_speed);
    motors_write(&motors, left_speed, right_speed, steer_angle);

    double steer_fl_actual_deg = wb_position_sensor_get_value(sensors.steer_fl_sensor) * RAD2DEG;
    double steer_fr_actual_deg = wb_position_sensor_get_value(sensors.steer_fr_sensor) * RAD2DEG;
    double steer_cmd_deg = steer_angle * RAD2DEG;

    log_write(log_file, wb_robot_get_time(), steer_cmd_deg, steer_fl_actual_deg, steer_fr_actual_deg, left_speed,
              right_speed);

    since_last_print_ms += TIME_STEP;
    if (since_last_print_ms >= PRINT_PERIOD_MS) {
      print_status(&sensors, steer_cmd_deg, steer_fl_actual_deg, steer_fr_actual_deg, left_speed, right_speed);
      since_last_print_ms = 0;
    }
  }

  if (log_file != NULL)
    fclose(log_file);

  wb_robot_cleanup();
  return 0;
}
