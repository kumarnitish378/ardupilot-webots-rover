/*
 * File:          epuck_go_forward.c
 * Date:
 * Description:   Drive 100 wheel revolutions forward, then 100 back to start.
 * Author:
 * Modifications:
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <math.h>   // for M_PI, fabs
#include <stdio.h>  // for printf

#define TIME_STEP 64

// 100 revolutions expressed in radians of wheel rotation
#define REVOLUTIONS   100.0
#define TARGET_RAD    (REVOLUTIONS * 2.0 * M_PI)   // ~628.32 rad
#define TOLERANCE     0.05                          // rad, "close enough" to target
#define WHEEL_SPEED   6.28                          // rad/s (max e-puck ~6.28)

int main(int argc, char **argv) {
  wb_robot_init();

  printf("epuck_go_forward controller started\n");
  fflush(stdout);

  // get the motor devices
  WbDeviceTag left_motor  = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

  // get the wheel position sensors so we know when a leg is finished
  WbDeviceTag left_sensor  = wb_robot_get_device("left wheel sensor");
  WbDeviceTag right_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_sensor, TIME_STEP);
  wb_position_sensor_enable(right_sensor, TIME_STEP);

  // cap the speed used to reach each position target
  wb_motor_set_velocity(left_motor, WHEEL_SPEED);
  wb_motor_set_velocity(right_motor, WHEEL_SPEED);

  // phase 0: drive forward 100 revolutions
  double target = TARGET_RAD;
  wb_motor_set_position(left_motor, target);
  wb_motor_set_position(right_motor, target);
  int phase = 0;
  printf("phase 0: driving forward %.0f revolutions\n", REVOLUTIONS);
  fflush(stdout);

  while (wb_robot_step(TIME_STEP) != -1) {
    double left_pos  = wb_position_sensor_get_value(left_sensor);
    double right_pos = wb_position_sensor_get_value(right_sensor);

    // both wheels within tolerance of the current target?
    if (fabs(left_pos - target) < TOLERANCE &&
        fabs(right_pos - target) < TOLERANCE) {
      if (phase == 0) {
        // phase 1: drive back 100 revolutions to the starting point
        phase = 1;
        target = 0.0;
        wb_motor_set_position(left_motor, target);
        wb_motor_set_position(right_motor, target);
        printf("phase 1: driving backward %.0f revolutions\n", REVOLUTIONS);
        fflush(stdout);
      } else {
        // finished: stop the wheels and hold
        wb_motor_set_velocity(left_motor, 0.0);
        wb_motor_set_velocity(right_motor, 0.0);
        printf("done: returned to start\n");
        fflush(stdout);
        break;
      }
    }
  }

  wb_robot_cleanup();
  return 0;
}
