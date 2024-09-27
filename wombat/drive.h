/* Drive.h
 * The drive library contains code that should work on any robot running on two powered, forward-
 * facing motors. The main purpose of this library is to facilitate usage of four movements:
 * Forward, Backward, Left, and Right. Constants defined as $ need to be defined by the user.
 * !TODO! Left/Right with gyro.
 * Clean up right/left by using math.h's absolute value function.
 */

// This segment prevents including Drive.h twice.
#ifndef DRIVE_H
#define DRIVE_H

int WHITE;
int BLACK;
// Hard constants
#define MAX_SPEED 1500

// Mathematically defined constants
#define CM_TO_BEMF (BEMFS_PER_ROTATION / (M_PI * WHEEL_DIAMETER)) //Centimeters travelled to motor ticks.
#define BEMF_TO_CM ((PI * WHEEL_DIAMETER) / BEMFS_PER_ROTATION) //Motor ticks to centimeters travelled.
#define SPD_L_TURN ((SPD_L_B + SPD_R_F) / 2)
#define SPD_R_TURN ((SPD_R_B + SPD_L_F) / 2)

// Robot-specific constants
#define MOT_LEFT 0 // Port the left drive motor is plugged into.
#define MOT_RIGHT 1// Port the right drive motor is plugged into.
#define R_BUMP digital(1)
#define L_BUMP digital(0)
#define WHEEL_DIAMETER 6 // Diameter of the wheel. Common values in appendix
#define ROBOT_DIAMETER 15 // Distance from the center of one wheel to the center of the other.
#define BEMFS_PER_ROTATION 1500 // Motor ticks per rotation. KIPR says this value should be around 1500.

// Tuning Constants
#define SPD_L_F 1300 // Left forward speed. Max is 1500.
#define SPD_R_F 1300 // Right forward speed. Max is 1500.
#define SPD_L_B 1300 // Left backward speed. Max is 1500.
#define SPD_R_B 1300 // Right backward speed. Max is 1500.

/* Gyro Constants
 * GYRO defines the orientation of the wallaby
 * GYRO_SENS is how sensitive the gyro code is
 * gyro_dev is the average deviation of the gyrometer
 */
#define GYRO 3 // 1: x, 2: y, 3: z
#define GYRO_SENS 2// Usually a value of 1 works quite well. Make this value higher to make it less sensitive.
float gyro_dev; // Use the function calc_dev() to set this variable

// Line follow constants
#define LEFT 0
#define RIGHT 1
#define LIGHT_S() analog10(1)
#define WAIT(thing); while(!(thing)) { msleep(1); }
// Low-Level drive commands

/*
 * \brief Shuts down the two drive motors.
 */
void drive_off();

/* (!NEEDS TESTING!)
 * \brief Actively brakes to shut off the two drive motors for more precision.
 */
 void drive_freeze();

/*
 * \brief Clears the position counter on the two drive motors.
 */
void drive_clear();

/*
 * \brief Sets the two drive motors to move at certain speeds.
 * \param left_speed the speed of the left motor.
 * \param right_speed the speed of the right motor.
 */
void drive(int left_speed, int right_speed);

// Main drive commands.

/*
 * \brief Turns right a certain amount of degrees on a certain radius
 * \param degrees the amount to turn, in degrees. Negative values do a reverse turn.
 * \param radius the radius of the turn, in centimeters. Only use values 0 and above.
 * NOTE: degrees is an int because the wallaby only has precision up to about 2-3 degrees.
 */
void right(int degrees, double radius);

/*
 * \brief Turns right a certain amount of degrees on a certain radius
 * \param degrees the amount to turn, in degrees. Negative values do a reverse turn.
 * \param radius the radius of the turn, in centimeters. Only use values 0 and above.
 * \param speed the speed to travel at.
 * NOTE: degrees is an int because the wallaby only has precision up to about 2-3 degrees.
 */
void right_speed(int degrees, double radius, int speed);

/*
 * \brief Turns left a certain amount of degrees on a certain radius
 * \param degrees the amount to turn, in degrees. Negative values do a reverse turn.
 * \param radius the radius of the turn, in centimeters. Only use values 0 and above.
 * NOTE: degrees is an int because the wallaby only has precision up to about 2-3 degrees.
 */
void left(int degrees, double radius);

/*
 * \brief Turns left a certain amount of degrees on a certain radius
 * \param degrees the amount to turn, in degrees. Negative values do a reverse turn.
 * \param radius the radius of the turn, in centimeters. Only use values 0 and above.
 * \param speed the speed to travel at.
 * NOTE: degrees is an int because the wallaby only has precision up to about 2-3 degrees.
 */
void left_speed(int degrees, double radius, int speed);

/*
 * \brief Drives forward a certain distance at default drive speed.
 * \param distance the distance to travel, in centimeters. Only use values 0 and above.
 * NOTE: distance is an integer as the wallaby only has precision up to about 1-2 centimeters.
 */
void forward(int distance);

/*
 * \brief Drives forward a certain distance at a custom speed.
 * \param distance the distance to travel, in centimeters. Only use values 0 and above.
 * \param speed the speed to travel at.
 * NOTE: distance is an integer as the wallaby only has precision up to about 1-2 centimeters.
 */
 void forward_speed(int distance, int speed);
 
/*
 * \brief Drives backward a certain distance at default drive speed.
 * \param distance the distance to travel, in centimeters. Only use values 0 and above.
 * NOTE: distance is an integer as the wallaby only has precision up to about 1-2 centimeters.
 */
void backward(int distance);

/*
 * \brief Drives backward a certain distance at default drive speed.
 * \param distance the distance to travel, in centimeters. Only use values 0 and above.
 * \param speed the speed to travel at.
 * NOTE: distance is an integer as the wallaby only has precision up to about 1-2 centimeters.
 */
void backward_speed(int distance, int speed);

void forward_gyro(float dist, int speed);

void calc_dev();

void lineFollow(int speed, int ms, int side, int port);

void light_start ();

// Below are added in 2022

void slow_servo(int fin, int inc, int port); //fin is the desired final position, inc is the increment to pause at in ms (ideally less than 20)

void calibrate_Tophat();

void square_up();

#endif

/* APPENDIX
 * Common wheel diameter values: (To be added at a future date)
 */

 
 
