#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-7, -11, 16},     // Left Chassis Ports (negative port will reverse it!)
    {18, 13, -6},  // Right Chassis Ports (negative port will reverse it!)

    10,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM = cartridge * (motor gear / wheel gear)

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(9, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
pid lady_brown_pid(200);

void initialize() {
  // Print our branding over your terminal :D
  //ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  // chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  // chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      //{"Drive\n\nDrive forward and come back", drive_example},
      //{"Turn\n\nTurn 3 times.", turn_example},
      //{"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
      //{"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
      //{"Swing Turn\n\nSwing in an 'S' curve", swing_example},
      //{"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
      //{"Combine all 3 movements", combining_movements},
      //{"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
      //{"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      //{"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      //{"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      //{"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      //{"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      //{"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

int intake_dir = 0;
int intake_count = 0;
bool jam = false;
void intake_check(){
  if (intake_dir == 0){
    intake.brake();
  }
  else if (intake_dir == -1){
    intake.move_velocity(-200);
  }
  else if (intake_dir == 1){
    if ((std::abs(intake.get_actual_velocity()) < 10) && !jam){
      if (intake_count > 20){
        jam = true;
        intake.move_velocity(-200);
      }
      else{
        intake_count += 1;
        intake.move_velocity(200);
      }
    }
    else{
      if (jam){
        if (intake_count > 0){
          intake.move_velocity(-200);
          intake_count -= 1;
        }
        else {
          jam = false;
          intake.move_velocity(200);
        }
      } 
      intake_count = 0;
      intake.move_velocity(200);
    }
  }
}

int sorted_color = 0;
bool seeing = false;

void color_sorter(){/*
  color_sensor.set_led_pwm(50);
 int rotation = 0;
 bool stop = false;
  while (true){  
  if ((color_sensor.get_hue() > 355 || color_sensor.get_hue() < 15 ) || (color_sensor.get_hue() > 210 && color_sensor.get_hue() < 240)){
    seeing = true;
  }
  else{seeing = false;}
  if (sorted_color == 1){
    master.print(0, 0, "red");
    //red out
    if (color_sensor.get_hue() > 355 || color_sensor.get_hue() < 15 ){
      stop = false;
      while (color_sensor.get_hue() > 355 || color_sensor.get_hue() < 15 ){pros::delay(5);}
      rotation = intake.get_position();
      while (intake.get_position() < rotation + 345)
      {
        pros::delay(5);
        if ( color_sensor.get_hue() > 355 || color_sensor.get_hue() < 15){      stop = true;
          break;}
      }
      if (! stop){
      intake.move_velocity(-200);
      pros::delay(250);
      intake.move_velocity(200);
      seeing = false;} 
    }
  }
  else if (sorted_color == 2){
    master.print(0, 0, "blue");
    stop = false;
    //blue
    if (color_sensor.get_hue() > 210 && color_sensor.get_hue() < 240){
      while ( color_sensor.get_hue() > 210 && color_sensor.get_hue() < 240){pros::delay(5);}
      rotation = intake.get_position();
      while (intake.get_position() < rotation + 345)
      {
        pros::delay(5);
        if ( color_sensor.get_hue() > 210 && color_sensor.get_hue() < 220){stop = true;
          break;}
      }
      if (!stop){
      intake.move_velocity(-200);
      pros::delay(150);
      intake.move_velocity(200);
      seeing = false;} 
    }
    }
    pros::delay(50);
  }*/
};

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void opcontrol() {
  pros::Task error_check{check_for_issues};
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */
  drive_example();
  //ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
  error_check.join();
}
bool lady_brown_pid_enabled = true;
void pid_loop(){
  if (lady_brown_pid_enabled){
  //lady_brown_pid.get_voltage(lady_brown_rotation.get_angle());
  float angle = lady_brown_rotation.get_angle()*0.01;
  if (angle > 300){
    angle -= 360;
  }
  wall_stake.move_voltage(lady_brown_pid.get_voltage(angle));
  master.print(1,1,"%i, %i, %i",lady_brown_pid.target,lady_brown_rotation.get_angle()/0.01f,lady_brown_pid.sensor);
  //master.print(1,1,"%f",lady_brown_rotation.get_angle());
    }   
}
/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {/*
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text*/
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
/*          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);*/
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void autonomous() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  bool wait = false;
  bool mogo_on = false;
  bool intake_running = false;
  bool doinker_on = false;
  int lady_brown_dir = 0;
  int lady_brown_pos = 0;
  pros::Task error_check{check_for_issues};
  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();
    chassis.opcontrol_tank();  // Tank control
    if (master.get_digital(DIGITAL_LEFT)){sorted_color = 1;};
    if (master.get_digital(DIGITAL_RIGHT)){sorted_color = 2;};

    if (master.get_digital(DIGITAL_A)){
      if (!wait){
                wait = true;
          if (!mogo_on) {clamp_piston.set_value(LOW);}
                else {clamp_piston.set_value(HIGH);}
                mogo_on = !mogo_on;}}
    /*else if (master.get_digital(DIGITAL_X)){
      if (!wait){
                wait = true;
          if (!doinker_on) {doinker.set_value(LOW);}
                else {doinker.set_value(HIGH);}
                doinker_on = !doinker_on;}
      }*/
    else if (master.get_digital(DIGITAL_R2) || master.get_digital(DIGITAL_R1)){
        if (!wait){
            wait = true;
            if (intake_running) {
                intake_dir = 0; 
                intake_running = 0;}
            else if (master.get_digital(DIGITAL_R2) && !intake_running) {
                intake_dir = -1; 
                intake_running = 1;}
            else if (master.get_digital(DIGITAL_R1) && !intake_running) {
                intake_dir = 1; 
                intake_running = 1;}
    }}
    else {wait = false;}

    if  (master.get_digital(DIGITAL_L2)){
      lady_brown_pid_enabled = false;
      wall_stake.move_velocity(200);
      lady_brown_dir = 1;}
    else if  (master.get_digital(DIGITAL_L1)){
      lady_brown_pid_enabled = false;
      wall_stake.move_velocity(-200);
      lady_brown_dir = -1;}
      else if(master.get_digital(DIGITAL_UP)) {lady_brown_pid.target = 32.5;
        lady_brown_dir = 0;
        lady_brown_pid_enabled = true;}
      else if(master.get_digital(DIGITAL_DOWN)) {lady_brown_pid.target = 160;
        lady_brown_dir = 0;
        lady_brown_pid_enabled = true;}
      else if (!lady_brown_pid_enabled && (std::abs(wall_stake.get_actual_velocity()) > 5)){
        wall_stake.brake();
        lady_brown_pid_enabled = true;
        float angle = lady_brown_rotation.get_angle()*0.01;
        if (angle > 300){
          angle -= 360;
        }  
        lady_brown_pid.target = angle;
      }
      else if (master.get_digital(DIGITAL_RIGHT)){
        lady_brown_pid_enabled = false;
        wall_stake.move_voltage(12000);
      }
      else if (master.get_digital(DIGITAL_LEFT)){
        lady_brown_pid_enabled = false;
        wall_stake.move_voltage(-12000);
      }
      if (lady_brown_pid.target < -30){lady_brown_pid.target = -30;}
      if (lady_brown_pid.target > 300){lady_brown_pid.target = 300;}
      // delay to save resources
    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
  
}

void check_for_issues(){
  while (true)
  {
  pros::delay(5);
  
  color_sorter();
  pid_loop();
  intake_check();
  
  }
}
