#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 15, 22, 23);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// Knob configuration (now using integers)
int min_angle = 0;              // Minimum angle in degrees
int max_angle = 300;            // Maximum angle in degrees
int step_size = 10;             // Step size in degrees
int num_positions = 31;         // Total positions (calculated automatically)

int current_position = 0;       // Current absolute position
float sensor_position = 0;      // Accumulated sensor position (keep as float for precision)
int target_position = 0;        // Target position in degrees
float last_sensor_angle = 0;    // Last raw sensor angle

// Gauge command - motor moves to this position
bool moving_to_gauge = false;   // True when actively moving to gauge position
int gauge_position = 0;         // Target position from gauge command

// Haptic feedback parameters
float detent_strength = 8.0;    // Strength of the detent (0-8)
float end_stop_strength = 4.0;  // Strength of hard stops
float damping = 0.8;            // Damping factor for smooth feel

// Print control
bool enable_position_print = true;  // Enable/disable position updates

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== Pro Motor Knob with Detents ===");
  Serial.println("Rotate the motor shaft by hand");
  Serial.println("Commands:");
  Serial.println("  l[min] [max] : set angle limits (l20 100)");
  Serial.println("  t[number]    : set step size (t5)");
  Serial.println("  g[number]    : set knob to position (g150)");
  Serial.println("  p            : toggle position print statements");
  Serial.println("  s[number]    : set detent strength (s2.0)");
  Serial.println("  d[number]    : set damping (d0.8)");
  Serial.println("  ?            : show current settings");
  Serial.println("-----------------------------------");

  Wire.begin(12, 13); // SDA, SCL
  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // Motor configuration for knob mode
  motor.sensor_direction = Direction::CW;
  motor.voltage_limit = 3.0;  // Lower voltage for safety when hand-rotating
  motor.velocity_limit = 100; // Limit speed for smooth operation
  motor.foc_modulation = FOCModulationType::SinePWM;

  // Use torque control mode for haptic feedback
  motor.controller = MotionControlType::torque;

  // Initialize motor
  motor.init();
  motor.initFOC();

  // Get initial position
  last_sensor_angle = sensor.getAngle();
  sensor_position = 0;  // Start at 0
  current_position = 0;
  target_position = 0;

  // Calculate initial num_positions
  updateNumPositions();

  Serial.println("Knob ready!");
  Serial.print("Initial position: ");
  Serial.print(target_position);
  Serial.println(" degrees");
}

void loop() {
  motor.loopFOC();

  // Static variables for state tracking
  static bool was_in_bounds = true;
  static bool was_out_of_bounds = false;
  static int last_reported_position = -999;

  // Read current angle from sensor (in radians)
  float sensor_angle = sensor.getAngle();

  // Calculate angle change (handle wraparound at 2*PI)
  float delta = sensor_angle - last_sensor_angle;
  if (delta > _PI) delta -= 2*_PI;
  if (delta < -_PI) delta += 2*_PI;

  // Update accumulated position (in radians, can go negative or > 2*PI)
  sensor_position += delta;
  last_sensor_angle = sensor_angle;

  // Convert accumulated position to degrees (integer)
  current_position = (int)(sensor_position * 180.0 / _PI);

  // Check if we're moving to a gauge position
  if (moving_to_gauge) {
    // Calculate error in radians
    float position_error = (gauge_position - current_position) * _PI / 180.0;

    // Check if we've reached the target (within 2 degrees)
    if (abs(gauge_position - current_position) < 2) {
      moving_to_gauge = false;
      if (enable_position_print) {
        Serial.print("Reached position: ");
        Serial.print(gauge_position);
        Serial.println(" degrees");
      }
      last_reported_position = -999; // Reset for knob mode reporting
    }

    // Strong position control to move to target
    float position_torque = 5.0 * position_error;

    // Add velocity damping
    float velocity = motor.shaft_velocity;
    float damping_torque = -0.05 * velocity;

    // Apply torque
    motor.move(position_torque + damping_torque);
  }
  else {
    // Normal knob mode with detents
    // Apply position limits with active snap-back
    if (current_position < min_angle) {
      // Below min - apply torque to push back
      float error = (min_angle - current_position) * _PI / 180.0;
      float stop_torque = end_stop_strength * error;
      motor.move(stop_torque);

      // Report position
      if (was_in_bounds && enable_position_print) {
        Serial.print("At lower limit (");
        Serial.print(min_angle);
        Serial.println(" degrees)");
        was_in_bounds = false;
        was_out_of_bounds = true;
      }
    }
    else if (current_position > max_angle) {
      // Above max - apply torque to push back
      float error = (max_angle - current_position) * _PI / 180.0;
      float stop_torque = end_stop_strength * error;
      motor.move(stop_torque);

      // Report position
      if (was_in_bounds && enable_position_print) {
        Serial.print("At upper limit (");
        Serial.print(max_angle);
        Serial.println(" degrees)");
        was_in_bounds = false;
        was_out_of_bounds = true;
      }
    }
    else {
      // Within bounds - normal detent behavior
      target_position = snapToDetent(current_position);

      // Calculate error to nearest detent (in radians)
      float position_error = (target_position - current_position) * _PI / 180.0;

      // Create detent torque
      float detent_torque = detent_strength * position_error;

      // Add velocity damping for smooth feel
      float velocity = motor.shaft_velocity;
      float damping_torque = -damping * velocity * 0.01;

      // Apply combined torque
      motor.move(detent_torque + damping_torque);

      // Reset bounds flags when back in range
      if (was_out_of_bounds) {
        was_out_of_bounds = false;
        was_in_bounds = true;
      }

      // Report new detent position
      if (abs(current_position - target_position) < 2 &&
          target_position != last_reported_position && enable_position_print) {
        last_reported_position = target_position;
        Serial.print("Position: ");
        Serial.print(target_position);
        Serial.println(" degrees");
      }
    }
  }

  // Handle Serial commands
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      char first_char = input.charAt(0);

      if (first_char == 'l') {
        // Set limits: l[min] [max]
        int space_pos = input.indexOf(' ', 1);
        if (space_pos > 0) {
          int new_min = input.substring(1, space_pos).toInt();
          int new_max = input.substring(space_pos + 1).toInt();

          if (new_min < new_max) {
            min_angle = new_min;
            max_angle = new_max;
            updateNumPositions();
            Serial.print("Limits set to: ");
            Serial.print(min_angle);
            Serial.print(" - ");
            Serial.print(max_angle);
            Serial.print(" degrees (");
            Serial.print(num_positions);
            Serial.println(" positions)");
          } else {
            Serial.println("Error: min must be < max");
          }
        } else {
          Serial.println("Usage: l[min] [max] (e.g., l20 100)");
        }
      }
      else if (first_char == 't') {
        // Set step size
        int new_step = input.substring(1).toInt();
        if (new_step > 0 && new_step <= (max_angle - min_angle)) {
          step_size = new_step;
          updateNumPositions();
          Serial.print("Step size set to: ");
          Serial.print(step_size);
          Serial.print(" degrees (");
          Serial.print(num_positions);
          Serial.println(" positions)");
        } else {
          Serial.println("Error: invalid step size");
        }
      }
      else if (first_char == 'g') {
        // Move knob to specified position
        int target = input.substring(1).toInt();
        gauge_position = constrain(target, min_angle, max_angle);
        moving_to_gauge = true;

        Serial.print("Moving to ");
        Serial.print(gauge_position);
        Serial.println(" degrees");
      }
      else if (input == "p") {
        // Toggle position print
        enable_position_print = !enable_position_print;
        Serial.print("Position printing: ");
        Serial.println(enable_position_print ? "enabled" : "disabled");
      }
      else if (first_char == 's') {
        // Set detent strength
        detent_strength = input.substring(1).toFloat();
        detent_strength = constrain(detent_strength, 0, 8);
        Serial.print("Detent strength set to: ");
        Serial.println(detent_strength);
      }
      else if (first_char == 'd') {
        // Set damping
        damping = input.substring(1).toFloat();
        damping = constrain(damping, 0, 2);
        Serial.print("Damping set to: ");
        Serial.println(damping);
      }
      else if (input == "?") {
        // Print current settings
        Serial.println("=== Current Settings ===");
        Serial.print("Current position: ");
        Serial.print(current_position);
        Serial.println(" degrees");
        Serial.print("Limits: ");
        Serial.print(min_angle);
        Serial.print(" - ");
        Serial.print(max_angle);
        Serial.println(" degrees");
        Serial.print("Step size: ");
        Serial.print(step_size);
        Serial.println(" degrees");
        Serial.print("Number of positions: ");
        Serial.println(num_positions);
        Serial.print("Detent strength: ");
        Serial.println(detent_strength);
        Serial.print("Damping: ");
        Serial.println(damping);
        Serial.print("Position printing: ");
        Serial.println(enable_position_print ? "enabled" : "disabled");
      }
    }
  }
}

// Snap angle to nearest detent position
int snapToDetent(int angle) {
  // Find nearest step position relative to min_angle
  int offset = angle - min_angle;
  int step_number = round((float)offset / step_size);
  step_number = constrain(step_number, 0, num_positions - 1);
  return min_angle + (step_number * step_size);
}

// Update number of positions based on current limits and step size
void updateNumPositions() {
  int range = max_angle - min_angle;
  num_positions = (range / step_size) + 1;
}
