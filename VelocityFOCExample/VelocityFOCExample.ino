#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 15, 22, 23);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

float target_velocity = 0;  // Target velocity in rad/s

// Control parameters
float velocity_voltage = 0.1;    // Base voltage per rad/s (feed-forward)
float correction_gain = 0.2;     // Feedback correction gain
float acceleration_limit = 10.0; // Smooth acceleration (rad/s²)
float reverse_compensation = 1.0; // Multiplier for reverse direction (adjust if asymmetric)

// State tracking
float smoothed_target = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== SimpleFOC Smooth Velocity Control ===");
  Serial.println("Commands:");
  Serial.println("  + / -    : increase/decrease speed by 5 rad/s");
  Serial.println("  [number] : set target speed in rad/s (can be negative)");
  Serial.println("  v[number]: set velocity voltage (v0.1)");
  Serial.println("  c[number]: set correction gain (c0.2)");
  Serial.println("  a[number]: set acceleration limit (a10.0)");
  Serial.println("  m[number]: set reverse compensation (m1.0)");
  Serial.println("  r        : reverse direction");
  Serial.println("  s        : stop motor");
  Serial.println("  ?        : show current settings");
  Serial.println("-----------------------------------");

  Wire.begin(12, 13); // SDA, SCL
  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.sensor_direction = Direction::CW;
  motor.voltage_limit = 6.0;
  motor.velocity_limit = 100;
  motor.foc_modulation = FOCModulationType::SinePWM;

  // Use torque/voltage control mode
  motor.controller = MotionControlType::torque;

  motor.init();
  motor.initFOC();

  Serial.println("FOC ready.");
  Serial.println("Enter velocity commands (rad/s)...");
}

void loop() {
  motor.loopFOC();

  // Smooth acceleration - gradually approach target velocity
  float target_delta = target_velocity - smoothed_target;
  float max_change = acceleration_limit * 0.001; // Assuming ~1ms loop time

  if (abs(target_delta) < max_change) {
    smoothed_target = target_velocity;
  } else {
    smoothed_target += (target_delta > 0) ? max_change : -max_change;
  }

  // Get current velocity from motor
  float current_velocity = motor.shaft_velocity;

  // Calculate velocity error
  float velocity_error = smoothed_target - current_velocity;

  // If target is zero and we're close to stopped, apply no voltage to prevent heating
  if (abs(smoothed_target) < 0.5 && abs(current_velocity) < 1.0) {
    motor.move(0);
  }
  else {
    // Apply reverse compensation if moving in negative direction
    float effective_velocity_voltage = velocity_voltage;
    if (smoothed_target < 0) {
      effective_velocity_voltage *= reverse_compensation;
    }

    // Feed-forward term: base voltage proportional to target velocity
    float feed_forward = effective_velocity_voltage * smoothed_target;

    // Feedback term: correction based on error
    float feedback = correction_gain * velocity_error;

    // Combined voltage
    float total_voltage = feed_forward + feedback;

    // Apply voltage as torque
    motor.move(total_voltage);
  }

  // Handle Serial commands
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      char first_char = input.charAt(0);

      // Check for single character commands first
      if (input == "+") {
        target_velocity += 5;
        Serial.print("Target velocity: ");
        Serial.print(target_velocity);
        Serial.println(" rad/s");
      }
      else if (input == "-") {
        target_velocity -= 5;
        Serial.print("Target velocity: ");
        Serial.print(target_velocity);
        Serial.println(" rad/s");
      }
      else if (input == "r") {
        target_velocity = -target_velocity;
        Serial.print("Direction reversed. Target: ");
        Serial.print(target_velocity);
        Serial.println(" rad/s");
      }
      else if (input == "s" || input == "0") {
        target_velocity = 0;
        Serial.println("Stopping motor...");
      }
      else if (input == "?") {
        // Print current settings
        Serial.println("=== Current Settings ===");
        Serial.print("Current velocity: ");
        Serial.print(current_velocity);
        Serial.println(" rad/s");
        Serial.print("Target velocity: ");
        Serial.print(target_velocity);
        Serial.println(" rad/s");
        Serial.print("Velocity voltage: ");
        Serial.println(velocity_voltage);
        Serial.print("Correction gain: ");
        Serial.println(correction_gain);
        Serial.print("Acceleration limit: ");
        Serial.print(acceleration_limit);
        Serial.println(" rad/s²");
        Serial.print("Reverse compensation: ");
        Serial.println(reverse_compensation);
      }
      else if (first_char == 'v') {
        // Set velocity voltage (feed-forward gain)
        velocity_voltage = input.substring(1).toFloat();
        velocity_voltage = constrain(velocity_voltage, 0.01, 0.5);
        Serial.print("Velocity voltage set to: ");
        Serial.println(velocity_voltage);
      }
      else if (first_char == 'c') {
        // Set correction gain
        correction_gain = input.substring(1).toFloat();
        correction_gain = constrain(correction_gain, 0.0, 2.0);
        Serial.print("Correction gain set to: ");
        Serial.println(correction_gain);
      }
      else if (first_char == 'a') {
        // Set acceleration limit
        acceleration_limit = input.substring(1).toFloat();
        acceleration_limit = constrain(acceleration_limit, 0.5, 50.0);
        Serial.print("Acceleration limit set to: ");
        Serial.print(acceleration_limit);
        Serial.println(" rad/s²");
      }
      else if (first_char == 'm') {
        // Set reverse compensation
        reverse_compensation = input.substring(1).toFloat();
        reverse_compensation = constrain(reverse_compensation, 0.5, 2.0);
        Serial.print("Reverse compensation set to: ");
        Serial.println(reverse_compensation);
      }
      else {
        // Parse as target velocity (handles negative numbers properly)
        float new_target = input.toFloat();
        if (new_target != 0 || input == "0" || input.startsWith("0")) {
          target_velocity = new_target;
          Serial.print("Target velocity set to: ");
          Serial.print(target_velocity);
          Serial.println(" rad/s");
        }
      }
    }
  }
}
