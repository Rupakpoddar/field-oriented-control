#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 15, 22, 23);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

float target_angle_deg = 0;   // Target angle in degrees
float target_angle_rad = 0;   // Target angle in radians

// Accumulated sensor position for multi-turn support
float sensor_position = 0;
float last_sensor_angle = 0;

// Control parameters
float position_strength = 5.0;  // Proportional gain for position error
float damping = 0.05;           // Damping factor to prevent oscillations

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== SimpleFOC Smooth Position Control ===");
  Serial.println("Enter target angle in degrees (e.g., 90 or -180):");
  Serial.println("Commands:");
  Serial.println("  [number] : set target angle");
  Serial.println("  s[number]: set position strength (s2.0)");
  Serial.println("  d[number]: set damping (d0.1)");
  Serial.println("  ?        : show current settings");

  Wire.begin(12, 13); // SDA, SCL
  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // Motor configuration
  motor.sensor_direction = Direction::CW;
  motor.voltage_limit = 5.0;
  motor.velocity_limit = 100;
  motor.foc_modulation = FOCModulationType::SinePWM;

  // Use torque control mode for smooth operation
  motor.controller = MotionControlType::torque;

  motor.init();
  motor.initFOC();

  // Initialize position tracking
  last_sensor_angle = sensor.getAngle();
  sensor_position = 0;
  target_angle_rad = 0;

  Serial.println("FOC ready. Motor will hold at 0 degrees.");
  Serial.println("Enter angle commands...");
}

void loop() {
  motor.loopFOC();

  // Read current angle from sensor (in radians)
  float sensor_angle = sensor.getAngle();

  // Calculate angle change (handle wraparound at 2*PI)
  float delta = sensor_angle - last_sensor_angle;
  if (delta > _PI) delta -= 2*_PI;
  if (delta < -_PI) delta += 2*_PI;

  // Update accumulated position (multi-turn capable)
  sensor_position += delta;
  last_sensor_angle = sensor_angle;

  // Calculate position error
  float position_error = target_angle_rad - sensor_position;

  // Create restoring torque proportional to position error
  float position_torque = position_strength * position_error;

  // Add velocity damping to prevent oscillations
  float velocity = motor.shaft_velocity;
  float damping_torque = -damping * velocity;

  // Apply combined torque
  motor.move(position_torque + damping_torque);

  // Read Serial input for target angle
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      if (input.startsWith("s")) {
        // Set position strength
        position_strength = input.substring(1).toFloat();
        position_strength = constrain(position_strength, 0.1, 10.0);
        Serial.print("Position strength set to: ");
        Serial.println(position_strength);
      }
      else if (input.startsWith("d")) {
        // Set damping
        damping = input.substring(1).toFloat();
        damping = constrain(damping, 0, 3.0);
        Serial.print("Damping set to: ");
        Serial.println(damping);
      }
      else if (input == "?") {
        // Print current settings
        Serial.println("=== Current Settings ===");
        Serial.print("Current angle: ");
        Serial.print(sensor_position * 180.0 / _PI);
        Serial.println(" deg");
        Serial.print("Target angle: ");
        Serial.print(target_angle_deg);
        Serial.println(" deg");
        Serial.print("Position strength: ");
        Serial.println(position_strength);
        Serial.print("Damping: ");
        Serial.println(damping);
      }
      else {
        // Set target angle
        target_angle_deg = input.toFloat();
        target_angle_rad = target_angle_deg * _PI / 180.0;
        Serial.print("Moving to angle: ");
        Serial.print(target_angle_deg);
        Serial.println(" deg");
      }
    }
  }
}
