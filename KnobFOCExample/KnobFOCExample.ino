#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 15, 22, 23);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// Knob configuration
const float MIN_ANGLE = 0;      // Minimum angle in degrees
const float MAX_ANGLE = 300;    // Maximum angle in degrees  
const float STEP_SIZE = 10;     // Step size in degrees
const int NUM_POSITIONS = 31;   // Total positions (0, 10, 20...300)

float current_position = 0;     // Current absolute position (can go negative or > 360)
float sensor_position = 0;      // Accumulated sensor position
float target_position = 0;      // Target position in degrees
float last_sensor_angle = 0;    // Last raw sensor angle

// Haptic feedback parameters
float detent_strength = 2.0;    // Strength of the detent (0-8)
float end_stop_strength = 4.0;  // Strength of hard stops
float damping = 0.8;            // Damping factor for smooth feel

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== Motor Knob with Detents ===");
  Serial.println("Rotate the motor shaft by hand");
  Serial.println("Steps at every 10 degrees, limits at 0-300 degrees");

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
  static float last_reported_position = -1;
  
  // Read current angle from sensor (in radians)
  float sensor_angle = sensor.getAngle();
  
  // Calculate angle change (handle wraparound at 2*PI)
  float delta = sensor_angle - last_sensor_angle;
  if (delta > _PI) delta -= 2*_PI;
  if (delta < -_PI) delta += 2*_PI;
  
  // Update accumulated position (in radians, can go negative or > 2*PI)
  sensor_position += delta;
  last_sensor_angle = sensor_angle;
  
  // Convert accumulated position to degrees
  current_position = sensor_position * 180.0 / _PI;
  
  // Apply position limits with active snap-back
  if (current_position < MIN_ANGLE) {
    // Below 0 degrees - apply torque to push back to 0
    float error = (MIN_ANGLE - current_position) * _PI / 180.0;
    float stop_torque = end_stop_strength * error;
    motor.move(stop_torque);
    
    // Report position
    if (was_in_bounds) {
      Serial.println("At lower limit (0 degrees)");
      was_in_bounds = false;
      was_out_of_bounds = true;
    }
  }
  else if (current_position > MAX_ANGLE) {
    // Above 300 degrees - apply torque to push back to 300
    float error = (MAX_ANGLE - current_position) * _PI / 180.0;
    float stop_torque = end_stop_strength * error;
    motor.move(stop_torque);
    
    // Report position
    if (was_in_bounds) {
      Serial.println("At upper limit (300 degrees)");
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
    if (abs(current_position - target_position) < 2.0 && 
        target_position != last_reported_position) {
      last_reported_position = target_position;
      Serial.print("Position: ");
      Serial.print(target_position);
      Serial.println(" degrees");
    }
  }
  
  // Optional: Add variable haptic strength based on Serial input
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.startsWith("s")) {
      // Set detent strength (s3.0)
      detent_strength = input.substring(1).toFloat();
      detent_strength = constrain(detent_strength, 0, 8);
      Serial.print("Detent strength set to: ");
      Serial.println(detent_strength);
    }
    else if (input.startsWith("d")) {
      // Set damping (d0.8)
      damping = input.substring(1).toFloat();
      damping = constrain(damping, 0, 2);
      Serial.print("Damping set to: ");
      Serial.println(damping);
    }
    else if (input == "?") {
      // Print current settings
      Serial.println("=== Current Settings ===");
      Serial.print("Position: ");
      Serial.print(current_position);
      Serial.println(" degrees");
      Serial.print("Detent Strength: ");
      Serial.println(detent_strength);
      Serial.print("Damping: ");
      Serial.println(damping);
    }
  }
}

// Snap angle to nearest detent position
float snapToDetent(float angle) {
  // Find nearest step position
  int step_number = round(angle / STEP_SIZE);
  step_number = constrain(step_number, 0, NUM_POSITIONS - 1);
  return step_number * STEP_SIZE;
}