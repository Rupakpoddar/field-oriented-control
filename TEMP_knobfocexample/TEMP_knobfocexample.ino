// TEMP_knobfocexample.ino - Copy of KnobFOCExample with serial input for step_size
#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 15, 22, 23);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

const float MIN_ANGLE = 0;      // Minimum angle in degrees
const float MAX_ANGLE = 300;    // Maximum angle in degrees
float step_size = 10;           // Step size in degrees (modifiable)
int num_positions = 31;         // Total positions (calculated)

float current_position = 0;
float sensor_position = 0;
float target_position = 0;
float last_sensor_angle = 0;

float detent_strength = 2.0;
float end_stop_strength = 4.0;
float damping = 0.8;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== TEMP Motor Knob with Detents ===");
  Serial.println("Send s<number> for detent strength, d<number> for damping, a<number> for step size");

  Wire.begin(12, 13); // SDA, SCL
  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.sensor_direction = Direction::CW;
  motor.voltage_limit = 3.0;
  motor.velocity_limit = 100;
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.controller = MotionControlType::torque;

  motor.init();
  motor.initFOC();

  last_sensor_angle = sensor.getAngle();
  sensor_position = 0;
  current_position = 0;
  target_position = 0;

  num_positions = (int)((MAX_ANGLE - MIN_ANGLE) / step_size) + 1;

  Serial.println("Knob ready!");
  Serial.print("Initial position: ");
  Serial.print(target_position);
  Serial.println(" degrees");
  Serial.print("Step size: ");
  Serial.println(step_size);
  // Serial.print(", Num positions: ");
  // Serial.println(num_positions);
}

void loop() {
  motor.loopFOC();

  static bool was_in_bounds = true;
  static bool was_out_of_bounds = false;
  static float last_reported_position = -1;

  float sensor_angle = sensor.getAngle();
  float delta = sensor_angle - last_sensor_angle;
  if (delta > _PI) delta -= 2*_PI;
  if (delta < -_PI) delta += 2*_PI;
  sensor_position += delta;
  last_sensor_angle = sensor_angle;
  current_position = sensor_position * 180.0 / _PI;

  if (current_position < MIN_ANGLE) {
    float error = (MIN_ANGLE - current_position) * _PI / 180.0;
    float stop_torque = end_stop_strength * error;
    motor.move(stop_torque);
    if (was_in_bounds) {
      Serial.println("At lower limit (0 degrees)");
      was_in_bounds = false;
      was_out_of_bounds = true;
    }
  }
  else if (current_position > MAX_ANGLE) {
    float error = (MAX_ANGLE - current_position) * _PI / 180.0;
    float stop_torque = end_stop_strength * error;
    motor.move(stop_torque);
    if (was_in_bounds) {
      Serial.println("At upper limit (300 degrees)");
      was_in_bounds = false;
      was_out_of_bounds = true;
    }
  }
  else {
    target_position = snapToDetent(current_position);
    float position_error = (target_position - current_position) * _PI / 180.0;
    float detent_torque = detent_strength * position_error;
    float velocity = motor.shaft_velocity;
    float damping_torque = -damping * velocity * 0.01;
    motor.move(detent_torque + damping_torque);
    if (was_out_of_bounds) {
      was_out_of_bounds = false;
      was_in_bounds = true;
    }
    if (abs(current_position - target_position) < 2.0 && target_position != last_reported_position) {
      last_reported_position = target_position;
      Serial.print("Position: ");
      Serial.print(target_position);
      Serial.print(" degrees, Step size: ");
      Serial.println(step_size);
      // Serial.print(", Num positions: ");
      // Serial.println(num_positions);
    }
  }

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.startsWith("s")) {
      detent_strength = input.substring(1).toFloat();
      detent_strength = constrain(detent_strength, 0, 8);
      Serial.print("Detent strength set to: ");
      Serial.println(detent_strength);
    }
    else if (input.startsWith("d")) {
      damping = input.substring(1).toFloat();
      damping = constrain(damping, 0, 2);
      Serial.print("Damping set to: ");
      Serial.println(damping);
    }
    else if (input.startsWith("a")) {
      float new_step = input.substring(1).toFloat();
      if (new_step > 0 && new_step <= (MAX_ANGLE - MIN_ANGLE)) {
        step_size = new_step;
        num_positions = (int)((MAX_ANGLE - MIN_ANGLE) / step_size) + 1;
        Serial.print("Step size set to: ");
        Serial.println(step_size);
        // Serial.print(", Num positions: ");
        // Serial.println(num_positions);
      } else {
        Serial.println("Invalid step size");
      }
    }
    else if (input == "?") {
      Serial.println("=== Current Settings ===");
      Serial.print("Position: ");
      Serial.print(current_position);
      Serial.println(" degrees");
      Serial.print("Detent Strength: ");
      Serial.println(detent_strength);
      Serial.print("Damping: ");
      Serial.println(damping);
      Serial.print("Step size: ");
      Serial.println(step_size);
      // Serial.print(", Num positions: ");
      // Serial.println(num_positions);
    }
  }
}

float snapToDetent(float angle) {
  int step_number = round((angle - MIN_ANGLE) / step_size);
  step_number = constrain(step_number, 0, num_positions - 1);
  return MIN_ANGLE + step_number * step_size;
}
