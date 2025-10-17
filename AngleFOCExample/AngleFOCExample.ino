#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 15, 22, 23);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

float target_angle_deg = 0;   // Target angle in degrees
float target_angle_rad = 0;   // Target angle in radians

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== SimpleFOC Position Control ===");
  Serial.println("Enter target angle in degrees (e.g., 90):");

  Wire.begin(12, 13); // SDA, SCL
  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  // Motor configuration
  motor.sensor_direction = Direction::CW;
  motor.voltage_limit = 5.0;
  motor.velocity_limit = 50; // deg/sec equivalent
  motor.foc_modulation = FOCModulationType::SinePWM;

  // Use position control mode
  motor.controller = MotionControlType::angle;

  // PID tuning for position hold
  motor.P_angle.P = 4.0;
  motor.P_angle.I = 0.0;
  motor.P_angle.D = 0.0;
  motor.LPF_angle.Tf = 0.01;

  // Default PID and filters
  motor.PID_velocity.P = 0.25;
  motor.PID_velocity.I = 0.5;
  motor.PID_velocity.D = 0.0;
  motor.LPF_velocity.Tf = 0.05;
  motor.motion_downsample = 10;

  motor.init();
  motor.initFOC();

  Serial.println("FOC ready. Waiting for angle commands...");
}

void loop() {
  motor.loopFOC();
  motor.move(target_angle_rad);

  // Read Serial input for target angle
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      target_angle_deg = input.toFloat();
      target_angle_rad = target_angle_deg * _PI / 180.0;  // convert to radians
      Serial.print("Moving to angle: ");
      Serial.print(target_angle_deg);
      Serial.println(" deg");
    }
  }
}
