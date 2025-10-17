#include <SimpleFOC.h>

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 15, 22, 23);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("SimpleFOC Live PID Tuning");
  Serial.println("Commands:");
  Serial.println("  + / - : increase/decrease target speed");
  Serial.println("  p : increase P");
  Serial.println("  P : decrease P");
  Serial.println("  i : increase I");
  Serial.println("  I : decrease I");
  Serial.println("  d : reverse direction");
  Serial.println("  s : stop motor");
  Serial.println("-----------------------------------");

  Wire.begin(12, 13); // SDA, SCL
  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.sensor_direction = Direction::CW;
  motor.voltage_limit = 5.0;  // 2.0
  motor.velocity_limit = 100; // 20
  motor.controller = MotionControlType::velocity;
  motor.foc_modulation = FOCModulationType::SinePWM;

  // Default PID and filters
  motor.PID_velocity.P = 0.25;
  motor.PID_velocity.I = 0.5;
  motor.PID_velocity.D = 0.0;
  motor.LPF_velocity.Tf = 0.05;
  motor.P_angle.P = 4.0;
  motor.motion_downsample = 10;

  motor.init();
  motor.initFOC();

  Serial.println("FOC ready.");
  printPID();
}

float target_velocity = 5.0;

void loop() {
  motor.loopFOC();
  motor.move(target_velocity);

  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '+': target_velocity += 2; break;
      case '-': target_velocity -= 2; break;
      case 'd': target_velocity = -target_velocity; break;
      case 's': target_velocity = 0; break;
      case 'p': motor.PID_velocity.P *= 1.2; break;
      case 'P': motor.PID_velocity.P /= 1.2; break;
      case 'i': motor.PID_velocity.I *= 1.2; break;
      case 'I': motor.PID_velocity.I /= 1.2; break;
    }
    printPID();
  }
}

void printPID() {
  Serial.print("Target velocity: ");
  Serial.print(target_velocity);
  Serial.print(" | P: ");
  Serial.print(motor.PID_velocity.P, 4);
  Serial.print(" | I: ");
  Serial.println(motor.PID_velocity.I, 4);
}
