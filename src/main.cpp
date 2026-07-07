#include <SimpleFOC.h>
#include "encoders/calibrated/CalibratedSensor.h"
#include "./MagneticSensorA1333.h"

#define IN1 PB6   // Фаза A+
#define IN2 PB7   // Фаза A-
#define IN3 PB8   // Фаза B+
#define IN4 PB9   // Фаза B-

#define VREF_A PB0
#define VREF_B PB1

#define SENSOR_MOSI PB15
#define SENSOR_MISO PB14
#define SENSOR_SCLK PB13
#define SENSOR_CS PB12

#define USART1_RX PA10
#define USART1_TX PA9

HardwareSerial Serial1(USART1);

// MOONS PG22L45.2-20P020L0-17: 5 pole pairs, gear ratio 45.2:1 (both confirmed by the
// open-loop 452-rotor-turn -> 10-output-turn test, and again by the single-turn
// backlash check: every turn after the first-after-reversal lands on exactly 2PI).
// Encoder is mounted on the motor rotor shaft, ahead of the gearhead.
StepperMotor motor = StepperMotor(5);

int in1[] = {IN1, IN2};  // Фаза A
int in2[] = {IN3, IN4};  // Фаза B
StepperDriver2PWM driver = StepperDriver2PWM(VREF_A, in1, VREF_B, in2);

const int LUT_SIZE = 256;
float calibrationLut[LUT_SIZE];

// 15 bit, 0x7FFF angle register, spi mode 3, 1MHz
MagneticSensorA1333 sensor = MagneticSensorA1333(SENSOR_CS);
CalibratedSensor sensor_calibrated = CalibratedSensor(sensor, LUT_SIZE, calibrationLut);
SPIClass SPI_2(SENSOR_MOSI, SENSOR_MISO, SENSOR_SCLK);

float target_velocity = _2PI;

void setup() {
  Serial1.begin(19200);

  sensor.init(&SPI_2);
  motor.linkSensor(&sensor);

  motor.foc_modulation = FOCModulationType::SinePWM;

  // VREF_A/B don't drive the phase windings directly - they feed an RC-filtered
  // analog current reference into each A4950 driver's VREF pin (external sense
  // resistor RS = 0.1 ohm on LSS). ITripMax = VREF / (10 * RS), so voltage_power_supply
  // here is the filtered VREF ceiling at 100% PWM duty (STM32 3.3V logic), not the
  // 12V motor supply (VBB), which only powers the bridge outputs to the winding.
  driver.voltage_power_supply = 3.3;
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 0.5;
  motor.voltage_limit = 0.7;
  motor.controller = MotionControlType::velocity;
  motor.PID_velocity.P = 0.1;
  // library default I=10 caused violent windup oscillation on this small motor - keep at 0
  motor.PID_velocity.I = 0;
  // gearbox backlash (~11 deg at the rotor, taken up at each direction reversal) plus
  // the coarse 25-point LUT correction make the raw velocity estimate noisy; heavy
  // filtering keeps the P controller from chasing that noise into repeated tiny
  // reversals that hit the backlash dead zone and self-reinforce into vibration.
  motor.LPF_velocity.Tf = 0.05;

  motor.useMonitoring(Serial1);
  motor.monitor_variables = _MON_TARGET | _MON_VOLT_Q |_MON_VOLT_D | _MON_VEL | _MON_ANGLE;
  // monitor_downsample counts calls to monitor(), not time - we gate the call
  // ourselves every 5s in loop(), so every gated call should actually print
  motor.monitor_downsample = 1;
  motor.init();

  // Experiment: let calibrate() find sensor_direction and zero_electric_angle
  // itself (it internally calls motor.initFOC() on the raw sensor, since both are
  // still UNKNOWN at this point), instead of using our own myAlignSensor(). Also
  // builds the LUT correcting the raw A1333's eccentricity/nonlinearity, same as
  // before.
  sensor_calibrated.voltage_calibration = motor.voltage_sensor_align;
  sensor_calibrated.calibrate(motor, 100);
  Serial1.print(F("sensor_direction: "));
  Serial1.println(motor.sensor_direction == Direction::CW ? "CW" : "CCW");
  Serial1.print(F("zero_electric_angle: "));
  Serial1.println(motor.zero_electric_angle);

  // linking the calibrated sensor to the motor
  motor.linkSensor(&sensor_calibrated);

  // zero_electric_angle/sensor_direction are already known, so this won't re-run
  // the alignment procedure
  motor.initFOC();

  Serial1.print(F("zero_electric_angle: "));
  Serial1.println(motor.zero_electric_angle);
  Serial1.println(motor.sensor_direction == Direction::CW ? "CW" : "CCW");

  Serial1.println(F("Motor ready."));

  _delay(1000);
}

void loop() {
  motor.loopFOC();
  motor.move(target_velocity);

  static unsigned long last_print = 0;
  if (millis() - last_print >= 5000) {
    last_print = millis();
    motor.monitor();
  }
}
