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

// Rotates the electrical angle through `revolutions` full electrical revolutions
// (500 substeps each, 2ms/substep), starting from `start_angle`, in direction
// `sign` (+1 or -1). Returns the electrical angle reached, so callers can keep
// accumulating from a continuous reference instead of jumping back to 0 each time.
float spinElectricalRevolutions(StepperMotor &motor, Sensor &raw_sensor,
                                 float start_angle, float sign, int revolutions) {
  float elec_angle = start_angle;
  for (int rev = 0; rev < revolutions; rev++) {
    for (int i = 0; i <= 500; i++) {
      motor.setPhaseVoltage(motor.voltage_sensor_align, 0, elec_angle + sign * _2PI * i / 500.0f);
      raw_sensor.update();
      _delay(2);
    }
    elec_angle += sign * _2PI;
  }
  return elec_angle;
}

// Custom replacement for StepperMotor::alignSensor() - determines sensor_direction
// and zero_electric_angle directly on `motor`, and is meant to be called *before*
// motor.initFOC() so that call sees both already set and skips its own alignment.
//
// Direction: one electrical revolution forward, compare mechanical position
// before/after by sign - same idea as the library's own method, just standalone,
// using getAngle() (not getMechanicalAngle()) so a random starting position near
// the wrap boundary can't flip the comparison and misdetect direction.
//
// Two idle full rotor revolutions (n_pole_pairs electrical revolutions each) are
// spun before any measurement is taken: one right at the start (moves off a cold,
// possibly stuck starting position) and one right after the direction is found
// (in that direction, so gearbox backlash is fully taken up before the zero-angle
// measurements start, instead of contaminating the first sample like it did with
// the earlier sweep-based approach).
//
// Zero angle: repeats initFOC()'s own single-point method (hold a *fixed*
// electrical angle long enough to fully settle, then read the sensor once) at
// n_samples different reference angles spread around the circle, and averages
// with proper circular mean (sin/cos, not naive angle averaging). This is the
// only formula that's actually been confirmed to spin the motor - the sweep-based
// formula from the library's calibrate() (with its extra +PI/2 term, presumably
// assuming a steady-state torque angle during continuous motion) gave a different,
// non-working zero angle on every attempt regardless of how the samples were
// filtered/skipped, so the formula itself - not sampling noise - was the problem.
void myAlignSensor(StepperMotor &motor, Sensor &raw_sensor, int n_samples, int settle_ms) {
  const int NPP = motor.pole_pairs;
  float elec_angle = 0;

  // idle rotor revolution right at the start
  elec_angle = spinElectricalRevolutions(motor, raw_sensor, elec_angle, 1, NPP);
  // motor.setPhaseVoltage(0, 0, 0);
  _delay(200);

  // --- direction ---
  raw_sensor.update();
  float before = raw_sensor.getAngle();
  elec_angle = spinElectricalRevolutions(motor, raw_sensor, elec_angle, 1, 1);
  _delay(200);
  raw_sensor.update();
  float after = raw_sensor.getAngle();
  // library convention (StepperMotor::alignSensor): forward electrical motion
  // increasing the mechanical angle means CW, decreasing means CCW
  motor.sensor_direction = (after > before) ? Direction::CW : Direction::CCW;
  Serial1.print(F("direction sweep: before="));
  Serial1.print(before);
  Serial1.print(F(" after="));
  Serial1.println(after);
  // motor.setPhaseVoltage(0, 0, 0);
  _delay(200);

  // idle rotor revolution again, now in the just-determined direction
  float dir_sign = (motor.sensor_direction == Direction::CW) ? 1.0f : -1.0f;
  elec_angle = spinElectricalRevolutions(motor, raw_sensor, elec_angle, dir_sign, NPP);
  // motor.setPhaseVoltage(0, 0, 0);
  _delay(200);

  // --- zero electric angle ---
  float sum_sin = 0, sum_cos = 0;
  for (int i = 0; i < n_samples; i++) {
    float reference_angle = _3PI_2 + i * (_2PI / n_samples);
    motor.setPhaseVoltage(motor.voltage_sensor_align, 0, reference_angle);
    _delay(settle_ms);
    raw_sensor.update();
    // setPhaseVoltage(Uq, 0, angle_el) puts the applied voltage vector at
    // angle_el + PI/2 (inverse Park transform), so the rotor settles at
    // reference_angle + PI/2, not at reference_angle itself
    float zero_angle = _normalizeAngle((int)motor.sensor_direction * NPP * raw_sensor.getAngle()
                                        - reference_angle - _PI_2);
    sum_sin += sin(zero_angle);
    sum_cos += cos(zero_angle);
  }
  motor.setPhaseVoltage(0, 0, 0);
  motor.zero_electric_angle = _normalizeAngle(atan2(sum_sin, sum_cos));
}

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

  // determines sensor_direction and zero_electric_angle ourselves, before any
  // initFOC() call - 8 static holds spread around the circle, 700ms settle each
  myAlignSensor(motor, sensor, 8, 700);
  Serial1.print(F("sensor_direction: "));
  Serial1.println(motor.sensor_direction == Direction::CW ? "CW" : "CCW");
  Serial1.print(F("zero_electric_angle: "));
  Serial1.println(motor.zero_electric_angle);
  Direction known_good_direction = motor.sensor_direction;
  float known_good_zero_angle = motor.zero_electric_angle;

  // builds a LUT correcting the raw A1333's eccentricity/nonlinearity. calibrate()
  // also recomputes its own zero_electric_angle/sensor_direction, but we trust our
  // own myAlignSensor() result more (see its header comment) - keep only the LUT
  // and restore our alignment afterwards.
  sensor_calibrated.voltage_calibration = motor.voltage_sensor_align;
  sensor_calibrated.calibrate(motor, 100);
  motor.zero_electric_angle = known_good_zero_angle;
  motor.sensor_direction = known_good_direction;

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
