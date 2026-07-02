#include <SimpleFOC.h>
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

// MOONS PG22L45.2-20P020L0-17: native motor step angle 18 deg (before the 1:45 gearhead)
// -> 20 full steps/rev -> 5 pole pairs. Encoder is mounted on the motor rotor shaft
// (ahead of the gearhead), so it reads the same electrical revolutions as pole_pairs below.
StepperMotor motor = StepperMotor(5);

int in1[] = {IN1, IN2};  // Фаза A
int in2[] = {IN3, IN4};  // Фаза B
StepperDriver2PWM driver = StepperDriver2PWM(VREF_A, in1, VREF_B, in2);

// 15 bit, 0x7FFF angle register, spi mode 3, 1MHz
MagneticSensorA1333 sensor = MagneticSensorA1333(SENSOR_CS);
SPIClass SPI_2(SENSOR_MOSI, SENSOR_MISO, SENSOR_SCLK);

float target_velocity = _2PI * 45.2 / 60;

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

  // keep every voltage (= VREF volts) at/under 0.5V -> 0.5A, the motor's rated current
  motor.voltage_sensor_align = 0.5;
  motor.voltage_limit = 0.5;
  motor.controller = MotionControlType::velocity;
  motor.PID_velocity.P = 0.2;
  motor.LPF_velocity.Tf = 0.01;

  motor.useMonitoring(Serial1);
  motor.init();

  // aligns against the raw (uncalibrated) sensor reading every boot - no LUT,
  // no EEPROM caching yet, just closed-loop commutation from the encoder
  motor.initFOC();

  Serial1.println(F("Motor ready."));
}

void loop() {
  motor.loopFOC();
  motor.move(target_velocity);
}
