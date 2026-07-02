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

// open-loop test target velocity
float target_velocity = _2PI / 4;  // ~0.25 rev/s

void setup() {
  Serial1.begin(19200);

  // encoder is read out only for verification during this open-loop test,
  // it is not yet linked to the motor
  sensor.init(&SPI_2);

  // VREF_A/B don't drive the phase windings directly - they feed an RC-filtered
  // analog current reference into each A4950 driver's VREF pin (external sense
  // resistor RS = 0.1 ohm on LSS). ITripMax = VREF / (10 * RS), so voltage_power_supply
  // here is the filtered VREF ceiling at 100% PWM duty (STM32 3.3V logic), not the
  // 12V motor supply (VBB), which only powers the bridge outputs to the winding.
  // 0.5A target -> VREF = 0.5A * 10 * 0.1ohm = 0.5V
  driver.voltage_power_supply = 3.3;
  driver.init();
  motor.linkDriver(&driver);

  motor.controller = MotionControlType::velocity_openloop;
  motor.voltage_limit = 0.5;

  motor.useMonitoring(Serial1);
  motor.init();

  Serial1.println(F("Open-loop test ready."));
}

void loop() {
  motor.move(target_velocity);

  sensor.update();
  static unsigned long last_print = 0;
  if (millis() - last_print > 200) {
    last_print = millis();
    Serial1.print(F("sensor angle: "));
    Serial1.println(sensor.getAngle());
  }
}
