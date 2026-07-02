#include <SimpleFOC.h>
#include "encoders/calibrated/CalibratedSensor.h"
// #include "HarmonicCalibratedSensor.h"
#include "./MagneticSensorA1333.h"
#include <EEPROM.h>

#define IN1 PB6   // Фаза A+
#define IN2 PB7   // Фаза A-
#define IN3 PB8  // Фаза B+
#define IN4 PB9  // Фаза B-

#define VREF_A PB0
#define VREF_B PB1

#define SENSOR_MOSI PB15
#define SENSOR_MISO PB14
#define SENSOR_SCLK PB13
#define SENSOR_CS PB12

#define USART1_RX PA10
#define USART1_TX PA9

// HardwareSerial Serial(USART1_RX, USART1_TX);
HardwareSerial Serial1(USART1);

// fill this array with the calibration values outputed by the calibration procedure
// float calibrationLut[256] = {0};
// float zero_electric_angle = 0;
// Direction sensor_direction = Direction::UNKNOWN;

const bool values_provided = false;

// StepperMotor(  int pp, (optional R, KV))
//  - pp  - pole pair number
//  - R   - phase resistance value - optional
//  - KV  - motor KV rating [rpm/V] - optional
// StepperMotor motor = StepperMotor(50, 1.5, 20.6);
// StepperMotor motor = StepperMotor(50, 2.4);
StepperMotor motor = StepperMotor(50);

//  StepperDriver4PWM( int ph1A,int ph1B,int ph2A,int ph2B, int en1 (optional), int en2 (optional))
//  - ph1A, ph1B - phase 1 pwm pins
//  - ph2A, ph2B - phase 2 pwm pins
//  - en1, en2  - enable pins (optional input)
// StepperDriver4PWM driver = StepperDriver4PWM(IN1, IN2, IN3, IN4);
// StepperDriver4PWM driver = StepperDriver4PWM(IN1, IN2, IN3, IN4);
int in1[] = {IN1, IN2};  // Фаза A
int in2[] = {IN3, IN4};  // Фаза B
StepperDriver2PWM driver = StepperDriver2PWM(VREF_A, in1, VREF_B, in2);
// StepperDriver2PWM driver = StepperDriver2PWM(3, {4,5}, 10, {9,8}, 11, 12);

const int LUT_SIZE = 256;
const uint32_t magic = 0xDEADBEEB;

struct CalibrationData {
  uint32_t magic;       // флаг наличия валидных данных
  float lut[LUT_SIZE];               // калибровочная таблица
  float zero_electric_angle;         // нулевой электрический угол
  Direction sensor_direction;        // направление датчика (enum)
};

CalibrationData calib_data;

// 15 bit, 0x7FFF angle register, spi mode 3, 1MHz
MagneticSensorA1333 sensor = MagneticSensorA1333(SENSOR_CS);
CalibratedSensor sensor_calibrated = CalibratedSensor(sensor, LUT_SIZE, calib_data.lut);
// HarmonicCalibratedSensor sensor_calibrated = HarmonicCalibratedSensor(sensor);

// these are valid pins (mosi, miso, sclk) for 2nd SPI bus on storm32 board (stm32f107rc)
SPIClass SPI_2(SENSOR_MOSI, SENSOR_MISO, SENSOR_SCLK);

// voltage set point variable
float target_voltage = 2;
float target_velocity = _2PI / 30;


void saveCalibrationToEEPROM() {
  EEPROM.put(0, calib_data);
  // EEPROM.commit();
}

bool loadCalibrationFromEEPROM() {
  EEPROM.get(0, calib_data);
  return calib_data.magic == magic;
}



// // instantiate the commander
// Commander command = Commander(Serial);

// void doTarget(char* cmd) { command.scalar(&target_voltage, cmd); }

void setup() {
  Serial1.begin(19200);
  
  // Инициализация энкодера
  sensor.init(&SPI_2);

  // link the sensor to the motor
  motor.linkSensor(&sensor);  
  
  // choose FOC modulation
  motor.foc_modulation = FOCModulationType::SinePWM;
  // set power supply voltage
  driver.voltage_power_supply = 12;
  // // // set driver voltage limit, this phase voltage
  // initialize driver
  driver.init();
  // link driver to motor
  motor.linkDriver(&driver);

  // aligning voltage 
  motor.voltage_sensor_align = 4;
  motor.voltage_limit = 1;
  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;
  // // velocity PI controller parameters
  // // default P=0.5 I = 10
  motor.PID_velocity.P = 0.2;
  // // motor.PID_velocity.I = 20;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  // motor.PID_velocity.output_ramp = 10;
  // // set motor voltage limit, this limits Vq
  // motor.voltage_limit = 6;

  // // velocity low pass filtering
  // // default 5ms - try different values to see what is the best. 
  // // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01;

  // // angle loop controller
  // // motor.P_angle.P = 20;
  // // angle loop velocity limit
  // motor.velocity_limit = 10;

  // use monitoring
  // motor.useMonitoring(debugMonitor);
  motor.useMonitoring(Serial1);
  // motor.monitor_variables =  _MON_VEL; 
  // motor.monitor_downsample = 100; // default 10

  // initialize motor
  motor.init();

  if(loadCalibrationFromEEPROM()) {
    motor.zero_electric_angle = calib_data.zero_electric_angle;
    motor.sensor_direction = calib_data.sensor_direction;
    // 	// Display the LUT
    // motor.monitor_port->print("float calibrationLut[");
    // motor.monitor_port->print(LUT_SIZE);
    // motor.monitor_port->println("] = {");
    // _delay(100); 
    // for (int i=0;i < LUT_SIZE; i++){
    //   motor.monitor_port->print(calib_data.lut[i],6);
    //   if(i < LUT_SIZE - 1) motor.monitor_port->print(", ");
    //   _delay(1);
    // }
    // motor.monitor_port->println("};");

  } else {
    // Running calibration
    sensor_calibrated.calibrate(motor, 100);
    calib_data.zero_electric_angle = motor.zero_electric_angle;
    calib_data.sensor_direction = motor.sensor_direction;
    calib_data.magic = magic;
    saveCalibrationToEEPROM();
  }

  // Linking sensor to motor object
  motor.linkSensor(&sensor_calibrated);

  // calibrated init FOC
  motor.initFOC();

  // // add target command T
  // command.add('T', doTarget, "target voltage");
  
  Serial1.println(F("Motor ready."));

  Serial1.println(F("Set the target voltage using serial terminal:"));
  _delay(1000);
}

void loop() {

  motor.loopFOC();
  motor.move(target_velocity);
  // command.run();
  // motor.monitor();
 
}
//   // set voltage to run calibration
//   sensor_calibrated.voltage_calibration = 2;
//   // Running calibration
//   sensor_calibrated.calibrate(motor); 

//   // Linking sensor to motor object
//   motor.linkSensor(&sensor_calibrated);
//   // // // initialize current sensing and link it to the motor
//   // // // https://docs.simplefoc.com/inline_current_sense#where-to-place-the-current_sense-configuration-in-your-foc-code
//   // current_sense.init();
//   // motor.linkCurrentSense(&current_sense);

//   // align sensor and start FOC
//   motor.initFOC();

//   motor.target = _2PI;

//   _delay(1000);
  
//   // mySerial.println("Sensor ready");
//   // _delay(1000);

// }

// // velocity set point variable
// float target_velocity = 4; // 2Rad/s ~ 20rpm

// void loop() {
//   // main FOC algorithm function
//   // motor.loopFOC();

//   // Motion control function
//   motor.move();
// }
// void loop() {
//   sensor.update();
//   mySerial.println(sensor.getAngle());
//   delay(100);
// }

// #include <Arduino.h>
// #include <SPI.h>
// #include <SoftwareSerial.h>


// // SPI2: PB15 = MOSI, PB14 = MISO, PB13 = SCK
// SPIClass SPI_2(PB15, PB14, PB13);
// const int CS_PIN = PB12;
// SoftwareSerial mySerial(PA6, PA7);  // PA6 = RX, PA7 = TX

// void setup() {
//   mySerial.begin(9600);
//   delay(500);

//   // Настройка пина CS
//   pinMode(CS_PIN, OUTPUT);
//   digitalWrite(CS_PIN, HIGH);

//   // Инициализация SPI в режиме Mode 3
//   SPI_2.begin();
//   SPI_2.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

//   mySerial.println("A1333 raw read test");
// }

// void loop() {
//   // Считываем 16-битное значение из регистра 0x20
//   uint16_t reg = 0x2000; // 0x20 << 8 | 0x00 (READ)
//   uint16_t raw = 0;

//   digitalWrite(CS_PIN, LOW);
//   delayMicroseconds(2);
//   SPI_2.transfer16(reg);     // Команда чтения
//   digitalWrite(CS_PIN, HIGH);

//   delayMicroseconds(2);

//   digitalWrite(CS_PIN, LOW);
//   delayMicroseconds(2);
//   raw = SPI_2.transfer16(0x0000);  // Считываем ответ
//   digitalWrite(CS_PIN, HIGH);

//   mySerial.print("Raw register 0x20: 0x");
//   mySerial.println(raw, HEX);

//   uint16_t angle = raw & 0x0FFF;  // Только младшие 12 бит — угол
//   float angle_rad = (float)angle / 4096.0f * 6.28;

//   mySerial.print("Angle (rad): ");
//   mySerial.println(angle_rad, 4);

//   delay(200);
// }



// #undef Serial
// #define Serial Serial1

// #include <SimpleFOC.h>
// #include "./MagneticSensorA1333.h"
// #include "./CalibratedSensor.h"
// // #include <SoftwareSerial.h>

// #define IN1 PB6   // Фаза A+
// #define IN2 PB7   // Фаза A-
// #define IN3 PB8  // Фаза B+
// #define IN4 PB9  // Фаза B-
// // #define EN_PIN PA5

// #define VREF_A PB0
// #define VREF_B PB1

// #define SENSOR_MOSI PB15
// #define SENSOR_MISO PB14
// #define SENSOR_SCLK PB13
// #define SENSOR_CS PB12

// #define mySerial_RX PA6
// #define mySerial_TX PA7
// #define USART1_RX PA10
// #define USART1_TX PA9

// // HardwareSerial Serial(USART1_RX, USART1_TX);
// HardwareSerial Serial(USART1);

// // StepperMotor(  int pp, (optional R, KV))
// //  - pp  - pole pair number
// //  - R   - phase resistance value - optional
// //  - KV  - motor KV rating [rpm/V] - optional
// // StepperMotor motor = StepperMotor(50, 1.5, 20.6);
// // StepperMotor motor = StepperMotor(50, 2.4);
// StepperMotor motor = StepperMotor(50);

// //  StepperDriver4PWM( int ph1A,int ph1B,int ph2A,int ph2B, int en1 (optional), int en2 (optional))
// //  - ph1A, ph1B - phase 1 pwm pins
// //  - ph2A, ph2B - phase 2 pwm pins
// //  - en1, en2  - enable pins (optional input)
// // StepperDriver4PWM driver = StepperDriver4PWM(IN1, IN2, IN3, IN4);
// // StepperDriver4PWM driver = StepperDriver4PWM(IN1, IN2, IN3, IN4);
// int in1[] = {IN1, IN2};  // Фаза A
// int in2[] = {IN3, IN4};  // Фаза B
// StepperDriver2PWM driver = StepperDriver2PWM(VREF_A, in1, VREF_B, in2);
// // StepperDriver2PWM driver = StepperDriver2PWM(3, {4,5}, 10, {9,8}, 11, 12);

// // 15 bit, 0x7FFF angle register, spi mode 3, 1MHz
// MagneticSensorA1333 sensor = MagneticSensorA1333(SENSOR_CS);
// CalibratedSensor sensor_calibrated = CalibratedSensor(sensor);

// // these are valid pins (mosi, miso, sclk) for 2nd SPI bus on storm32 board (stm32f107rc)
// SPIClass SPI_2(SENSOR_MOSI, SENSOR_MISO, SENSOR_SCLK);


// void setup() {
//   Serial.begin(19200);
  
//   // Инициализация энкодера
//   sensor.init(&SPI_2);

//   // link the sensor to the motor
//   motor.linkSensor(&sensor);  
  
//   // choose FOC modulation
//   motor.foc_modulation = FOCModulationType::SinePWM;
//   // set power supply voltage
//   driver.voltage_power_supply = 12;
//   // // // set driver voltage limit, this phase voltage
//   // initialize driver
//   driver.init();
//   // link driver to motor
//   motor.linkDriver(&driver);

//   // set motion control type to velocity
//   motor.controller = MotionControlType::velocity;

//   // velocity PI controller parameters
//   // default P=0.5 I = 10
//   motor.PID_velocity.P = 0.2;
//   // motor.PID_velocity.I = 20;
//   // set motor voltage limit, this limits Vq
//   motor.voltage_limit = 6;

//   // velocity low pass filtering
//   // default 5ms - try different values to see what is the best. 
//   // the lower the less filtered
//   motor.LPF_velocity.Tf = 0.01;

//   // angle loop controller
//   // motor.P_angle.P = 20;
//   // angle loop velocity limit
//   motor.velocity_limit = 10;

//   // use monitoring
//   // motor.useMonitoring(debugMonitor);
//   motor.useMonitoring(Serial);

//   // initialize motor
//   motor.init();

//   // set voltage to run calibration
//   sensor_calibrated.voltage_calibration = 2;
//   // Running calibration
//   sensor_calibrated.calibrate(motor); 

//   // Linking sensor to motor object
//   motor.linkSensor(&sensor_calibrated);
//   // // // initialize current sensing and link it to the motor
//   // // // https://docs.simplefoc.com/inline_current_sense#where-to-place-the-current_sense-configuration-in-your-foc-code
//   // current_sense.init();
//   // motor.linkCurrentSense(&current_sense);

//   // align sensor and start FOC
//   motor.initFOC();

//   motor.target = _2PI;

//   _delay(1000);
  
//   // mySerial.println("Sensor ready");
//   // _delay(1000);

// }

// // // velocity set point variable
// // float target_velocity = 4; // 2Rad/s ~ 20rpm

// void loop() {
//   // main FOC algorithm function
//   // motor.loopFOC();

//   // Motion control function
//   motor.move();
// }
// // void loop() {
// //   sensor.update();
// //   mySerial.println(sensor.getAngle());
// //   delay(100);
// // }

// // #include <Arduino.h>
// // #include <SPI.h>
// // #include <SoftwareSerial.h>


// // // SPI2: PB15 = MOSI, PB14 = MISO, PB13 = SCK
// // SPIClass SPI_2(PB15, PB14, PB13);
// // const int CS_PIN = PB12;
// // SoftwareSerial mySerial(PA6, PA7);  // PA6 = RX, PA7 = TX

// // void setup() {
// //   mySerial.begin(9600);
// //   delay(500);

// //   // Настройка пина CS
// //   pinMode(CS_PIN, OUTPUT);
// //   digitalWrite(CS_PIN, HIGH);

// //   // Инициализация SPI в режиме Mode 3
// //   SPI_2.begin();
// //   SPI_2.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

// //   mySerial.println("A1333 raw read test");
// // }

// // void loop() {
// //   // Считываем 16-битное значение из регистра 0x20
// //   uint16_t reg = 0x2000; // 0x20 << 8 | 0x00 (READ)
// //   uint16_t raw = 0;

// //   digitalWrite(CS_PIN, LOW);
// //   delayMicroseconds(2);
// //   SPI_2.transfer16(reg);     // Команда чтения
// //   digitalWrite(CS_PIN, HIGH);

// //   delayMicroseconds(2);

// //   digitalWrite(CS_PIN, LOW);
// //   delayMicroseconds(2);
// //   raw = SPI_2.transfer16(0x0000);  // Считываем ответ
// //   digitalWrite(CS_PIN, HIGH);

// //   mySerial.print("Raw register 0x20: 0x");
// //   mySerial.println(raw, HEX);

// //   uint16_t angle = raw & 0x0FFF;  // Только младшие 12 бит — угол
// //   float angle_rad = (float)angle / 4096.0f * 6.28;

// //   mySerial.print("Angle (rad): ");
// //   mySerial.println(angle_rad, 4);

// //   delay(200);
// // }
