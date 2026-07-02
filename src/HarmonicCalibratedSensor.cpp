#include "HarmonicCalibratedSensor.h"

  // wrapped        - исходный датчик (например, MagneticSensorI2C)
  // n_samples      - кол-во опорных выборок по одному мех. обороту (кратно NPP)
  // n_harmonics    - число гармоник в Фурье-аппроксимации (K)
  HarmonicCalibratedSensor::HarmonicCalibratedSensor(Sensor &wrapped, int n_samples, int n_harmonics)
      : _wrapped(wrapped), N(n_samples), K(n_harmonics) {
    a = new float[K + 1];   // a[0] = среднее (DC), a[1..K]
    b = new float[K + 1];   // b[0] не используется
    memset(a, 0, (K + 1) * sizeof(float));
    memset(b, 0, (K + 1) * sizeof(float));
  }


HarmonicCalibratedSensor::~HarmonicCalibratedSensor() {
    delete[] a;
    delete[] b;
};

// call update of calibrated sensor
void HarmonicCalibratedSensor::update()
{
	_wrapped.update();
	this->Sensor::update();
};

// Retrieve the calibrated sensor angle
void HarmonicCalibratedSensor::init()
{
	// assume wrapped sensor has already been initialized
	this->Sensor::init(); // call superclass init
}

// Retrieve the calibrated sensor angle
float HarmonicCalibratedSensor::getSensorAngle()
{
	if(!coef_ready) {
		return _wrapped.getMechanicalAngle();
	}
    // raw encoder position e.g. 0-2PI
    float raw = fmodf(_wrapped.getMechanicalAngle(), _2PI);
    raw += (raw < 0) ? _2PI : 0;

    // θ_cal = θ_raw - (a0 + sum_k a_k sin(kθ) + b_k cos(kθ))
    float corr = a[0];
    for (int k = 1; k <= K; k++) {
      float ktheta = k * raw;
      corr += a[k] * sinf(ktheta) + b[k] * cosf(ktheta);
    }
    float res = raw - corr;
    return _normalizeAngle(res);

	// float raw_angle = fmodf(_wrapped.getMechanicalAngle(), _2PI);
    // raw_angle += raw_angle < 0 ? _2PI:0;

    // // Calculate the resolution of the LUT in radians
    // float lut_resolution = _2PI / n_lut;
    // // Calculate LUT index
    // int lut_index = raw_angle / lut_resolution;

    // // Get calibration values from the LUT
    // float y0 = calibrationLut[lut_index];
    // float y1 = calibrationLut[(lut_index + 1) % n_lut];

    // // Linearly interpolate between the y0 and y1 values
    // // Calculate the relative distance from the y0 (raw_angle has to be between y0 and y1)
    // // If distance = 0, interpolated offset = y0
    // // If distance = 1, interpolated offset = y1
    // float distance = (raw_angle - lut_index * lut_resolution) / lut_resolution;
    // float offset = (1 - distance) * y0 + distance * y1;

    // // Calculate the calibrated angle
    // return raw_angle - offset;
}

// // Perform filtering to linearize position sensor eccentricity
// // FIR n-sample average, where n = number of samples in the window
// // This filter has zero gain at electrical frequency and all integer multiples
// // So cogging effects should be completely filtered out
// void HarmonicCalibratedSensor::filter_error(float* error, float &error_mean, int n_ticks, int window){
// 	float window_buffer[window];
// 	memset(window_buffer, 0, window*sizeof(float));
// 	float window_sum = 0;
// 	int buffer_index = 0;
// 	// fill the inital window buffer
// 	for (int i = 0; i < window; i++) {
// 		int ind = n_ticks - window/2 -1 + i;
// 		window_buffer[i] = error[ind % n_ticks];
// 		window_sum += window_buffer[i];
// 	}
// 	// calculate the moving average
// 	error_mean = 0;
// 	for (int i = 0; i < n_ticks; i++)
// 	{
// 		// Update buffer
// 		window_sum -= window_buffer[buffer_index];
// 		window_buffer[buffer_index] = error[( i + window/2 ) %n_ticks];
// 		window_sum += window_buffer[buffer_index];
// 		// update the buffer index
// 		buffer_index = (buffer_index + 1) % window;

// 		// Update filtered error
// 		error[i] = window_sum / (float)window;
// 		// update the mean value
// 		error_mean += error[i] / n_ticks;
// 	}

// }

// void HarmonicCalibratedSensor::calibrate(FOCMotor &motor, int settle_time_ms)
// {
// 	// if the LUT is already defined, skip the calibration

// 	if(calibrationLut == NULL) {
// 		allocated = true;
// 		calibrationLut = new float[n_lut];
// 	}
// 	motor.monitor_port->println("Starting Sensor Calibration.");

// 	// Calibration variables
	
//     // Init inital angles
//     float theta_actual = 0.0;
//     float avg_elec_angle = 0.0;
// 	// set the inital electric angle to 0
//     float elec_angle = 0.0;

// 	// Calibration parameters
// 	// The motor will take a n_pos samples per electrical cycle
// 	// which amounts to n_ticks (n_pos * motor.pole_pairs) samples per mechanical rotation
// 	// Additionally, the motor will take n2_ticks steps to reach any of the n_ticks posiitons
// 	// incrementing the electrical angle by deltaElectricalAngle each time
// 	int n_pos = 5;
// 	int _NPP = motor.pole_pairs;								      // number of pole pairs which is user input
// 	const int n_ticks = n_pos * _NPP;							      // number of positions to be sampled per mechanical rotation.  Multiple of NPP for filtering reasons (see later)
// 	const int n2_ticks = 5;										      // increments between saved samples (for smoothing motion)
// 	float deltaElectricalAngle = _2PI * _NPP / (n_ticks * n2_ticks);  // Electrical Angle increments for calibration steps
// 	float error[n_ticks];	         						  		  // pointer to error array (average of forward & backward)
// 	memset(error, 0, n_ticks*sizeof(float));
// 	// The fileter window size is set to n_pos - one electrical cycle 
// 	// important for cogging filtering !!!
// 	const int window = n_pos; // window size for moving average filter of raw error
	

// 	// find the first guess of the motor.zero_electric_angle 
// 	// and the sensor direction
// 	// updates motor.zero_electric_angle
// 	// updates motor.sensor_direction
// 	// temporarily unlink the sensor and current sense
// 	CurrentSense *current_sense = motor.current_sense;
// 	motor.current_sense = nullptr;
// 	motor.linkSensor(&this->_wrapped);
// 	if(!motor.initFOC()){
// 		motor.monitor_port->println("Failed to align the sensor.");
// 		return;
// 	}
// 	// link back the sensor and current sense
// 	motor.linkSensor(this);
// 	motor.linkCurrentSense(current_sense);

// 	// Set voltage angle to zero, wait for rotor position to settle
// 	// keep the motor in position while getting the initial positions
// 	motor.setPhaseVoltage(1, 0, elec_angle);
// 	_delay(1000);
// 	_wrapped.update();
// 	float theta_init = _wrapped.getAngle();
// 	float theta_absolute_init = _wrapped.getMechanicalAngle();

// 	/*
// 	Start Calibration
// 	Loop over  electrical angles from 0 to NPP*2PI, once forward, once backward
// 	store actual position and error as compared to electrical angle
// 	*/

// 	/*
// 	forwards rotation
// 	*/
// 	motor.monitor_port->print("Rotating: ");
// 	motor.monitor_port->println( motor.sensor_direction == Direction::CCW ? "CCW" : "CW" );
// 	float zero_angle_prev = 0.0;
// 	for (int i = 0; i < n_ticks * 4; i++)
// 	{
// 		for (int j = 0; j < n2_ticks; j++) // move to the next location
// 		{
// 			_wrapped.update();
// 			elec_angle += deltaElectricalAngle;
// 			motor.setPhaseVoltage(voltage_calibration, 0, elec_angle);
// 		}

// 		// delay to settle in position before taking a position sample
// 		_delay(settle_time_ms);
// 		_wrapped.update();
// 		// calculate the error
// 		theta_actual = (int)motor.sensor_direction * (_wrapped.getAngle() - theta_init);
//         error[i % n_ticks] = 0.5 * (theta_actual - elec_angle / _NPP);

// 		// calculate the current electrical zero angle
// 		float zero_angle = ((int)motor.sensor_direction * _wrapped.getMechanicalAngle() * _NPP ) - (elec_angle + _PI_2);
// 		zero_angle = _normalizeAngle(zero_angle);
// 		// remove the 2PI jumps
// 		if(zero_angle - zero_angle_prev > _PI){
// 			zero_angle = zero_angle - _2PI;
// 		}else if(zero_angle - zero_angle_prev < -_PI){
// 			zero_angle = zero_angle + _2PI;
// 		}
// 		zero_angle_prev = zero_angle;
// 		avg_elec_angle += zero_angle/n_ticks;

// 		// motor.monitor_port->print(">zero:");
// 		// motor.monitor_port->println(zero_angle);
// 		// motor.monitor_port->print(">zero_average:");
// 		// motor.monitor_port->println(avg_elec_angle/2.0);
// 	}
// 	_delay(2000);

// 	/*
// 	backwards rotation
// 	*/
// 	motor.monitor_port->print("Rotating: ");
// 	motor.monitor_port->println( motor.sensor_direction == Direction::CCW ? "CW" : "CCW" );
// 	for (int i = n_ticks * 4 - 1; i >= 0; i--)
// 	{
// 		for (int j = 0; j < n2_ticks; j++) // move to the next location
// 		{
// 			_wrapped.update();
// 			elec_angle -= deltaElectricalAngle;
// 			motor.setPhaseVoltage(voltage_calibration, 0, elec_angle);
// 		}

// 		// delay to settle in position before taking a position sample
// 		_delay(settle_time_ms);
// 		_wrapped.update();
// 		// calculate the error
// 		theta_actual = (int)motor.sensor_direction * (_wrapped.getAngle() - theta_init);
//         error[i % n_ticks] += 0.5 * (theta_actual - elec_angle / _NPP);
// 		// calculate the current electrical zero angle
// 		float zero_angle = ((int)motor.sensor_direction * _wrapped.getMechanicalAngle() * _NPP ) - (elec_angle + _PI_2);
// 		zero_angle = _normalizeAngle(zero_angle);
// 		// remove the 2PI jumps
// 		if(zero_angle - zero_angle_prev > _PI){
// 			zero_angle = zero_angle - _2PI;
// 		}else if(zero_angle - zero_angle_prev < -_PI){
// 			zero_angle = zero_angle + _2PI;
// 		}
// 		zero_angle_prev = zero_angle;
// 		avg_elec_angle += zero_angle/n_ticks;

// 		// motor.monitor_port->print(">zero:");
// 		// motor.monitor_port->println(zero_angle);
// 		// motor.monitor_port->print(">zero_average:");
// 		// motor.monitor_port->println(avg_elec_angle/2.0);
// 	}

// 	// get post calibration mechanical angle.
// 	_wrapped.update();
// 	float theta_absolute_post = _wrapped.getMechanicalAngle();

// 	// done with the measurement
// 	motor.setPhaseVoltage(0, 0, 0);

// 	// raw offset from initial position in absolute radians between 0-2PI
// 	float raw_offset = (theta_absolute_init + theta_absolute_post) / 2;

// 	// calculating the average zero electrical angle from the forward calibration.
// 	motor.zero_electric_angle = _normalizeAngle(avg_elec_angle / (2.0));
// 	motor.monitor_port->print("Average Zero Electrical Angle: ");
// 	motor.monitor_port->println(motor.zero_electric_angle);
// 	_delay(1000);

// 	// Perform filtering to linearize position sensor eccentricity
// 	// FIR n-sample average, where n = number of samples in one electrical cycle
// 	// This filter has zero gain at electrical frequency and all integer multiples
// 	// So cogging effects should be completely filtered out
// 	float error_mean = 0.0;
// 	this->filter_error(error, error_mean, n_ticks, window);

// 	_delay(1000);
// 	// calculate offset index
// 	int index_offset = floor((float)n_lut * raw_offset / _2PI);
// 	float dn = n_ticks / (float)n_lut;

// 	motor.monitor_port->println("Constructing LUT.");
// 	_delay(1000);
// 	// Build Look Up Table
// 	for (int i = 0; i < n_lut; i++)
// 	{
// 		int ind = index_offset + i*motor.sensor_direction;
// 		if (ind > (n_lut - 1)) ind -= n_lut;
// 		if (ind < 0) ind += n_lut;
// 		calibrationLut[ind] = (float)(error[(int)(i * dn)] - error_mean); 
// 		// negate the error if the sensor is in the opposite direction
// 		calibrationLut[ind] =  (int)motor.sensor_direction * calibrationLut[ind];
// 	}
// 	motor.monitor_port->println("");
// 	_delay(1000);

// 	// Display the LUT
// 	motor.monitor_port->print("float calibrationLut[");
// 	motor.monitor_port->print(n_lut);
// 	motor.monitor_port->println("] = {");
// 	_delay(100); 
// 	for (int i=0;i < n_lut; i++){
// 		motor.monitor_port->print(calibrationLut[i],6);
// 		if(i < n_lut - 1) motor.monitor_port->print(", ");
// 		_delay(1);
// 	}
// 	motor.monitor_port->println("};");
// 	_delay(1000);

// 	// Display the zero electrical angle
// 	motor.monitor_port->print("float zero_electric_angle = ");
// 	motor.monitor_port->print(motor.zero_electric_angle,6);
// 	motor.monitor_port->println(";");

// 	// Display the sensor direction
// 	motor.monitor_port->print("Direction sensor_direction = ");
// 	motor.monitor_port->println(motor.sensor_direction == Direction::CCW ? "Direction::CCW;" : "Direction::CW;");
// 	_delay(1000);

// 	motor.monitor_port->println("Sensor Calibration Done.");
// }

  // Главная калибровка Фурье
  void HarmonicCalibratedSensor::calibrate(FOCMotor &motor, int settle_time_ms, float voltage_calibration) {
    coef_ready = false;
    motor.monitor_port->println("Starting Harmonic (Fourier) Sensor Calibration.");

    // --- Параметры съёма данных ---
    // Для устойчивого фильтра от "кокинга" берём N кратным числу пар полюсов
    int NPP = motor.pole_pairs;
    if (N < NPP) N = NPP;
    if (N % NPP != 0) N = (N / NPP + 1) * NPP;   // кратность NPP
    const int n2_ticks = 5;                      // микрошаги между точками
    const float dEl = _2PI * NPP / (N * n2_ticks);

    // --- Временные буферы ---
    float *err = new float[N];
    memset(err, 0, N * sizeof(float));
    float avg_elec_zero = 0.0f;

    // Временно отвяжем наш калибратор от мотора, чтобы найти zero_electric_angle и направление
    CurrentSense *cs = motor.current_sense;
    motor.current_sense = nullptr;
    motor.linkSensor(&_wrapped);
    if (!motor.initFOC()) {
      motor.monitor_port->println("Failed to align the sensor.");
      motor.linkSensor(this);
      motor.linkCurrentSense(cs);
      delete[] err;
      return;
    }
    motor.linkSensor(this);       // обратно вешаем калиброванный датчик (пока коэффициенты нулевые)
    motor.linkCurrentSense(cs);

    // Удерживаем ротор, запоминаем исходные углы
    float elec = 0.0f;
    motor.setPhaseVoltage(voltage_calibration, 0, elec);
    _delay(1000);
    _wrapped.update();
    float theta_init = _wrapped.getAngle();
    float theta_abs_init = _wrapped.getMechanicalAngle();

    // -------- Проход вперёд --------
    motor.monitor_port->print("Rotating: ");
    motor.monitor_port->println(motor.sensor_direction == Direction::CCW ? "CCW" : "CW");
    float zero_prev = 0.0f;
    for (int i = 0; i < N; i++) {
      // подходим к точке i
      for (int j = 0; j < n2_ticks; j++) {
        _wrapped.update();
        elec += dEl;
        motor.setPhaseVoltage(voltage_calibration, 0, elec);
      }
      _delay(settle_time_ms);
      _wrapped.update();

      float theta_act = (int)motor.sensor_direction * (_wrapped.getAngle() - theta_init);
      err[i] = 0.5f * (theta_act - elec / NPP);

      float zero = ((int)motor.sensor_direction * _wrapped.getMechanicalAngle() * NPP) - (elec + _PI_2);
      zero = _normalizeAngle(zero);
      // убираем скачки 2π
      if (zero - zero_prev > _PI) zero -= _2PI;
      else if (zero - zero_prev < -_PI) zero += _2PI;
      zero_prev = zero;
      avg_elec_zero += zero / N;
    }

    _delay(500);

    // -------- Проход назад --------
    motor.monitor_port->print("Rotating: ");
    motor.monitor_port->println(motor.sensor_direction == Direction::CCW ? "CW" : "CCW");
    for (int i = N - 1; i >= 0; i--) {
      for (int j = 0; j < n2_ticks; j++) {
        _wrapped.update();
        elec -= dEl;
        motor.setPhaseVoltage(voltage_calibration, 0, elec);
      }
      _delay(settle_time_ms);
      _wrapped.update();

      float theta_act = (int)motor.sensor_direction * (_wrapped.getAngle() - theta_init);
      err[i] += 0.5f * (theta_act - elec / NPP);

      float zero = ((int)motor.sensor_direction * _wrapped.getMechanicalAngle() * NPP) - (elec + _PI_2);
      zero = _normalizeAngle(zero);
      if (zero - zero_prev > _PI) zero -= _2PI;
      else if (zero - zero_prev < -_PI) zero += _2PI;
      zero_prev = zero;
      avg_elec_zero += zero / N;
    }

    // завершаем съём
    _wrapped.update();
    float theta_abs_post = _wrapped.getMechanicalAngle();
    motor.setPhaseVoltage(0, 0, 0);

    // Средний сдвиг абсолютного мех. угла (для фазы сетки)
    float raw_offset = (theta_abs_init + theta_abs_post) * 0.5f;

    // Итоговый ноль электрического угла
    motor.zero_electric_angle = _normalizeAngle(avg_elec_zero * 0.5f);
    motor.monitor_port->print("Average Zero Electrical Angle: ");
    motor.monitor_port->println(motor.zero_electric_angle, 6);

    // --- Фильтрация ошибки (скользящее среднее на один эл. период) ---
    const int window = max(1, N / NPP); // один эл. период = N/NPP отсчётов
    movingAverageInPlace(err, N, window);

    // --- Оценка коэффициентов Фурье ---
    // θ_n = raw_offset + 2π * n / N
    // a0 = (1/N) * Σ e[n]
    // a_k = (2/N) * Σ e[n] * sin(k θ_n)
    // b_k = (2/N) * Σ e[n] * cos(k θ_n)
    float a0 = 0.0f;
    for (int n = 0; n < N; n++) a0 += err[n];
    a[0] = a0 / (float)N;

    for (int k = 1; k <= K; k++) {
      double akk = 0.0;
      double bkk = 0.0;
      for (int n = 0; n < N; n++) {
        float theta_n = raw_offset + _2PI * (float)n / (float)N;
        float sk = sinf(k * theta_n);
        float ck = cosf(k * theta_n);
        akk += (double)err[n] * (double)sk;
        bkk += (double)err[n] * (double)ck;
      }
      a[k] = (2.0f / (float)N) * (float)akk;
      b[k] = (2.0f / (float)N) * (float)bkk;
      // учёт направления сенсора — меняет знак ошибки
      a[k] = (int)motor.sensor_direction * a[k];
      b[k] = (int)motor.sensor_direction * b[k];
    }
    // для a0 знак направления тоже релевантен
    a[0] = (int)motor.sensor_direction * a[0];

    coef_ready = true;

    // --- Отладочный вывод коэффициентов ---
    motor.monitor_port->println("Fourier coefficients:");
    motor.monitor_port->print("a0 = "); motor.monitor_port->println(a[0], 6);
    for (int k = 1; k <= K; k++) {
      motor.monitor_port->print("a["); motor.monitor_port->print(k); motor.monitor_port->print("] = ");
      motor.monitor_port->println(a[k], 6);
      motor.monitor_port->print("b["); motor.monitor_port->print(k); motor.monitor_port->print("] = ");
      motor.monitor_port->println(b[k], 6);
    }

    // Вывод базовых параметров для копирования в проект
    motor.monitor_port->print("float zero_electric_angle = ");
    motor.monitor_port->print(motor.zero_electric_angle, 6);
    motor.monitor_port->println(";");

    motor.monitor_port->print("Direction sensor_direction = ");
    motor.monitor_port->println(motor.sensor_direction == Direction::CCW ? "Direction::CCW;" : "Direction::CW;");

    motor.monitor_port->println("Harmonic Sensor Calibration Done.");

    delete[] err;
  }

// Простой скользящий средний по окну 'w'
  void HarmonicCalibratedSensor::movingAverageInPlace(float *x, int n, int w) {
    if (w <= 1 || w > n) return;
    float sum = 0;
    for (int i = 0; i < w; i++) sum += x[i];
    float *y = new float[n];
    for (int i = 0; i < n; i++) {
      if (i == 0) y[0] = sum / w;
      else {
        int out = i - 1;
        int in  = (i - 1 + w) % n;
        sum += x[in];
        sum -= x[out];
        y[i] = sum / w;
      }
    }
    memcpy(x, y, n * sizeof(float));
    delete[] y;
  }