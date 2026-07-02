#ifndef __CALIBRATEDSENSOR_H__
#define __CALIBRATEDSENSOR_H__

#include "common/base_classes/Sensor.h"
#include "BLDCMotor.h"
#include "common/base_classes/FOCMotor.h"
#include "common/foc_utils.h"


class HarmonicCalibratedSensor: public Sensor{

public:
    /**
     * @brief Constructor of class with pointer to base class sensor and driver
     * @param wrapped the wrapped sensor which needs calibration
     * @param n_samples the number of reference samples per mech. turnover (multiple of NPP)
     * @param n_harmonics the number of harmonics in the Fourier approximation (K)
     */
    HarmonicCalibratedSensor(Sensor& wrapped, int n_samples = 200, int n_harmonics = 8);

    ~HarmonicCalibratedSensor();

    /*
    Override the update function
    */
    virtual void update() override;

    virtual void init() override;

    /**
    * Calibrate method computes the LUT for the correction
    */
    virtual void calibrate(FOCMotor &motor, int settle_time_ms = 20, float voltage_calibration = 1.0f);

    // voltage to run the calibration: user input
    float voltage_calibration = 1;    
protected:

    /**
    * getSenorAngle() method of CalibratedSensor class.
    * This should call getAngle() on the wrapped instance, and then apply the correction to
    * the value returned. 
    */
    virtual float getSensorAngle() override;
    /**
    * delegate instance of Sensor class
    */
    Sensor& _wrapped;

//     void alignSensor(FOCMotor &motor);
//     void filter_error(float* error, float &error_mean, int n_ticks, int window);
    
//      // lut size - settable by the user
//     int  n_lut { 200 } ;
//     // pointer for lut memory 
//     // depending on the size of the lut
//     // will be allocated in the calibrate function if not given.
//     bool allocated;
//     float* calibrationLut;
private:
//   Sensor &_wrapped;

    // Количество равномерных выборок по одному мех. обороту
    int N;
    // Максимальный порядок гармоники
    int K;

    // Коэффициенты Фурье
    float *a; // a[0] = DC, a[1..K] при sin
    float *b; // b[1..K] при cos
    bool coef_ready = false;

    void movingAverageInPlace(float *x, int n, int w);
};

#endif
