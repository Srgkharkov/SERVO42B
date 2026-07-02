
#include "./MagneticSensorA1333.h"
#include "common/foc_utils.h"
#include "common/time_utils.h"

MagneticSensorA1333::MagneticSensorA1333(int nCS, SPISettings settings) : settings(settings), nCS(nCS) {

}


MagneticSensorA1333::~MagneticSensorA1333(){ 

}


void MagneticSensorA1333::init(SPIClass* _spi) {
    // this->A1334::init(_spi);
	spi = _spi;
	if (nCS>=0) {
		pinMode(nCS, OUTPUT);
		digitalWrite(nCS, HIGH);
	}
	//SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
	spi->begin();
	readRawAngle(); // read an angle

    this->Sensor::init();
}

uint16_t MagneticSensorA1333::readRawAngle() {
	uint16_t command = A1333_REG_ANG;
	uint16_t cmdResult = spi_transfer16(command);	// TODO fast mode
	cmdResult = spi_transfer16(command);	
	// A1334Angle result = { .reg = cmdResult };
	// TODO check parity
	// errorflag = result.ef;
	return cmdResult & 0x7FFF;;
};

uint16_t MagneticSensorA1333::spi_transfer16(uint16_t outdata) {
	spi->beginTransaction(settings);
	if (nCS>=0)
		digitalWrite(nCS, 0);
	uint16_t result = spi->transfer16(outdata);
	if (nCS>=0)
		digitalWrite(nCS, 1);
	spi->endTransaction();
	return result;
};

float MagneticSensorA1333::getSensorAngle() {
    uint16_t angle_data = readRawAngle();
    float result = ( angle_data / (float)A1333_CPR) * _2PI;
    // return the shaft angle
    return result;
}
