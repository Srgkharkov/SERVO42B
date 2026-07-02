
#define A1333_CPR 32768
#define A1333_BITORDER MSBFIRST
#define A1333_REG_ANG 0x3200

#include "common/base_classes/Sensor.h"
#include <SPI.h>
// #include "./A1334.h"

static SPISettings A1333SPISettings(8000000, A1333_BITORDER, SPI_MODE3); // @suppress("Invalid arguments")


typedef union {
	struct {
		uint16_t angle:12;
		uint16_t p:1;
		uint16_t nf:1;
		uint16_t ef:1;
		uint16_t ridc:1;
	};
	uint16_t reg;
} A1334Angle;







// class A1334 {
// public:
// 	A1334(SPISettings settings = A1334SPISettings, int nCS = -1);
// 	virtual ~A1334();

// 	virtual void init(SPIClass* _spi = &SPI);
// 	A1334Angle readRawAngle(); // 10 or 12 bit angle value    
// protected:
// 	uint16_t spi_transfer16(uint16_t outdata);
// 	SPIClass* spi;
// 	SPISettings settings;
// 	int nCS = -1;
// };






class MagneticSensorA1333 : public Sensor {
public:
	MagneticSensorA1333(int nCS = -1, SPISettings settings = A1333SPISettings);
	virtual ~MagneticSensorA1333();

    virtual float getSensorAngle() override;

	virtual void init(SPIClass* _spi = &SPI);
	uint16_t readRawAngle(); // 10 or 12 bit angle value  
protected:
	uint16_t spi_transfer16(uint16_t outdata);
	SPIClass* spi;
	SPISettings settings;
	int nCS = -1;
private:
};



