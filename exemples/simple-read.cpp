#define SERIAL_DEBUG
#include "mp6500.h"

MP6500 imu;

IRAM_ATTR void cal() {
	imu.CalibrateGyro();
} 

// Initializations
void setup()
{
	Wire.begin();
	Serial.begin(115200);
	delay(10);

	imu.Init();
	imu.CalibrateGyro();

	pinMode(13, INPUT);
	attachInterrupt(digitalPinToInterrupt(13), cal, FALLING);
}


void loop()
{
	imu.GetAndParseDataRegisters();
	imu.PrintIMUdata();
}