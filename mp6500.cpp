#include "mp6500.h"

// Constructor
MP6500::MP6500(uint8_t acc_conf, uint8_t gyro_conf) {
    _acc_conf = acc_conf;
    _gyro_conf = gyro_conf;

    switch (acc_conf) {
        case GYRO_FULL_SCALE_250_DPS:
            _gyro_scale = 250.0F;
            break;
        case GYRO_FULL_SCALE_500_DPS:
            _gyro_scale = 500.0F;
            break;
        case GYRO_FULL_SCALE_1000_DPS:
            _gyro_scale = 1000.0F;
            break;
        case GYRO_FULL_SCALE_2000_DPS:
            _gyro_scale = 2000.0F;
            break;
    };

    switch (gyro_conf) {
        case ACC_FULL_SCALE_2_G:
            _acc_scale = 2.0F * SENSORS_GRAVITY_EARTH;
            break;
        case ACC_FULL_SCALE_4_G:
            _acc_scale = 4.0F * SENSORS_GRAVITY_EARTH;
            break;
        case ACC_FULL_SCALE_8_G:
            _acc_scale = 8.0F * SENSORS_GRAVITY_EARTH;
            break;
        case ACC_FULL_SCALE_16_G:
            _acc_scale = 16.0F * SENSORS_GRAVITY_EARTH;
            break;
    };

	Serial.printf("CONSTRUCTOR acc_scale: %f, gyro_scale: %f\n", _acc_scale, _gyro_scale);

}

// Public
void MP6500::Init() {
    #ifdef SERIAL_DEBUG
        Serial.println(" ");

        // Read the WHO_AM_I register, this is a good test of communication
        Serial.println("MPU9250 or MPU6500 motion sensor...");
    #endif
    
    byte c = __I2CreadByte(MPU6500_ADDRESS, REG_WHO_AM_I);  // Read WHO_AM_I register for MPU-9250/MPU-6500

    #ifdef SERIAL_DEBUG
        Serial.print("Identification from WHO_AM_I register: >>"); Serial.print(c, HEX); 
        Serial.println("<<. ");
        Serial.println("It should be 0x71 (MPU9250) or 0x70 (MPU6500).");
    #endif

    while (c != 0x70) {
        delay(50);
        #ifdef SERIAL_DEBUG
            Serial.print(".");
        #endif
    }

    // Configure gyroscope range
    __I2CwriteByte(MPU6500_ADDRESS, REG_GYRO_CONFIG, _gyro_conf);
    // Configure accelerometers range
    __I2CwriteByte(MPU6500_ADDRESS, REG_ACCEL_CONFIG, _acc_conf);
    // Configure Low-Pass filtering 
    __I2CwriteByte(MPU6500_ADDRESS, REF_CONFIG, LOWPASS_05HZ);
}


void MP6500::CalibrateGyro(uint32_t numberOfReads) {
	Serial.println("CALIBRATION");
	delay(2000);

	const int16_t calibration_const = -2;
	int16_t zeros[] = {0, 0, 0};
	__I2CwriteWords(MPU6500_ADDRESS, REG_GYRO_OFFSET, zeros, 3);
	delay(10);

	//CALIBRATION
	int16_t buffer[3];
	int16_t gx, gy, gz;
	int64_t gxsum=0, gysum=0, gzsum=0;

	for (uint32_t i = 0; i < numberOfReads; i++) {
		__I2CreadWords(MPU6500_ADDRESS, 0x43, 3, buffer);
		gx = buffer[0];
		gy = buffer[1];
		gz = buffer[2];

		gxsum += gx;
		gysum += gy;
		gzsum += gz;
		delay(2);
	}
 
	gx = gxsum / numberOfReads;
	gy = gysum / numberOfReads;
	gz = gzsum / numberOfReads;

	Serial.printf("Gyro offsets: x => %d, y => %d, z => %d\n", gx, gy, gz);
	
	int16_t buff[] = {int16_t(calibration_const*gx), int16_t(calibration_const*gy), int16_t(calibration_const*gz)}; // Better calibration multiplying by 2 for 2000DPS

	__I2CwriteWords(MPU6500_ADDRESS, REG_GYRO_OFFSET, buff, 3);
}

void MP6500::GetAndParseDataRegisters(uint8_t numberOfReads) {
	// Read accelerometer and gyroscope
	// 59: ACCEL_X_H ACCEL_X_L
	// 61: ACCEL_Y_H ACCEL_Y_L
	// 63: ACCEL_Z_H ACCEL_Z_L
	// 65: TEMP_H TEMP_L
	// 67: GYRO_X_H GYRO_X_L
	// 69: GYRO_Y_H GYRO_Y_L
	// 71: GYRO_Z_H GYRO_Z_L
	
	const uint16_t buffer_length = 7;

	int16_t buffer[buffer_length];

	int32_t axsum = 0;	int32_t aysum = 0; 	int32_t azsum = 0;
	int32_t gxsum = 0; 	int32_t gysum = 0; 	int32_t gzsum = 0; int32_t temsum =0;

	int16_t ax, ay, az, gx, gy, gz, tem;
	for (uint8_t i = 0; i < numberOfReads; i++)
	{
		__I2CreadWords(MPU6500_ADDRESS, REG_DATA, buffer_length, buffer);
		ax = buffer[0];
		ay = buffer[1];
		az = buffer[2];
		// Gyroscope
		gx = buffer[4];
		gy = buffer[5];
		gz = buffer[6];
		// Temperature
		tem = buffer[3];

		axsum += ax; aysum += ay; azsum += az;
		gxsum += gx; gysum += gy; gzsum += gz;
		temsum += tem;
		delay(2);
	}
	ax = axsum / numberOfReads;	ay = aysum / numberOfReads;	az = azsum / numberOfReads;
	gx = gxsum / numberOfReads;	gy = gysum / numberOfReads;	gz = gzsum / numberOfReads;
	tem = temsum / numberOfReads;


	acc_x = (ax / 32767.0F) * _acc_scale;
	acc_y = (ay / 32767.0F) * _acc_scale;
	acc_z = (az / 32767.0F) * _acc_scale;

	rot_x = (gx / 32767.0F) * _gyro_scale;
	rot_y = (gy / 32767.0F) * _gyro_scale;
	rot_z = (gz / 32767.0F) * _gyro_scale;

	temperature = 21.0F + tem / 333.87F;
} 


void MP6500::PrintIMUdata() {
	Serial.printf(">ax:%f\n", acc_x);
	Serial.printf(">ay:%f\n", acc_y);
	Serial.printf(">az:%f\n", acc_z);
	Serial.printf(">gx:%f\n", rot_x);
	Serial.printf(">gy:%f\n", rot_y);
	Serial.printf(">gz:%f\n", rot_z);
}

// Private
void MP6500::__I2CreadByte(uint8_t addr, uint8_t reg, uint8_t qty, uint8_t* data) {
    uint8_t index = 0;

	Wire.beginTransmission(addr);	// Set register address
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(addr, qty);	// Read qty bytes
	while (Wire.available()) data[index++] = Wire.read();
}

void MP6500::__I2CreadWords(uint8_t addr, uint8_t reg, uint8_t qty, int16_t* data) {
	uint8_t *buffer = (uint8_t*)malloc(qty*2*sizeof(uint8_t));

	__I2CreadByte(addr, reg, qty*2, buffer);
	for (int i=0; i < qty; i++) {
		data[i] = buffer[2*i] << 8 | buffer[2*i+1];
	}

	free(buffer);
}

void MP6500::__I2CwriteByte(uint8_t addr, uint8_t reg, uint8_t data) {
	Wire.beginTransmission(addr);	// Set register address
	Wire.write(reg);
	Wire.write(data);
	Wire.endTransmission();
}

void MP6500::__I2CwriteWords(uint8_t addr, uint8_t startReg, int16_t* data, uint32_t length) {
	Wire.beginTransmission(addr);	// Set register address
	Wire.write(startReg);
	for (uint32_t i = 0; i < length; i++) {
		Wire.write(int8_t(data[i] >> 8));
		Wire.write(int8_t(data[i]));
	}
	Wire.endTransmission();
}

void MP6500::__I2CwriteWord(uint8_t addr, uint8_t startReg, int16_t data) {
	__I2CwriteWords(addr, startReg, &data, 1);
}

uint8_t MP6500::__I2CreadByte(uint8_t addr, uint8_t reg) {
	uint8_t data; 
	Wire.beginTransmission(addr);         // Initialize the Tx buffer
	Wire.write(reg);	                 // Put slave register address in Tx buffer
	Wire.endTransmission();
	Wire.requestFrom(addr, (size_t)1);    // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

