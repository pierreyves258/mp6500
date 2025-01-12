#ifndef mp6500_h
#define mp6500_h

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#define MPU6500_ADDRESS     0x68

#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_DATA            0x3B
#define REG_GYRO_OFFSET     0x13
#define REG_WHO_AM_I        0x75 // Should return 0x71 for MPU9250 and 0x70 for MPU6500
#define REF_CONFIG			0x1A

#define GYRO_FULL_SCALE_250_DPS     0x00  
#define GYRO_FULL_SCALE_500_DPS     0x08  
#define GYRO_FULL_SCALE_1000_DPS    0x10  
#define GYRO_FULL_SCALE_2000_DPS    0x18  

#define ACC_FULL_SCALE_2_G      0x00  	
#define ACC_FULL_SCALE_4_G      0x08
#define ACC_FULL_SCALE_8_G      0x10
#define ACC_FULL_SCALE_16_G     0x18

#define LOWPASS_10HZ    0x05
#define LOWPASS_05HZ    0x06

#define SENSORS_GRAVITY_EARTH (9.80665F) // Earth's gravity in m/s^2 

class MP6500 {
    private:
        uint8_t _acc_conf, _gyro_conf;
        float _acc_scale, _gyro_scale;
        void __I2CreadByte(uint8_t, uint8_t, uint8_t, uint8_t*);
        void __I2CreadWords(uint8_t, uint8_t, uint8_t, int16_t*);
        void __I2CwriteByte(uint8_t, uint8_t, uint8_t);
        void __I2CwriteWords(uint8_t, uint8_t, int16_t*, uint32_t);
        void __I2CwriteWord(uint8_t, uint8_t, int16_t);
        uint8_t __I2CreadByte(uint8_t, uint8_t);
    public:
        float acc_x, acc_y, acc_z, rot_x, rot_y, rot_z, temperature;
        MP6500(uint8_t = ACC_FULL_SCALE_16_G, uint8_t = GYRO_FULL_SCALE_2000_DPS);
        void Init();
        void CalibrateGyro(uint32_t = 100);
        void GetAndParseDataRegisters(uint8_t = 4);
        void PrintIMUdata();
};

#endif