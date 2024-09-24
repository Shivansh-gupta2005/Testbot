#include "MPU9250.h"

MPU9250 mpu;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    delay(2000);

    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;
    
    mpu.verbose(true);
    
    if (!mpu.setup(0x68, setting)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();

    mpu.setMagBias(-153.835, -127.095, -381.37);
    mpu.setMagScale(1.00788, 1.0275, 0.9666);
    mpu.setMagneticDeclination(0.62);

    mpu.verbose(false);
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            print_ypr();
            prev_ms = millis();
        }
    }
}

void print_mag() {
    Serial.print(mpu.getMagX());
    Serial.print(",");
    Serial.print(mpu.getMagY());
    Serial.print(",");
    Serial.println(mpu.getMagZ());
}

void print_ypr() {
    Serial.print(mpu.getRoll());
    Serial.print(",");
    Serial.print(mpu.getPitch());
    Serial.print(",");
    Serial.println(mpu.getYaw());
}
