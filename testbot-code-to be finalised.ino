#include<IntervalTimer.h>
#include<Wire.h>
#include<LiquidCrystal_I2C.h>
#include<ros.h>
#include<MPU9250.h>
#include<Kalman.h>
#include<sensor_msgs/Imu.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>





Kalman kalmanYaw, kalmanPitch, kalmanRoll;
const float Q_angle = 0.001; 
const float Q_bias = 0.003;   
const float R_measure = 0.03; 
float angleYaw, anglePitch, angleRoll;

TinyGPSPlus gps; 
#define gpsSerial Serial1 

ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu_data",&imu_msg);

sensor_msgs::NavSatFix navsat_msg;
ros::Publisher navsat_pub("navsatfix",&navsat_msg);




IntervalTimer lcd_timer;
IntervalTimer imu_timer;
IntervalTimer gps_timer;

void setup(){
  Wire.begin():
  lcd.begin(&Wire1);
  lcd.init();
  lcd.backlight();
  pinMode(38,INPUT);
  analogReadResolution(10);
  while (!Serial); 

if (!mpu.setup(0x68)) {
        Serial.println("MPU connection failed. Check your connection.");
        while (1);
    }

    kalmanYaw.setAngle(0);
    kalmanPitch.setAngle(0);
    kalmanRoll.setAngle(0);
    kalmanYaw.setQangle(Q_angle);
    kalmanYaw.setQbias(Q_bias);
    kalmanYaw.setRmeasure(R_measure);
    kalmanPitch.setQangle(Q_angle);
    kalmanPitch.setQbias(Q_bias);
    kalmanPitch.setRmeasure(R_measure);
    kalmanRoll.setQangle(Q_angle);
    kalmanRoll.setQbias(Q_bias);
    kalmanRoll.setRmeasure(R_measure);

    gpsSerial.begin(9600);

  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(navsat_pub);
  lcd_timer.begin(display_LCD,150000);
  imu_timer.begin(readIMU,150000);
  gps_timer.begin(readGPS,150000);

}

void loop(){
  nh.spinOnce();
}


void  display_LCD(){
  float voltage = (analogValue * 22.2) / 1024.0;
  lcd.setCursor(0, 0);
  lcd.print("Voltage: ");
  lcd.print(voltage, 2);  
  lcd.print("V    ");
}

void readIMU(){
  if (mpu.update()) {
        float gyroX = mpu.getGyroX();
        float gyroY = mpu.getGyroY();
        float gyroZ = mpu.getGyroZ();

        // Update Kalman filters
        float dt = 0.025; // Time interval in seconds (25 milliseconds)
        angleYaw = kalmanYaw.getAngle(mpu.getYaw(), gyroX, dt);
        anglePitch = kalmanPitch.getAngle(mpu.getPitch(), gyroY, dt);
        angleRoll = kalmanRoll.getAngle(mpu.getRoll(), gyroZ, dt);

        // Convert Euler angles to quaternion
        float q[4];
        float cy = cos(angleYaw * 0.5);
        float sy = sin(angleYaw * 0.5);
        float cp = cos(anglePitch * 0.5);
        float sp = sin(anglePitch * 0.5);
        float cr = cos(angleRoll * 0.5);
        float sr = sin(angleRoll * 0.5);

        q[0] = cr * cp * cy + sr * sp * sy; // q[0] = w
        q[1] = sr * cp * cy - cr * sp * sy; // q[1] = x
        q[2] = cr * sp * cy + sr * cp * sy; // q[2] = y
        q[3] = cr * cp * sy - sr * sp * cy; // q[3] = z

        // Fill the IMU message
        imu_msg.header.stamp = nh.now();
        imu_msg.orientation.x = q[1];
        imu_msg.orientation.y = q[2];
        imu_msg.orientation.z = q[3];
        imu_msg.orientation.w = q[0];
        imu_msg.angular_velocity.x = gyroX;
        imu_msg.angular_velocity.y = gyroY;
        imu_msg.angular_velocity.z = gyroZ;

        // Publish IMU message
        imu_pub.publish(&imu_msg);
    }
}


void readGPS(){
     while (gpsSerial.available() > 0) {
        char c = gpsSerial.read();
        gps.encode(c);  // Feed the characters to the GPS object
    }

    // Publish GPS data if it's ready
    if (gps.location.isUpdated()) {
        // Fill NavSatFix message with GPS data
        navsat_msg.latitude = gps.location.lat();
        navsat_msg.longitude = gps.location.lng();
        navsat_msg.altitude = gps.altitude.meters();

        // Set position covariance and its type (optional)
        navsat_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

        // Fill status field
        navsat_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        navsat_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

        // Publish the NavSatFix message
        navsat_pub.publish(&navsat_msg);

        // Print GPS data to Serial Monitor for debugging
        Serial.print("Latitude: ");
        Serial.print(navsat_msg.latitude, 6);  
        Serial.print(" Longitude: ");
        Serial.println(navsat_msg.longitude, 6);
        Serial.print("Altitude: ");
        Serial.print(navsat_msg.altitude);
        Serial.println(" m");
    }


}

