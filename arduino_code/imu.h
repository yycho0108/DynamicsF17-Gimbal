#include <quaternionFilters.h>
#include <MPU9250.h>

class IMUManager {
  public:
    MPU9250 imu;
    //int32_t int_pin;
  public:
    IMUManager()
    {

    }
    void setup(bool mag_cal = false)
    {
      Wire.begin();
      //pinMode(int_pin, INPUT);
      //digitalWrite(int_pin, LOW);

      // Read the WHO_AM_I register, this is a good test of communication
      byte c = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
      Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
      Serial.print(" I should be "); Serial.println(0x71, HEX);

      if (c == 0x71) {
        imu.MPU9250SelfTest(imu.selfTest);
        imu.calibrateMPU9250(imu.gyroBias, imu.accelBias);

        imu.initMPU9250();

        byte d = imu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
        Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
        Serial.print(" I should be "); Serial.println(0x48, HEX);

        imu.initAK8963(imu.factoryMagCalibration);
        Serial.print("X-Axis sensitivity adjustment value ");
        Serial.println(imu.factoryMagCalibration[0], 2);
        Serial.print("Y-Axis sensitivity adjustment value ");
        Serial.println(imu.factoryMagCalibration[1], 2);
        Serial.print("Z-Axis sensitivity adjustment value ");
        Serial.println(imu.factoryMagCalibration[2], 2);

        imu.getAres();
        imu.getGres();
        imu.getMres();

        if (mag_cal) {
          imu.magCalMPU9250(imu.magBias, imu.magScale);

          Serial.println("AK8963 mag biases (mG)");
          Serial.println(imu.magBias[0]);
          Serial.println(imu.magBias[1]);
          Serial.println(imu.magBias[2]);

          Serial.println("AK8963 mag scale (mG)");
          Serial.println(imu.magScale[0]);
          Serial.println(imu.magScale[1]);
          Serial.println(imu.magScale[2]);
        } else {
          imu.magBias[0] = 347.03;
          imu.magBias[1] = 167.22;
          imu.magBias[2] = 6.93;
          imu.magScale[0] = 1.03;
          imu.magScale[1] = 1.11;
          imu.magScale[2] = 0.89;
        }
      } else {
        Serial.print("Fail : 0x");
        Serial.println(c, HEX);
        while (true);
      }
    }

    void read(unsigned long now) {
      if (imu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
        imu.readAccelData(imu.accelCount);

        imu.ax = (float)imu.accelCount[0] * imu.aRes; - imu.accelBias[0];
        imu.ay = (float)imu.accelCount[1] * imu.aRes; - imu.accelBias[1];
        imu.az = (float)imu.accelCount[2] * imu.aRes; - imu.accelBias[2];

        imu.readGyroData(imu.gyroCount);  // Read the x/y/z adc values

        // Calculate the gyro value into actual degrees per second
        // This depends on scale being set
        imu.gx = (float)imu.gyroCount[0] * imu.gRes;
        imu.gy = (float)imu.gyroCount[1] * imu.gRes;
        imu.gz = (float)imu.gyroCount[2] * imu.gRes;

        imu.readMagData(imu.magCount);  // Read the x/y/z adc values
        imu.mx = (float)imu.magCount[0] * imu.mRes * imu.factoryMagCalibration[0] -
                 imu.magBias[0];
        imu.my = (float)imu.magCount[1] * imu.mRes * imu.factoryMagCalibration[1] -
                 imu.magBias[1];
        imu.mz = (float)imu.magCount[2] * imu.mRes * imu.factoryMagCalibration[2] -
                 imu.magBias[2];

        imu.updateTime();

        MahonyQuaternionUpdate(imu.ax, imu.ay, imu.az, imu.gx * DEG_TO_RAD,
                               imu.gy * DEG_TO_RAD, imu.gz * DEG_TO_RAD, imu.my,
                               imu.mx, imu.mz, imu.deltat);

        imu.delt_t = now - imu.count;
        if (imu.delt_t > 500) {
          /*
             Serial.print("ax = "); Serial.print(9.81 * imu.ax);
             Serial.print(" ay = "); Serial.print(9.81 * imu.ay);
             Serial.print(" az = "); Serial.print(9.81 * imu.az);
             Serial.println(" m/s^2");

             Serial.print("gx = "); Serial.print( imu.gx, 2);
             Serial.print(" gy = "); Serial.print( imu.gy, 2);
             Serial.print(" gz = "); Serial.print( imu.gz, 2);
             Serial.println(" deg/s");

             Serial.print("mx = "); Serial.print( (int)imu.mx );
             Serial.print(" my = "); Serial.print( (int)imu.my );
             Serial.print(" mz = "); Serial.print( (int)imu.mz );
             Serial.println(" mG");

             Serial.print("q0 = "); Serial.print(*getQ());
             Serial.print(" qx = "); Serial.print(*(getQ() + 1));
             Serial.print(" qy = "); Serial.print(*(getQ() + 2));
             Serial.print(" qz = "); Serial.println(*(getQ() + 3));
          */
          imu.yaw   = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *
                                    *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1) * *(getQ() + 1)
                            - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3) * *(getQ() + 3));
          imu.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
                                    *(getQ() + 2)));
          imu.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2) *
                                    *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1) * *(getQ() + 1)
                            - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3) * *(getQ() + 3));
          imu.pitch *= RAD_TO_DEG;
          imu.yaw   *= RAD_TO_DEG;
          imu.yaw   -= 8.5; // TODO : calibrate
          imu.roll  *= RAD_TO_DEG;
          /*
             Serial.print("Yaw, Pitch, Roll: ");
          */
          //Serial.println(imu.yaw);
          /*
             Serial.print(", ");
             Serial.print(imu.pitch, 2);
             Serial.print(", ");
             Serial.println(imu.roll, 2);
             Serial.print("rate = ");
             Serial.print((float)imu.sumCount / imu.sum, 2);
             Serial.println(" Hz");
          */
          imu.count = now;
          imu.sumCount = 0;
          imu.sum = 0;
        }
      }

    }
};
