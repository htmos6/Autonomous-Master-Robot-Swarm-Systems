#ifndef IMU_H
#define IMU_H

//-------------------------------MPU6050 Accelerometer and Gyroscope C++ library-----------------------------
//Copyright (c) 2019, Alex Mous
//Licensed under the CC BY-NC SA 4.0


//-----------------------MODIFY THESE PARAMETERS-----------------------

#define GYRO_RANGE 0 //Select which gyroscope range to use (see the table below) - Default is 0
//	Gyroscope Range
//	0	+/- 250 degrees/second
//	1	+/- 500 degrees/second
//	2	+/- 1000 degrees/second
//	3	+/- 2000 degrees/second
//See the MPU6000 Register Map for more information


#define ACCEL_RANGE 0 //Select which accelerometer range to use (see the table below) - Default is 0
//	Accelerometer Range
//	0	+/- 2g
//	1	+/- 4g
//	2	+/- 8g
//	3	+/- 16g
//See the MPU6000 Register Map for more information


//Offsets - supply your own here (calculate offsets with getOffsets function)
//     Accelerometer
// Start from acceleratometer
#define A_OFF_X -12366.752//78942.214
#define A_OFF_Y 19349.885
#define A_OFF_Z -27224.5653
//    Gyroscope
#define G_OFF_X -166.555//-157.717
#define G_OFF_Y -63.214//-64.2975
#define G_OFF_Z 176.8640869 //1.35009990*131.0


/* 

INITIAL CALIB VALUES


#define A_OFF_X -12942.214
#define A_OFF_Y 19135.738
#define A_OFF_Z -43610.496
//    Gyroscope
#define G_OFF_X -157.717
#define G_OFF_Y -64.2975
#define G_OFF_Z 176.8640869 //1.35009990*131.0


*/

//-----------------------END MODIFY THESE PARAMETERS-----------------------

#include <iostream>
#include <time.h>
#include <cmath>
#include <string>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "pico/multicore.h"

#include <vector>
#include <algorithm>

#define _POSIX_C_SOURCE 200809L //Used for calculating time

#define TAU 0.05 //Complementary filter percentage
#define RAD_T_DEG 57.29577951308 //Radians to degrees (180/PI)

//Select the appropriate settings
#if GYRO_RANGE == 1
	#define GYRO_SENS 65.5
	#define GYRO_CONFIG 0b00001000
#elif GYRO_RANGE == 2
	#define GYRO_SENS 32.8
	#define GYRO_CONFIG 0b00010000
#elif GYRO_RANGE == 3
	#define GYRO_SENS 16.4
	#define GYRO_CONFIG 0b00011000
#else //Otherwise, default to 0
	#define GYRO_SENS 131.0
	#define GYRO_CONFIG 0b00000000
#endif
#undef GYRO_RANGE


#if ACCEL_RANGE == 1
	#define ACCEL_SENS 8192.0
	#define ACCEL_CONFIG 0b00001000
#elif ACCEL_RANGE == 2
	#define ACCEL_SENS 4096.0
	#define ACCEL_CONFIG 0b00010000
#elif ACCEL_RANGE == 3
	#define ACCEL_SENS 2048.0
	#define ACCEL_CONFIG 0b00011000
#else //Otherwise, default to 0
	#define ACCEL_SENS 16384.0
	#define ACCEL_CONFIG 0b00000000
#endif
#undef ACCEL_RANGE




class MPU6050 {
	private:
		double _accel_angle[3];
		double _gyro_angle[3];
		double _angle[3]; //Store all angles (accel roll, accel pitch, accel yaw, gyro roll, gyro pitch, gyro yaw, comb roll, comb pitch comb yaw)

		double ax, ay, az, gr, gp, gy; //Temporary storage variables used in _update()

		int MPU6050_addr;
		int f_dev; //Device file

		double dt; //Loop time (recalculated with each loop)

		struct timespec start,end; //Create a time structure

		bool _first_run = 1; //Variable for whether to set gyro angle to acceleration angle in compFilter

        const uint8_t REG_GYRO = 0x43;
        const uint8_t REG_ACCEL = 0x3b;

		bool stopImu = false;
		bool isStopped = true;



		float velocity[3];
		float position[3];
		float lastUpdateTime;
		const float accelerometerNoise = 1.0; // Placeholder for the accelerometer measurement noise
		const float gyroscopeNoise = 1.0;     // Placeholder for the gyroscope measurement noise
	public:
		MPU6050(int8_t addr);
		MPU6050(int8_t addr, bool run_update_thread);
		void getAccelRaw(double *x, double *y, double *z);
		void getGyroRaw(double *roll, double *pitch, double *yaw);
		void getAccel(double *x, double *y, double *z);
		void getGyro(double *roll, double *pitch, double *yaw);
		void getOffsets(double *ax_off, double *ay_off, double *az_off, double *gr_off, double *gp_off, double *gy_off);
		int getAngle(int axis, double *result);
		bool calc_yaw;

        void _update();
		void writeReg(uint8_t reg, uint8_t data);
		int16_t readReg(uint8_t reg);
		void getAccelData(int16_t* ax, int16_t* ay, int16_t* az);
		double getHeading();
		void reset();
		void startImu();
		std::tuple<double, double, double> getPosition();

		void getMotion6(int16_t *Ax, int16_t *Ay, int16_t *Az, int16_t *Gx, int16_t *Gy, int16_t *Gz);
		void setXAccelOffset(int16_t offset);
		void setYAccelOffset(int16_t offset);
		void setZAccelOffset(int16_t offset);
		void setXGyroOffset(int16_t offset);
		void setYGyroOffset(int16_t offset);
		void setZGyroOffset(int16_t offset);

		int16_t ax_off = 0, ay_off = 0, az_off = 0, gx_off = 0, gy_off = 0, gz_off = 0;

		/* double kPGyro = 0.6;
		double kIGyro = 50;
		double kPAccel = 0.15;
		double kIAccel = 8;
		double MPUOffsets[6];

		double ITerm[6] = {0, 0, 0, 0, 0, 0};
		double ITermReadings[10][6];
		int Readings = 0;
		int StopCount;
		int ReadingDetailTime = 1000;
		int Tries = 0;
		uint32_t ITermLastSum;
		double ITermSameCtr;
		double ITermSameCtr2;
		double ITermSameCtrMax = 100.0;
		int TestCtr = 0;

		double DefaultskPGyro = 0.6;
		double DefaultskIGyro = 50;
		double DefaultskPAccel = 0.15;
		double DefaultskIAccel = 8;
		int ITermSameCtrLimit1 = 5;
		int ITermSameCtrLimit2 = 10;

		unsigned long DetailsTimer, StartTime;

		double ax_f = 0.0;
		double ay_f = 0.0;
		double az_f = 0.0;
		double gx_f = 0.0;
		double gy_f = 0.0;
		double gz_f = 0.0;*/
};

int clock_gettime(clockid_t unused, struct timespec *tp);
static MPU6050* IMU_obj;
void update_wrapper();


std::vector<double> findOutliers(const std::vector<double>& arr);

#endif // IMU_H