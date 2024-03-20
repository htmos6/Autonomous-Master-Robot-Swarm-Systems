#include "IMU.hpp"

MPU6050::MPU6050(int8_t addr, bool run_update_thread) {
	int status;

	MPU6050_addr = addr;
	dt = 0.009; //Loop time (recalculated with each loop)
	_first_run = 1; //Variable for whether to set gyro angle to acceleration angle in compFilter
	calc_yaw = true;

    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

	writeReg(0x6B, 0b00000001); // Wake up MPU6050
	//writeReg(0x1a, 0b00000011);
	//writeReg(0x19, 0b00000100);
	writeReg(0x1b, GYRO_CONFIG);
	writeReg(0x1c, ACCEL_CONFIG);
	writeReg(0x06, 0);
	writeReg(0x07, 0);
	writeReg(0x08, 0);
	writeReg(0x09, 0);
	writeReg(0x0A, 0);
	writeReg(0x0B, 0);
	writeReg(0x13, 0);
	writeReg(0x14, 0);
	writeReg(0x15, 0);
	writeReg(0x16, 0);
	writeReg(0x17, 0);
	writeReg(0x18, 0);

	std::cout << "Constructed" << std::endl;

    /*uint8_t buf[2];
    buf[0] = 0x6b;
    buf[1] = 0b00000000;
    i2c_write_blocking(i2c_default, MPU6050_addr, buf, 2, false);
    buf[0] = 0x1a;
    buf[1] = 0b00000011;
    i2c_write_blocking(i2c_default, MPU6050_addr, buf, 2, false);
    buf[0] = 0x19;
    buf[1] = 0b00000100;
    i2c_write_blocking(i2c_default, MPU6050_addr, buf, 2, false);
    buf[0] = 0x1b;
    buf[1] = GYRO_CONFIG;
    i2c_write_blocking(i2c_default, MPU6050_addr, buf, 2, false);
    buf[0] = 0x1c;
    buf[1] = ACCEL_CONFIG;
    i2c_write_blocking(i2c_default, MPU6050_addr, buf, 2, false);
    buf[0] = 0x06;
    buf[1] = 0b00000000;
    i2c_write_blocking(i2c_default, MPU6050_addr, buf, 2, false);
    buf[0] = 0x07;
    buf[1] = 0b00000000;
    i2c_write_blocking(i2c_default, MPU6050_addr, buf, 2, false);
    buf[0] = 0x08;
    buf[1] = 0b00000000;
    i2c_write_blocking(i2c_default, MPU6050_addr, buf, 2, false);
    buf[0] = 0x09;
    buf[1] = 0b00000000;
    i2c_write_blocking(i2c_default, MPU6050_addr, buf, 2, false);
    buf[0] = 0x0A;
    buf[1] = 0b00000000;
    i2c_write_blocking(i2c_default, MPU6050_addr, buf, 2, false);
    buf[0] = 0x0B;
    buf[1] = 0b00000000;
    i2c_write_blocking(i2c_default, MPU6050_addr, buf, 2, false);
    buf[0] = 0x00;
    buf[1] = 0b00000000;
    i2c_write_blocking(i2c_default, MPU6050_addr, buf, 2, false);
    buf[0] = 0x01;
    buf[1] = 0b00000000;
    i2c_write_blocking(i2c_default, MPU6050_addr, buf, 2, false);
    buf[0] = 0x02;
    buf[1] = 0b00000000;
    i2c_write_blocking(i2c_default, MPU6050_addr, buf, 2, false);*/

    IMU_obj = this;
    //multicore_launch_core1(update_wrapper);
}

MPU6050::MPU6050(int8_t addr) : MPU6050(addr, true){}

void MPU6050::getGyroRaw(double *roll, double *pitch, double *yaw) {
    /*uint8_t buf[6];
    i2c_write_blocking(i2c_default, MPU6050_addr, &REG_GYRO, 1, true);
    i2c_read_blocking(i2c_default, MPU6050_addr, buf, 6, false);
    int16_t *parsed_axes = reinterpret_cast<int16_t *>(buf);*/
	int16_t X = readReg(0x43) + gx_off;
	int16_t Y = readReg(0x45) + gy_off;
	int16_t Z = readReg(0x47) + gz_off;
	*roll = (double)X; //Roll on X axis
	*pitch = (double)Y; //Pitch on Y axis
	*yaw = (double)Z; //Yaw on Z axis
}

void MPU6050::getGyro(double *roll, double *pitch, double *yaw) {
	getGyroRaw(roll, pitch, yaw); //Store raw values into variables
	*roll = round((*roll - G_OFF_X) * 1000.0 / GYRO_SENS) / 1000.0; //Remove the offset and divide by the gyroscope sensetivity (use 1000 and round() to round the value to three decimal places)
	*pitch = round((*pitch - G_OFF_Y) * 1000.0 / GYRO_SENS) / 1000.0;
	*yaw = round((*yaw - G_OFF_Z) * 1000.0 / GYRO_SENS) / 1000.0;
}

void MPU6050::getAccelRaw(double *x, double *y, double *z) {
    /*uint8_t buf[6];
    i2c_write_blocking(i2c_default, MPU6050_addr, &REG_ACCEL, 1, true);
    i2c_read_blocking(i2c_default, MPU6050_addr, buf, 6, false);
    int16_t *parsed_axes = reinterpret_cast<int16_t *>(buf);*/
	int16_t X = readReg(0x3B) + ax_off;
	int16_t Y = readReg(0x3D) + ay_off;
	int16_t Z = readReg(0x3F) + az_off;
	*x = (double)X;
	*y = (double)Y;
	*z = (double)Z;
}

void MPU6050::getAccel(double *x, double *y, double *z) {
	getAccelRaw(x, y, z); //Store raw values into variables
	*x = round((*x - A_OFF_X) * 1000.0 / ACCEL_SENS) / 1000.0; //Remove the offset and divide by the accelerometer sensetivity (use 1000 and round() to round the value to three decimal places)
	*y = round((*y - A_OFF_Y) * 1000.0 / ACCEL_SENS) / 1000.0;
	*z = round((*z - A_OFF_Z) * 1000.0 / ACCEL_SENS) / 1000.0;
}

void MPU6050::getMotion6(int16_t *Ax, int16_t *Ay, int16_t *Az, int16_t *Gx, int16_t *Gy, int16_t *Gz){
	*Ax = readReg(0x3B) - ax_off;
	*Ay = readReg(0x3D) - ay_off;
	*Az = readReg(0x3F) - az_off;

	*Gx = readReg(0x43) - gx_off;
	*Gy = readReg(0x45) - gy_off;
	*Gz = readReg(0x47) - gz_off;
}

void MPU6050::setXAccelOffset(int16_t offset){
	/*writeReg(0x06, static_cast<uint8_t>((offset>>8)&0xFF));
	writeReg(0x07, static_cast<uint8_t>(offset&0xFF));*/
	ax_off = offset;
}

void MPU6050::setYAccelOffset(int16_t offset){
	/*writeReg(0x08, static_cast<uint8_t>((offset>>8)&0xFF));
	writeReg(0x09, static_cast<uint8_t>(offset&0xFF));*/
	ay_off = offset;
}

void MPU6050::setZAccelOffset(int16_t offset){
	/*writeReg(0x0A, static_cast<uint8_t>((offset>>8)&0xFF));
	writeReg(0x0B, static_cast<uint8_t>(offset&0xFF));*/
	az_off = offset;
}

void MPU6050::setXGyroOffset(int16_t offset){
	/*writeReg(0x13, static_cast<uint8_t>((offset>>8)&0xFF));
	writeReg(0x14, static_cast<uint8_t>(offset&0xFF));*/
	gx_off = offset;
}

void MPU6050::setYGyroOffset(int16_t offset){
	/*writeReg(0x15, static_cast<uint8_t>((offset>>8)&0xFF));
	writeReg(0x16, static_cast<uint8_t>(offset&0xFF));*/
	gy_off = offset;
}

void MPU6050::setZGyroOffset(int16_t offset){
	/*writeReg(0x17, static_cast<uint8_t>((offset>>8)&0xFF));
	writeReg(0x18, static_cast<uint8_t>(offset&0xFF));*/
	gz_off = offset;
}

std::vector<double> findOutliers(const std::vector<double>& arr) {
    std::vector<double> outliers;

    // Sorting the array
    std::vector<double> sortedArr = arr;
    std::sort(std::begin(sortedArr), std::end(sortedArr), std::less<double>{});

    // Calculating the first and third quartiles
    int n = sortedArr.size();
    double q1 = sortedArr[n / 4];
    double q3 = sortedArr[(3 * n) / 4];

    // Calculating the interquartile range (IQR)
    double iqr = q3 - q1;

    // Finding the lower and upper bounds for outliers
    double lowerBound = q1 - (1.5 * iqr);
    double upperBound = q3 + (1.5 * iqr);

    // Finding outliers
    for (double num : arr) {
        if (!(num < lowerBound || num > upperBound)) {
            outliers.push_back(num);
        }
    }

    return outliers;
}

void MPU6050::getOffsets(double *ax_off, double *ay_off, double *az_off, double *gr_off, double *gp_off, double *gy_off) {
	double gyro_off[3]; //Temporary storage
	double accel_off[3];

	std::vector<double> gx, gy, gz, ax, ay, az;

	*gr_off = 0, *gp_off = 0, *gy_off = 0; //Initialize the offsets to zero
	*ax_off = 0, *ay_off = 0, *az_off = 0; //Initialize the offsets to zero

	for (int i = 0; i < 2000; i++) { //Use loop to average offsets
		getGyroRaw(&gyro_off[0], &gyro_off[1], &gyro_off[2]); //Raw gyroscope values
		*gr_off = *gr_off + gyro_off[0], *gp_off = *gp_off + gyro_off[1], *gy_off = *gy_off + gyro_off[2]; //Add to sum
		gx.push_back(gyro_off[0]);
		gy.push_back(gyro_off[1]);
		gz.push_back(gyro_off[2]);

		getAccelRaw(&accel_off[0], &accel_off[1], &accel_off[2]); //Raw accelerometer values
		*ax_off = *ax_off + accel_off[0], *ay_off = *ay_off + accel_off[1], *az_off = *az_off + accel_off[2]; //Add to sum
		ax.push_back(accel_off[0]);
		ay.push_back(accel_off[1]);
		az.push_back(accel_off[2]);
		sleep_ms(10);
	}

	*gr_off = *gr_off / 2000.0, *gp_off = *gp_off / 2000.0, *gy_off = *gy_off / 2000.0; //Divide by number of loops (to average)
	*ax_off = *ax_off / 2000.0, *ay_off = *ay_off / 2000.0, *az_off = *az_off / 2000.0;

	*az_off = *az_off - ACCEL_SENS; //Remove 1g from the value calculated to compensate for gravity)

	/**gr_off = 0.0;
	std::vector<double> out = findOutliers(gx);
	std::cout << out.size() << "  ";
	for (double num : out) {
		//std::cout << num << " ";
		*gr_off += num;
	}
	std::cout << std::endl;
	*gr_off /= static_cast<double>(out.size());
	out.clear();

	*gp_off = 0.0;
	out = findOutliers(gy);
	std::cout << out.size() << "  ";
	for (double num : out) {
		//std::cout << num << " ";
		*gp_off += num;
	}
	std::cout << std::endl;
	*gp_off /= static_cast<double>(out.size());
	out.clear();

	*gy_off = 0.0;
	out = findOutliers(gz);
	std::cout << out.size() << "  ";
	for (double num : out) {
		//std::cout << num << " ";
		*gy_off += num;
	}
	std::cout << std::endl;
	*gy_off /= static_cast<double>(out.size());
	out.clear();

	*ax_off = 0.0;
	out = findOutliers(ax);
	std::cout << out.size() << "  ";
	for (double num : out) {
		//std::cout << num << " ";
		*ax_off += num;
	}
	std::cout << std::endl;
	*ax_off /= static_cast<double>(out.size());
	out.clear();

	*ay_off = 0.0;
	out = findOutliers(ay);
	std::cout << out.size() << "  ";
	for (double num : out) {
		//std::cout << num << " ";
		*ay_off += num;
	}
	std::cout << std::endl;
	*ay_off /= static_cast<double>(out.size());
	out.clear();

	*az_off = 0.0;
	out = findOutliers(az);
	std::cout << out.size() << "  ";
	for (double num : out) {
		//std::cout << num << " ";
		*az_off += num;
	}
	std::cout << std::endl;
	*az_off /= static_cast<double>(out.size());
	out.clear();*/

/*	// PI Variables
  double Error[6];
  double PTerm[6];
  double DTerm[6] = {0, 0, 0, 0, 0, 0};
  double g[3];
  double a[3];
  double ax, ay, az, gx, gy, gz;
  long error;
  double ITermSum;
  DetailsTimer = time_us_64()/1000.0;
  while (1) {
    Tries++;
    getAccel(&ax, &ay, &az);
	getGyro(&gx, &gy, &gz);
    az = az - 16384.0; // remove Gravity
    if (Tries == 1) { // Set it close and get a new reading
      getAccel(&ax, &ay, &az);
	  getGyro(&gx, &gy, &gz);
      az = az - 16384.0; // remove Gravity
	  ax_f = -ax*ACCEL_SENS;
	  ay_f = -ay*ACCEL_SENS;
	  az_f = -az*ACCEL_SENS;
	  gx_f = -gx*GYRO_SENS; 
	  gy_f = -gy*GYRO_SENS; 
	  gz_f = -gz*GYRO_SENS; 
      //setOffset(-(ax >> 3), -(ay >> 3), -(az >> 3), -(gx >> 2), -(gy >> 2), -(gz >> 2));
      getAccel(&ax, &ay, &az);
	  getGyro(&gx, &gy, &gz);
      az = az - 16384.0; // remove Gravity
      // Prime PI Loop
      ITerm[0] = -(ax);
      ITerm[1] = -(ay);
      ITerm[2] = -(az);
      ITerm[3] = -(gx);
      ITerm[4] = -(gy);
      ITerm[5] = -(gz);
      DetailsTimer = time_us_64()/1000.0;
    } else {
      a[0] = ax;
      a[1] = ay;
      a[2] = az;
      g[0] = gx;
      g[1] = gy;
      g[2] = gz;
      ITermSum = 0.0;
      for (int i = 0; i < 6; i++) ITermSum += abs(ITerm[i]);
      ITermSameCtr = (ITermSum == ITermLastSum) ? (ITermSameCtr + 1) : 0;
      ITermSameCtr2 = (ITermSum == ITermLastSum) ? (ITermSameCtr2 + 1) : 0;
      ITermLastSum = ITermSum;
      ITermSameCtrMax = ((ITermSameCtrMax > ITermSameCtr) ? ITermSameCtrMax : ITermSameCtr);
      if (((time_us_64()/1000.0 - DetailsTimer) >= (ReadingDetailTime)) || (ITermSameCtr2 >= ITermSameCtrLimit2)) {
        DetailsTimer = time_us_64()/1000.0;
        if ((ITermSameCtr >= ITermSameCtrLimit1) || (ITermSameCtr2 >= ITermSameCtrLimit2)) {
          ITermSameCtr = 0;
          if ((ITermSameCtr2 >= ITermSameCtrLimit2)) {
            for (int i = 0; i < 6; i++) {
              ITermReadings[TestCtr % 10][i] = ITerm[i]; // % is remainder after division. This prevents us from counting over 9 (0-9) 10/10 remainder = 0
            }
            std::cout << ax_f << " " << ay_f << " " << az_f << " " << gx_f << " " << gy_f << " " << gz_f << std::endl;
            return;
          }
          // Re-tune PID and run again
          ReadingDetailTime *= .95;
          kPGyro = kPGyro * 0.5;
          kIGyro = kIGyro * 0.5;
          kPAccel = kPAccel * 0.5;
          kIAccel = kIAccel * 0.5;
          DetailsTimer = time_us_64()/1000.0;
        }
        ReadingDetailTime *= .60;
        kPGyro = kPGyro * 0.70;
        kIGyro = kIGyro * 0.70;
        kPAccel = kPAccel * 0.70;
        kIAccel = kIAccel * 0.70;
      }
      // PI of PID Calculations 
      for (int i = 0; i < 3; i++) { // PI Calculations
        // Accelerometer
        Error[i] = 0.0 - a[i];
        PTerm[i] = kPAccel * Error[i];
        ITerm[i] += Error[i] * 0.002 * kIAccel; // Integral term 1000 Calculations a second = 0.001
        MPUOffsets[i] = (PTerm[i] + ITerm[i]); // Compute PID Output
        // Gyro
        Error[i + 3] = 0 - g[i];
        PTerm[i + 3] = kPGyro * Error[i + 3];
        ITerm[i + 3] += Error[i + 3] * 0.002 * kIGyro; // Integral term 1000 Calculations a second = 0.001
        MPUOffsets[i + 3] = (PTerm[i + 3] + ITerm[i + 3]); // Compute PID Output
      }
      //setOffset(round(MPUOffsets[0] / 8), round(MPUOffsets[1] / 8), round(MPUOffsets[2] / 8), round(MPUOffsets[3] / 4), round(MPUOffsets[4] / 4), round(MPUOffsets[5]) / 4);
	  ax_f = MPUOffsets[0]*ACCEL_SENS;
	  ay_f = MPUOffsets[1]*ACCEL_SENS;
	  az_f = MPUOffsets[2]*ACCEL_SENS;
	  gx_f = MPUOffsets[3]*GYRO_SENS; 
	  gy_f = MPUOffsets[4]*GYRO_SENS; 
	  gz_f = MPUOffsets[5]*GYRO_SENS; 
      // get another reading for the next loop
    }
    sleep_ms(2000);
  };*/
}

int MPU6050::getAngle(int axis, double *result) {
	if (axis >= 0 && axis <= 2) { //Check that the axis is in the valid range
		*result = _angle[axis]; //Get the result
		return 0;
	}
	else {
		std::cout << "ERR (MPU6050.cpp:getAngle()): 'axis' must be between 0 and 2 (for roll, pitch or yaw)\n"; //Print error message
		*result = 0; //Set result to zero
		return 1;
	}
}

void MPU6050::_update() { //Main update function - runs continuously
	clock_gettime(CLOCK_REALTIME, &start); //Read current time into start variable

	while (1) { //Loop forever
		if (stopImu == true){
			break;
		}
		getGyro(&gr, &gp, &gy); //Get the data from the sensors
		getAccel(&ax, &ay, &az);

		/*double ax_s, ay_s, az_s, gr_s, gp_s, gy_s;
		ax = 0; ay = 0; az = 0; gr = 0; gp = 0; gy = 0;
		for(int i = 0; i < 4096; i++){
			getGyro(&gr_s, &gp_s, &gy_s); //Get the data from the sensors
			getAccel(&ax_s, &ay_s, &az_s);
			ax+=ax_s; ay+=ay_s; az+=az_s;
			gr+=gr_s; gp+=gp_s; gy+=gy_s;
		}
		ax/=4096.0; ay/=4096.0; az/=4096.0;
		gr/=4096.0; gp/=4096.0; gy/=4096.0;*/

		//X (roll) axis
		_accel_angle[0] = atan2(az, ay) * RAD_T_DEG - 90.0; //Calculate the angle with z and y convert to degrees and subtract 90 degrees to rotate
		_gyro_angle[0] = _angle[0] + gr*dt; //Use roll axis (X axis)

		//Y (pitch) axis
		_accel_angle[1] = atan2(az, ax) * RAD_T_DEG - 90.0; //Calculate the angle with z and x convert to degrees and subtract 90 degrees to rotate
		_gyro_angle[1] = _angle[1] + gp*dt; //Use pitch axis (Y axis)

		//Z (yaw) axis
		if (calc_yaw) {
			//_gyro_angle[2] = gy;
			_gyro_angle[2] = _angle[2] + ((abs(gy) >= 0.0075) ? gy*dt : 0.0); //Use yaw axis (Z axis)
		}


		if (_first_run) { //Set the gyroscope angle reference point if this is the first function run
			for (int i = 0; i <= 1; i++) {
				_gyro_angle[i] = _accel_angle[i]; //Start off with angle from accelerometer (absolute angle since gyroscope is relative)
			}
			_gyro_angle[2] = 0; //Set the yaw axis to zero (because the angle cannot be calculated with the accelerometer when vertical)
			_first_run = 0;
		}

		double asum = abs(ax) + abs(ay) + abs(az); //Calculate the sum of the accelerations
		double gsum = abs(gr) + abs(gp) + abs(gy); //Calculate the sum of the gyro readings

		for (int i = 0; i <= 1; i++) { //Loop through roll and pitch axes
			if (abs(_gyro_angle[i] - _accel_angle[i]) > 5) { //Correct for very large drift (or incorrect measurment of gyroscope by longer loop time)
				_gyro_angle[i] = _accel_angle[i];
			}

			//Create result from either complementary filter or directly from gyroscope or accelerometer depending on conditions
			if (asum > 0.1 && asum < 3 && gsum > 0.3) { //Check that th movement is not very high (therefore providing inacurate angles)
				_angle[i] = (1 - TAU)*(_gyro_angle[i]) + (TAU)*(_accel_angle[i]); //Calculate the angle using a complementary filter
			}
			else if (gsum > 0.3) { //Use the gyroscope angle if the acceleration is high
				_angle[i] = _gyro_angle[i];
			}
			else if (gsum <= 0.3) { //Use accelerometer angle if not much movement
				_angle[i] = _accel_angle[i];
			}
		}

		//The yaw axis will not work with the accelerometer angle, so only use gyroscope angle
		if (calc_yaw) { //Only calculate the angle when we want it to prevent large drift
			_angle[2] = _gyro_angle[2];
		}
		else {
			_angle[2] = 0;
			_gyro_angle[2] = 0;
		}

		double acceleration[] = {ax, ay, az};
		double angularVelocity[] = {gr, gp, gy};

		// Integrate acceleration to get velocity and apply Kalman filter
        for (int i = 0; i < 3; i++) {
            // Prediction step
            float predictedVelocity = velocity[i] + acceleration[i] * dt;
            float predictedPosition = position[i] + velocity[i] * dt;
            
            // Update step
            float kalmanGain = (predictedVelocity + angularVelocity[i]) / (predictedVelocity + angularVelocity[i] + accelerometerNoise + gyroscopeNoise);
            velocity[i] = predictedVelocity + kalmanGain * (angularVelocity[i] - predictedVelocity);
            position[i] = predictedPosition + kalmanGain * (position[i] - predictedPosition);
        }




		clock_gettime(CLOCK_REALTIME, &end); //Save time to end clock
		dt = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9; //Calculate new dt
		clock_gettime(CLOCK_REALTIME, &start); //Save time to start clock
	}
	isStopped = true;
}

int clock_gettime(clockid_t unused, struct timespec *tp) {
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

void update_wrapper(){
    IMU_obj->_update();
}

void MPU6050::writeReg(uint8_t reg, uint8_t data) {
	uint8_t buf[2] = { reg, data };
	i2c_write_blocking(i2c_default, MPU6050_addr, buf, 2, false);
}

int16_t MPU6050::readReg(uint8_t reg) {
	uint8_t buf[2];
	i2c_write_blocking(i2c_default, MPU6050_addr, &reg, 1, true);
	i2c_read_blocking(i2c_default, MPU6050_addr, buf, 2, false);
	return (buf[0] << 8) | buf[1];
}


void MPU6050::getAccelData(int16_t* ax, int16_t* ay, int16_t* az) {
	*ax = readReg(0x3B);
	*ay = readReg(0x3D);
	*az = readReg(0x3F);
}

double MPU6050::getHeading() {
	int16_t ax, ay, az;
	getAccelData(&ax, &ay, &az);
	return atan2(ay, ax) * 180 / M_PI; // Calculate heading in degrees
}

void MPU6050::reset() {
	stopImu = true;
	_first_run = true;
	while(!isStopped){;}

	_accel_angle[0] = 0.0;
	_accel_angle[1] = 0.0;
	_accel_angle[2] = 0.0;

	_gyro_angle[0] = 0.0;
	_gyro_angle[1] = 0.0;
	_gyro_angle[2] = 0.0;

	_angle[0] = 0.0;
	_angle[1] = 0.0;
	_angle[2] = 0.0;
}

void MPU6050::startImu() {
	stopImu = false;
	isStopped = false;

	multicore_reset_core1();
	multicore_launch_core1(update_wrapper);
}

std::tuple<double, double, double> MPU6050::getPosition(){
	return {position[0], position[1], position[2]};
}