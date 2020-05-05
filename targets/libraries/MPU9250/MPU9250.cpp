#include <MPU9250.h>

// Constructor for MPU9250 Class
MPU9250::MPU9250(I2C *i2c, uint8_t addr)
	{
		_i2c=i2c;
		if(addr){
		MPU9250_ADDRESS = MPU9250_ADDRESS_1;
		}
		else{
		MPU9250_ADDRESS = MPU9250_ADDRESS_0;
		}		
		
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void MPU9250::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
	{
	   char data_write[2];
	   data_write[0] = subAddress;
	   data_write[1] = data;
	   _i2c->write(address, data_write, 2, 0);
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

char MPU9250::readByte(uint8_t address, uint8_t subAddress)
	{
		char data[1]; // `data` will store the register data
		char data_write[1];
		data_write[0] = subAddress;
		_i2c->write(address, data_write, 1, 1); // no stop
		_i2c->read(address, data, 1, 0);
		return data[0];
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void MPU9250::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
	{
		char data[14];
		char data_write[1];
		data_write[0] = subAddress;
		_i2c->write(address, data_write, 1, 1); // no stop
		_i2c->read(address, data, count, 0);

		for(int ii = 0; ii < count; ii++) {
			 dest[ii] = data[ii];
			}
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

float MPU9250::getMres(uint8_t Mscale)
	{
		  switch (Mscale)
		  {
			// Possible magnetometer scales (and their register bit settings) are:
			// 14 bit resolution (0) and 16 bit resolution (1)
			case MFS_14BITS:
				  return 10.0*4219.0/8190.0; // Proper scale to return milliGauss
			case MFS_16BITS:
				  return 10.0*4219.0/32760.0; // Proper scale to return milliGauss
		  }
		  return 0;
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

float MPU9250::getGres(uint8_t Gscale)
	{
		  switch (Gscale)
		  {
			// Possible gyro scales (and their register bit settings) are:
			// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
			case GFS_250DPS:
				  return 250.0/32768.0;
			case GFS_500DPS:
				  return 500.0/32768.0;
			case GFS_1000DPS:
				  return 1000.0/32768.0;
			case GFS_2000DPS:
				  return 2000.0/32768.0;
		  }
		  return 0;
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

float MPU9250::getAres(uint8_t Ascale)
	{
		  switch (Ascale)
		  {
			// Possible accelerometer scales (and their register bit settings) are:
			// 2 g (00), 4 g (01), 8 g (10), and 16 g  (11).
			case AFS_2G:
				  return 2.0/32768.0;
			case AFS_4G:
				  return 4.0/32768.0;
			case AFS_8G:
				  return 8.0/32768.0;
			case AFS_16G:
				  return 16.0/32768.0;
		  }
		  return 0;
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void MPU9250::readAccelData(float * destination)
	{
		  uint8_t rawData[6];  // x/y/z accel register data stored here
		  float accel_Res;
		  int16_t accel_raw[3];

		  accel_Res = getAres(Ascale); // Get accelerometer sensitivity
		  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
		  accel_raw[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		  accel_raw[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		  accel_raw[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;


		  // Now we'll calculate the accleration value into actual g's
		  destination[0] = (float)accel_raw[0]*accel_Res - accelBias[0];  // get actual g value, this depends on scale being set
		  destination[1] = (float)accel_raw[1]*accel_Res - accelBias[1];
		  destination[2] = (float)accel_raw[2]*accel_Res - accelBias[2];
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void MPU9250::readGyroData(float * destination)
	{
		  uint8_t rawData[6];  // x/y/z gyro register data stored here
		  float gyro_Res;
		  int16_t gyro_raw[3];

		  gyro_Res = getGres(Gscale); // Get gyro sensitivity
		  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
		  gyro_raw[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		  gyro_raw[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		  gyro_raw[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		  // Calculate the gyro value into actual degrees per second
		  destination[0] = (float)gyro_raw[0]*gyro_Res - gyroBias[0];  // get actual gyro value, this depends on scale being set
		  destination[1] = (float)gyro_raw[1]*gyro_Res - gyroBias[1];
		  destination[2] = (float)gyro_raw[2]*gyro_Res - gyroBias[2];
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void MPU9250::readMagData(float * destination)
	{
		  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
		  float mag_Res;
		  int16_t mag_raw[3];

		  mag_Res = getMres(Mscale); // Get magnetometer sensitivity
		  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22); // enable Bypass
		  if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
			  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
			  uint8_t c = rawData[6]; // End data read by reading ST2 register

			  if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
				  mag_raw[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
				  mag_raw[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) ;  // Data stored as little Endian
				  mag_raw[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) ;

				  // Calculate the magnetometer values in milliGauss
				  // Include factory calibration per data sheet and user environmental corrections
				  destination[0] = (float)mag_raw[0]*mag_Res*magCalibration[0] ;  // get actual magnetometer value, this depends on scale being set
				  destination[1] = (float)mag_raw[1]*mag_Res*magCalibration[1] ;
				  destination[2] = (float)mag_raw[2]*mag_Res*magCalibration[2] ;
			  }
		  }
		  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x20); // disable Bypass
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

float MPU9250::readTempData()
	{
		  uint8_t rawData[2];  // x/y/z gyro register data stored here
		  int16_t temp_adc;
		  float temp;

		  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
		  temp_adc = (int16_t)(((int16_t)rawData[0]) << 8 | rawData[1]) ;  // Turn the MSB and LSB into a 16-bit value
		  temp = ((((float) temp_adc) / 333.87f + 21.0f)); // Temperature in degrees Centigrade
		  return (float)temp;
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void MPU9250::resetMPU9250()
	{
		// reset device
		writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
		thread_sleep_for(100);
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void MPU9250::initAK8963()
	{	
		writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22); // enable Bypass
		Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
		uint8_t Mmode = 0x06;        // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR

		// First extract the factory calibration for each magnetometer axis
		uint8_t rawData[3];  // x/y/z gyro calibration data stored here
		writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
		thread_sleep_for(10);
		writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
		thread_sleep_for(10);
		readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
		magCalibration[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
		magCalibration[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;
		magCalibration[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;
		writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
		thread_sleep_for(10);
		// Configure the magnetometer for continuous read and highest resolution
		// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
		// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
		writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
		thread_sleep_for(10);
		writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x20); // disable Bypass
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void MPU9250::initMPU9250()
	{

		  Ascale = AFS_4G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
		  Gscale = GFS_500DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS

		  // Initialize MPU9250 device
		 // wake up device
		  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
		  thread_sleep_for(100); //  100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

		 // get stable time source
		  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

		 // Configure Gyro and Accelerometer
		 // Disable FSYNC and set gyro and temp bandwidth to 41 and 42 Hz, respectively;
		 // DLPF_CFG = bits 2:0 = 011; this sets the sample rate at 1 kHz for both
		 // Maximum  is 5.9 ms which is just over a 170 Hz maximum rate
		  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

		 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
		  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Use a 1000 Hz rate; the same rate set in CONFIG above

		 // Set gyroscope full scale range
		 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
		  uint8_t c =  readByte(MPU9250_ADDRESS, GYRO_CONFIG);
		  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
		  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
		  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro

		 // Set accelerometer configuration
		  c =  readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
		  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
		  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
		  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer

		 // Set accelerometer sample rate configuration
		 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
		 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
		  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
		  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
		  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

		 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
		 // but all these rates are further reduced by a factor of 1 to 1000 Hz because of the SMPLRT_DIV setting

		  // Configure Interrupts and Bypass Enable
		  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, (not enable I2C_BYPASS_EN so additional chips
		  // can join the I2C bus and all can be controlled by the Arduino as master)
		   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x20);
		   writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void MPU9250::calibrateMPU9250()
	{
		  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
		  uint16_t ii, packet_count, fifo_count;
		  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

		// reset device, reset all registers, clear gyro and accelerometer bias registers
		  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
		  thread_sleep_for(100);

		// get stable time source
		// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
		  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
		  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
		  thread_sleep_for(200);

		// Configure device for bias calculation
		  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
		  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
		  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
		  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
		  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
		  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
		  thread_sleep_for(15);

		// Configure MPU9250 gyro and accelerometer for bias calculation
		  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
		  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
		  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
		  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

		  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
		  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

		// Configure FIFO to capture accelerometer and gyro data for bias calculation
		  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
		  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
		  thread_sleep_for(40); // accumulate 40 samples in 80 milliseconds = 480 bytes

		// At end of sample accumulation, turn off FIFO sensor read
		  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
		  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
		  fifo_count = ((uint16_t)data[0] << 8) | data[1];
		  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

		  for (ii = 0; ii < packet_count; ii++)
		  	  {
					int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
					readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
					accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
					accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
					accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
					gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
					gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
					gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

					accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
					accel_bias[1] += (int32_t) accel_temp[1];
					accel_bias[2] += (int32_t) accel_temp[2];
					gyro_bias[0]  += (int32_t) gyro_temp[0];
					gyro_bias[1]  += (int32_t) gyro_temp[1];
					gyro_bias[2]  += (int32_t) gyro_temp[2];
		  	  }

		  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
		  accel_bias[1] /= (int32_t) packet_count;
		  accel_bias[2] /= (int32_t) packet_count;
		  gyro_bias[0]  /= (int32_t) packet_count;
		  gyro_bias[1]  /= (int32_t) packet_count;
		  gyro_bias[2]  /= (int32_t) packet_count;

		  if(accel_bias[2] > 0L)
		  	  {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
		  else
		  	  {accel_bias[2] += (int32_t) accelsensitivity;}

		// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
		  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
		  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
		  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
		  data[3] = (-gyro_bias[1]/4)       & 0xFF;
		  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
		  data[5] = (-gyro_bias[2]/4)       & 0xFF;

		/// Push gyro biases to hardware registers
		/*  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
		  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
		  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
		  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
		  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
		  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
		*/
		  gyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
		  gyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
		  gyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

		// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
		// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
		// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
		// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
		// the accelerometer biases calculated above must be divided by 8.

		  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
		  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
		  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
		  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
		  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
		  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
		  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

		  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
		  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

		  for(ii = 0; ii < 3; ii++)
		  	  {
			  	  if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
		  	  }

		  // Construct total accelerometer bias, including calculated average accelerometer bias from above
		  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
		  accel_bias_reg[1] -= (accel_bias[1]/8);
		  accel_bias_reg[2] -= (accel_bias[2]/8);

		  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
		  data[1] = (accel_bias_reg[0])      & 0xFF;
		  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
		  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
		  data[3] = (accel_bias_reg[1])      & 0xFF;
		  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
		  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
		  data[5] = (accel_bias_reg[2])      & 0xFF;
		  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

		// Apparently this is not working for the acceleration biases in the MPU-9250
		// Are we handling the temperature correction bit properly?
		// Push accelerometer biases to hardware registers
		/*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
		  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
		  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
		  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
		  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
		  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
		*/
		// Output scaled accelerometer biases for manual subtraction in the main program
		  accelBias[0] = (float)accel_bias[0]/(float)accelsensitivity;
		  accelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
		  accelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
	}
/////////////////////////////////////////////////////////////////////////////////////////////////////////

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250::MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
	{
	   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
	   uint8_t selfTest[6];
	   int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
	   float factoryTrim[6];
	   uint8_t FS = 0;

	   writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz
	   writeByte(MPU9250_ADDRESS, CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	   writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 1<<FS); // Set full scale range for the gyro to 250 dps
	   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

	   for( int ii = 0; ii < 200; ii++)
	   	   { // get average current values of gyro and acclerometer

		   	   readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
		   	   aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		   	   aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		   	   aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		   	   readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
		   	   gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		   	   gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		   	   gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	   	   }

	   for (int ii =0; ii < 3; ii++)
	   	   { // Get average of 200 values and store as average current readings
		   	   aAvg[ii] /= 200;
		   	   gAvg[ii] /= 200;
	   	   }

	   // Configure the accelerometer for self-test
	   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	   writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	   thread_sleep_for(25); //  a while to let the device stabilize

	   for( int ii = 0; ii < 200; ii++)
	   	   { // get average self-test values of gyro and acclerometer

		   	   readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
		   	   aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		   	   aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		   	   aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		   	   readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
		   	   gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
		   	   gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		   	   gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	   	   }

	   for (int ii =0; ii < 3; ii++)
	   	   { // Get average of 200 values and store as average self-test readings
		   	   aSTAvg[ii] /= 200;
		   	   gSTAvg[ii] /= 200;
	   	   }

	   // Configure the gyro and accelerometer for normal operation
	   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
	   writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
	   thread_sleep_for(25); //  a while to let the device stabilize

	   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	   selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
	   selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
	   selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	   selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO); // X-axis gyro self-test results
	   selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO); // Y-axis gyro self-test results
	   selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO); // Z-axis gyro self-test results

	  // Retrieve factory self-test value from self-test code reads
	   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
	   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
	   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
	   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
	   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
	   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

	 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	 // To get percent, must multiply by 100
	   for (int i = 0; i < 3; i++)
	   	   {
		   	   destination[i] = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i]; // Report percent differences
		   	   destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
	   	   }
	}


