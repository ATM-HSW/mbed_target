/* Copyright 2015 M. Marquardt, HS Wismar */

#include "lsm303dlhc.h"



LSM303DLHC::LSM303DLHC(I2C *i2c)
{
  _i2c = i2c;
}

void LSM303DLHC::write_i2c(uint8_t reg, uint8_t *data, uint8_t len)
{	
	_i2c->write(dev_addr, (char*)&reg, 1, 1);
	_i2c->write(dev_addr, (char*)data, len, 0);
}


void LSM303DLHC::read_i2c(uint8_t reg, uint8_t *data, uint8_t len)
{		
	_i2c->write(dev_addr, (char*)&reg, 1, 1);
	_i2c->read(dev_addr, (char*)data, len, 0);
}


bool LSM303DLHC::init_acc(uint8_t rate, uint8_t scale, uint8_t hp_filter, bool drdy_int)
{
    uint8_t buf[5], whoami=0;
	
    // setup for accelerometer
		buf[0] = LSM303DLHC_CTRL_REG1_A | 0x80;
    buf[1] = (rate & 0xF0) | LSM303DLHC_AXES_ENABLE;
    buf[2] = (hp_filter & 0x68);
    buf[3] = (drdy_int & 0x1);
    buf[4] = 0xC0 | (scale & 0x30) | LSM303DLHC_HR_ENABLE;

    switch(scale)
    {
    case LSM303DLHC_FULLSCALE_2G:
        sensitivity_acc = (float)LSM303DLHC_ACC_SENSITIVITY_2G/16.0f;
        break;
    case LSM303DLHC_FULLSCALE_4G:
        sensitivity_acc = (float)LSM303DLHC_ACC_SENSITIVITY_4G/16.0f;
        break;
    case LSM303DLHC_FULLSCALE_8G:
        sensitivity_acc = (float)LSM303DLHC_ACC_SENSITIVITY_8G/16.0f;
        break;
    default:
        sensitivity_acc = (float)LSM303DLHC_ACC_SENSITIVITY_16G/16.0f;
        break;
    }


		dev_addr = LSM303DLHC_ACC_BASE_ADDR;
		read_i2c(LSM303DLHC_WHO_AM_I_ADDR, &whoami, 1);
		if( !(whoami == I_AM_LMS303DLHC) )
		{
				return true;
		}
		
		_i2c->write(dev_addr, (char*)&buf, 5, 0);

		read_i2c(LSM303DLHC_CTRL_REG1_A | 0x80, buf, 4);
		
    return false;
}

bool LSM303DLHC::init_mag(uint8_t rate, uint8_t scale, uint8_t enable_temp)
{
    uint8_t buf[4];

    // setup for magnetometer
		buf[0] = LSM303DLHC_CRA_REG_M;
    buf[1] = (enable_temp & 0x80) | (rate & 0x1C);
    buf[2] = scale & 0xE0;
    buf[3] = LSM303DLHC_CONTINUOS_CONVERSION;


		dev_addr = LSM303DLHC_MAG_BASE_ADDR;

		//write_i2c(LSM303DLHC_CRA_REG_M, buf, 3); // configure magnetometer
		_i2c->write(dev_addr, (char*)&buf, 4, 0);
		read_i2c(LSM303DLHC_CRA_REG_M, buf, 3);

		switch(scale)
    {
    case LSM303DLHC_FS_1_3_GA:
        sensitivity_mag_xy = 1.0f/(float)LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
				sensitivity_mag_z = 1.0f/(float)LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;
        break;
    case LSM303DLHC_FS_1_9_GA:
        sensitivity_mag_xy = 1.0f/(float)LSM303DLHC_M_SENSITIVITY_XY_1_9Ga;
				sensitivity_mag_z = 1.0f/(float)LSM303DLHC_M_SENSITIVITY_Z_1_9Ga;
        break;
    case LSM303DLHC_FS_2_5_GA:
        sensitivity_mag_xy = 1.0f/(float)LSM303DLHC_M_SENSITIVITY_XY_2_5Ga;
				sensitivity_mag_z = 1.0f/(float)LSM303DLHC_M_SENSITIVITY_Z_2_5Ga;
        break;
    case LSM303DLHC_FS_4_0_GA:
        sensitivity_mag_xy = 1.0f/(float)LSM303DLHC_M_SENSITIVITY_XY_4Ga;
				sensitivity_mag_z = 1.0f/(float)LSM303DLHC_M_SENSITIVITY_Z_4Ga;
        break;
    case LSM303DLHC_FS_4_7_GA:
        sensitivity_mag_xy = 1.0f/(float)LSM303DLHC_M_SENSITIVITY_XY_4_7Ga;
				sensitivity_mag_z = 1.0f/(float)LSM303DLHC_M_SENSITIVITY_Z_4_7Ga;
        break;
    case LSM303DLHC_FS_5_6_GA:
        sensitivity_mag_xy = 1.0f/(float)LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;
				sensitivity_mag_z = 1.0f/(float)LSM303DLHC_M_SENSITIVITY_Z_5_6Ga;
        break;
    default:
        sensitivity_mag_xy = 1.0f/(float)LSM303DLHC_M_SENSITIVITY_XY_8_1Ga;
				sensitivity_mag_z = 1.0f/(float)LSM303DLHC_M_SENSITIVITY_Z_8_1Ga;
        break;
    }
		
    return false;
}

void LSM303DLHC::get_xyz_raw(uint8_t reg, int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6];
	
    read_i2c(reg, (uint8_t*)buf, 6);

    *x = ((int16_t)((uint16_t)buf[0] << 8) + buf[1]);
	
    *y = ((int16_t)((uint16_t)buf[2] << 8) + buf[3]);
	
    *z = ((int16_t)((uint16_t)buf[4] << 8) + buf[5]);
}

int16_t LSM303DLHC::get_temp_raw(void)
{
    uint8_t buf[2];

		dev_addr = LSM303DLHC_MAG_BASE_ADDR;
		read_i2c(LSM303DLHC_TEMP_OUT_H_M, buf, 2);


    return (buf[0] << 8) | buf[1];
}

void LSM303DLHC::get_xyz_acc(float *x, float *y, float *z)
{
    int16_t x_, y_, z_;

    dev_addr = LSM303DLHC_ACC_BASE_ADDR;

    get_xyz_raw(LSM303DLHC_OUT_X_L_A | 0x80, &x_, &y_, &z_);

    *x = (float)x_ * sensitivity_acc;
    *y = (float)y_ * sensitivity_acc;
    *z = (float)z_ * sensitivity_acc;
}

void LSM303DLHC::get_xyz_mag(float *x, float *y, float *z)
{
    int16_t x_, y_, z_;

		dev_addr = LSM303DLHC_MAG_BASE_ADDR;
	
		get_xyz_raw(LSM303DLHC_OUT_X_H_M, &x_, &y_, &z_);

    *x = (float)x_ * sensitivity_mag_xy;
    *y = (float)y_ * sensitivity_mag_xy;
    *z = (float)z_ * sensitivity_mag_z;
}

float LSM303DLHC::get_temp(void)
{
    return ((float)get_temp_raw()/16.0f)/8.0f;
}

