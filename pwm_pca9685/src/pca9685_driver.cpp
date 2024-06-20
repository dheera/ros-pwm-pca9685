/* pca9685_activity.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 *
 * Defines a PCA9685 Driver class, constructed with node handles
 * and which handles all ROS duties.
 */

#include "pwm_pca9685/pca9685_activity.h"

namespace pwm_pca9685 {

// ******** constructors ******** //

PCA9685Driver::PCA9685Driver(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv) :
  nh(_nh), nh_priv(_nh_priv) {

}

// ******** private methods ******** //

bool PCA9685Driver::reset() {
    int i = 0;

    // reset
    _i2c_smbus_write_byte_data(file, PCA9685_MODE1_REG, 0b10000000);
    ros::Duration(0.500).sleep();

    // set frequency
    uint8_t prescale = (uint8_t)(25000000.0 / 4096.0 / param_frequency + 0.5);

    _i2c_smbus_write_byte_data(file, PCA9685_MODE1_REG, 0b00010000); // sleep
    ros::Duration(0.025).sleep();

    _i2c_smbus_write_byte_data(file, PCA9685_PRESCALE_REG, prescale); // set prescale
    ros::Duration(0.025).sleep();

    _i2c_smbus_write_byte_data(file, PCA9685_MODE2_REG, 0x04); // outdrv
    ros::Duration(0.025).sleep();

    _i2c_smbus_write_byte_data(file, PCA9685_MODE1_REG, 0xA1); // un-sleep
    ros::Duration(0.025).sleep();

    _i2c_smbus_write_byte_data(file, PCA9685_CHANNEL_ALL_REG, 0);
    _i2c_smbus_write_byte_data(file, PCA9685_CHANNEL_ALL_REG + 1, 0);
    _i2c_smbus_write_byte_data(file, PCA9685_CHANNEL_ALL_REG + 2, 0);
    _i2c_smbus_write_byte_data(file, PCA9685_CHANNEL_ALL_REG + 3, 0);

    return true;
}

bool PCA9685Driver::set(uint8_t channel, uint16_t value) {
    uint16_t value_12bit = value >> 4;
    uint8_t values[4];
    if(value_12bit == 0x0FFF) { // always on
    	values[0] = 0x10;
    	values[1] = 0x00;
    	values[2] = 0x00;
    	values[3] = 0x00;
    } else if(value_12bit == 0x0000) { // always off
    	values[0] = 0x00;
    	values[1] = 0x00;
    	values[2] = 0x10;
    	values[3] = 0x00;
    } else { // PWM
    	values[0] = 0x00;
    	values[1] = 0x00;
    	values[2] = (value_12bit + 1) & 0xFF;
    	values[3] = (value_12bit + 1) >> 8;
    }

    _i2c_smbus_write_i2c_block_data(file, PCA9685_CHANNEL_0_REG + (channel * 4), 4, values);
}

// ******** public methods ******** //

bool PCA9685Driver::start() {
    ROS_INFO("starting");


    return true;
}

}
