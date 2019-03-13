/* pca9685_activity.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 *
 * Defines a PCA9685 Activity class, constructed with node handles
 * and which handles all ROS duties.
 */

#include "pwm_pca9685/pca9685_activity.h"

namespace pwm_pca9685 {

// ******** constructors ******** //

PCA9685Activity::PCA9685Activity(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv) :
  nh(_nh), nh_priv(_nh_priv) {
    ROS_INFO("initializing");
    nh_priv.param("device", param_device, (std::string)"/dev/i2c-1");
    nh_priv.param("address", param_address, (int)PCA9685_ADDRESS);
    nh_priv.param("frequency", param_frequency, (int)1600);
    nh_priv.param("frame_id", param_frame_id, (std::string)"imu");

}

// ******** private methods ******** //

bool PCA9685Activity::reset() {
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

bool PCA9685Activity::set(uint8_t channel, uint16_t value) {
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
    ROS_WARN_STREAM("setting " << int(channel) << " to " << int(value));
    ROS_WARN_STREAM("i2c write " << int(PCA9685_CHANNEL_0_REG + (channel) * 4) << " to " << int(values[0]) << " " << int(values[1]) << " " << int(values[2]) << " " << int(values[3]));
    _i2c_smbus_write_i2c_block_data(file, PCA9685_CHANNEL_0_REG + (channel * 4), 4, values);
}

// ******** public methods ******** //

bool PCA9685Activity::start() {
    ROS_INFO("starting");

    if(!sub_command) sub_command = nh.subscribe("command", 1, &PCA9685Activity::onCommand, this);

    file = open(param_device.c_str(), O_RDWR);
    if(ioctl(file, I2C_SLAVE, param_address) < 0) {
        ROS_ERROR("i2c device open failed");
        return false;
    }

    if(!reset()) {
        ROS_ERROR("chip reset and setup failed");
        return false;
    }

    return true;
}

bool PCA9685Activity::spinOnce() {
    ros::spinOnce();

    ros::Time time = ros::Time::now();

    return true;    
}

bool PCA9685Activity::stop() {
    ROS_INFO("stopping");

    if(sub_command) sub_command.shutdown();

    return true;
}

void PCA9685Activity::onCommand(const std_msgs::Int32MultiArrayPtr &msg) {
    if(msg->data.size() != 16) {
        ROS_ERROR("array is not have a size of 16");
        return;
    }

    for(size_t i = 0; i < msg->data.size(); i++) {
      if(msg->data[i] < 0) continue;
      if(msg->data[i] > 65535) {
          set(i, 65535);
      } else {
          set(i, msg->data[i]);
      }
    }
}


}
