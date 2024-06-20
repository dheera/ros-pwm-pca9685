/* pca9685_i2c_node.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 *
 * Instantiates a PCA9685 Activity class, as well as
 * a Watchdog that causes this node to die if things aren't
 * working.
 */

#include <pwm_pca9685/pca9685_activity.h>
#include <csignal>

PCA9685Node::PCA9685Node(int argc, char* argv[]) {
    ros::init(argc, argv, "pca9685_node");
    nh = new ros::NodeHandle();
    nh_priv = new ros::NodeHandle("~");

    if(!nh || !nh_priv) {
        ROS_FATAL("Failed to initialize node handles");
        ros::shutdown();
        return;
    }

    nh_priv.param("device", param_device, (std::string)"/dev/i2c-1");
    nh_priv.param("address", param_address, (int)PCA9685_ADDRESS);
    nh_priv.param("frequency", param_frequency, (int)1600);
    nh_priv.param("frame_id", param_frame_id, (std::string)"imu");
    
    // timeouts in milliseconds per channel
    nh_priv.param("timeout", param_timeout, std::vector<int>{
        5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000,
        5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000
    });

    // minimum pwm value per channel
    nh_priv.param("pwm_min", param_pwm_min, std::vector<int>{
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0
    });

    // maximum pwm value per channel
    nh_priv.param("pwm_max", param_pwm_max, std::vector<int>{
        65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535,
        65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535
    });

    // default pwm value per channel after timeout is reached
    nh_priv.param("timeout_value", param_timeout_value, std::vector<int>{
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0
    });

    if(param_timeout.size() != 16) {
        ROS_ERROR("size of param timeout must be 16");
        ros::shutdown();
    }

    if(param_timeout_value.size() != 16) {
        ROS_ERROR("size of param timeout_value must be 16");
        ros::shutdown();
    }

    if(param_pwm_min.size() != 16) {
        ROS_ERROR("size of param pwm_min must be 16");
        ros::shutdown();
    }

    if(param_pwm_max.size() != 16) {
        ROS_ERROR("size of param pwm_min must be 16");
        ros::shutdown();
    }

    if(param_address < 0 || param_address > 127) {
        ROS_ERROR("param address must be between 0 and 127 inclusive");
        ros::shutdown();
    }

    if(param_frequency <= 0) {
        ROS_ERROR("param frequency must be positive");
        ros::shutdown();
    }

    pca9685 = std::make_unique<pca9685::PCA9685Driver>(param_device, param_address);
    pca9685->init();

    file = open(param_device.c_str(), O_RDWR);
    if(ioctl(file, I2C_SLAVE, param_address) < 0) {
        ROS_ERROR("i2c device open failed");
        return false;
    }

    if(!reset()) {
        ROS_ERROR("chip reset and setup failed");
        return false;
    }

    if(!sub_command) sub_command = nh.subscribe("command", 1, &PCA9685Driver::onCommand, this);

    ros::Time time = ros::Time::now();
    uint64_t t = 1000 * (uint64_t)time.sec + (uint64_t)time.nsec / 1e6;

    for(int channel = 0; channel < 16; channel++) {
      last_set_times[channel] = t;
      last_change_times[channel] = t;
      last_data[channel] = 0;
    }
}

void PCA9685Node::onCommand(const std_msgs::Int32MultiArrayPtr &msg) {
    ros::Time time = ros::Time::now();
    uint64_t t = 1000 * (uint64_t)time.sec + (uint64_t)time.nsec / 1e6;

    if(msg->data.size() != 16) {
        ROS_ERROR("array is not have a size of 16");
        return;
    }

    for(int channel = 0; channel < 16; channel++) {
      if(msg->data[channel] < 0) continue;

      if(msg->data[channel] != last_data[channel]) {
          last_change_times[channel] = t;
      }

      if(msg->data[channel] == last_data[channel] && param_timeout[channel]) continue;

      if(msg->data[channel] > param_pwm_max[channel]) {
	  set(channel, param_pwm_max[channel]);
      } else if(msg->data[channel] < param_pwm_min[channel]) {
          set(channel, param_pwm_min[channel]);
      } else {
          set(channel, msg->data[channel]);
      }
      last_set_times[channel] = t;
      last_data[channel] = msg->data[channel];
    }
}

void BNO055I2CNode::run() {
    while(ros::ok()) {
        rate->sleep();
    	ros::spinOnce();
    	ros::Time time = ros::Time::now();
        uint64_t t = 1000 * (uint64_t)time.sec + (uint64_t)time.nsec / 1e6;

	    if(seq++ % 10 == 0) {
	      for(int channel = 0; channel < 16; channel++) {
	        // positive timeout: timeout when no cammand is received
	        if(param_timeout[channel] > 0 && t - last_set_times[channel] > std::abs(param_timeout[channel])) {
	          set(channel, param_timeout_value[channel]);
	        }
	        // negative timeout: timeout when value doesn't change
		else if(param_timeout[channel] < 0 && t - last_change_times[channel] > std::abs(param_timeout[channel])) {
	          set(channel, param_timeout_value[channel]);
		  ROS_WARN_STREAM("timeout " << channel);
	        }
		// zero timeout: no timeout
	      }
    	}
    } 
}

void PCA9685Node::stop() {
    ROS_INFO("stopping");
    if(sub_command) sub_command.shutdown();
}

int main(int argc, char *argv[]) {
    PCA9685Node node(argc, argv);
    node.run();
    return 0;
}

