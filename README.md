# ROS driver for NXP Semiconductor PCA9685 I2C PWM chip

This is a ROS node for the PCA9685. The chip is notably used in the following Adafruit products:

* [Adafruit 16-Channel 12-bit PWM/Servo Driver](https://www.adafruit.com/product/815)
* [Adafruit DC and Stepper Motor HAT for Raspberry Pi](https://www.adafruit.com/product/2348)

There should be no dependencies besides libi2c-dev.

## Parameters:

* **device** -- the path to the i2c device. Default is /dev/i2c-1. Use i2cdetect in the i2c-tools package to find out which bus your IMU is on.
* **address** -- the i2c address of the IMU. Default is 0x28.
* **frequency** -- PWM frequency in Hz. Default is 1600.

## Subscribers
* **command** -- a Int32MultiArray containing exactly 16 values corresponding to the 16 PWM channels. For each value, specify -1 to make no change to a channel. Specify a value between 0 and 65535 inclusive to update the channel's PWM value.

## Publishers
None.

## Services
None.

