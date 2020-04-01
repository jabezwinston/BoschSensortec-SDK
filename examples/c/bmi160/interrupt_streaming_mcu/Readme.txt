#####################################################################################################
This software example stream bmi160 sensor data based on the interrupt using coinesAPI and sensorAPI

Uses bmi08x sensorAPI 
  --> Use sub-members in 'accel_cfg'/'gyro_cfg' member of  bmi160_dev to configure Output Data Rate (ODR),
      Bandwidth (BW) , Range, power modes ,etc.,of Accelerometer/Gyroscope
  --> bmi160_set_sens_conf() to reflect the change in accel/gyro sensor configuration registers
  --> Use 'bmi160_int_settg' structure members and bmi160_set_int_config() API for configuring interrupts

This example works with both I2C and SPI mode.

#####################################################################################################

Map INT1 (COINES_SHUTTLE_PIN_20) to accel./gyro. data ready interrupt using bmi160_set_int_config()

Map COINES_SHUTTLE_PIN_20 to bmi160_drdy_int() callback handler
coines_attach_interrupt(COINES_SHUTTLE_PIN_20, bmi160_drdy_int, COINES_PIN_INTERRUPT_FALLING_EDGE);

In bmi160_drdy_int() callback,check for accel./gyro. data ready interrupt and read data 

#####################################################################################################
