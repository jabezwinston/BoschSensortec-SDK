#####################################################################################################
This software example stream bmi08x sensor data based on the interrupt using coines API

Uses bmi08x sensorAPI
  --> bmi08a_set_power_mode() / bmi08g_set_power_mode() to switch accel/gyro between power mode like sleep ,normal ,etc.,
  --> Use sub-members in 'accel_cfg'/'gyro_cfg' member of  bmi80x_dev to set Output Data Rate (ODR), Bandwidth (BW) , Range
  --> Use bmi08a_set_meas_conf() / bmi08g_set_meas_conf() to reflect the change in accel/gyro sensor measurement configuration registers
  --> Use accel_int_* / gyro_int_* members in int_config structure to configure for interrupts
  --> bmi08a_set_int_config() / bmi08g_set_int_config() to reflect the change in accel/gyro interrupt configuration registers

This example supports both BMI085 and BMI088. Works with both I2C and SPI mode.

Corresponding sensor flag need to be enabled in the sensor API  in bmi08x_defs.h.

/** \name enable bmi085 sensor */
#ifndef BMI08X_ENABLE_BMI085
#define BMI08X_ENABLE_BMI085       1
#endif
/** \name enable bmi088 sensor */
#ifndef BMI08X_ENABLE_BMI088
#define BMI08X_ENABLE_BMI088       0
#endif

                                   (or)
								 
Specify SHUTTLE_BOARD=<BMI08x> as commandline parameter to make utility

Eg : mingw32-make SHUTTLE_BOARD=BMI088

#####################################################################################################

Map INT1 (COINES_SHUTTLE_PIN_21) to accel. data ready interrupt using bmi08a_set_int_config()
Map INT3 (COINES_SHUTTLE_PIN_19) to accel. data ready interrupt using bmi08g_set_int_config()

Map INT1 and INT3 pin interrupts to bmi08x_accel_drdy_int() and bmi08x_gyro_drdy_int() callback handlers

coines_attach_interrupt(COINES_SHUTTLE_PIN_21, bmi08x_accel_drdy_int, COINES_PIN_INTERRUPT_FALLING_EDGE);
coines_attach_interrupt(COINES_SHUTTLE_PIN_22, bmi08x_gyro_drdy_int, COINES_PIN_INTERRUPT_FALLING_EDGE);

Read accel./gyro. data using bmi08a_get_data() / bmi08g_get_data() in bmi08x_accel_drdy_int() 
and bmi08x_gyro_drdy_int() respectively.

#####################################################################################################
