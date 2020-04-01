# BMA400 change log
All notable changes to the BMA400 Sensor API will be documented in this file.

## v1.4.0, 07 Dec 2017
### Changed
- Modified BMA400_defs.h file for linux compatibility
- Activity data WALK/STILL/RUN tested and updated as macros
- Temperature data support is added
- Tap interrupt parameters set/get added
- Orientation feature enabled and verified

## v1.3.1, 26 Sep 2017
### Changed
- Decoupled wakeup interrupt enabling and added as part of enable interrupts 
- Autowakeup timeout and Autowakeup interrupt based setting is removed

## v1.3.0, 08 Sep 2017
### Added
- Getting the interrupt enabled/disabled and the INT pin mapping in the sensor
- Self test, Temperature data implementation
- Interrupt overrun handled
- Activity data made available (RUN/WALK/STILL)

## v1.2.0, 30 Aug 2017
### Added
- FIFO configuration settings
- FIFO support for the BMA400 to read data in 12/8 bit mode
- FIFO watermark , FIFO full interrupts

## v1.1.1, 08 Aug 2017
### Changed
- Step counter value excluded 0x18 register
- Reading of 0x19 register so that power mode remains unchanged

## v1.1.0, 28 Jul 2017
### Added
- BMA400 support to configure and read the following interrupt settings and interrupt status 
	 - GENERIC 1 / GENERIC 2 interrupt
	 - Data ready interrupt
	 - Wakeup interrupt
	 - Activity change interrupt
	 - Auto wakeup / Auto low-power features
	 
## v1.0.0, 5 Jul 2017
### Added
- Basic API implementation with the following APIs ready to use
	 - bma400_init
	 - bma400_set_regs
	 - bma400_get_regs
	 - bma400_soft_reset
	 - bma400_set_power_mode
	 - bma400_get_power_mode
	 - bma400_get_accel_data
	 - bma400_set_sensor_setting  (Only basic accel configurations set/get done)
	 - bma400_get_sensor_setting