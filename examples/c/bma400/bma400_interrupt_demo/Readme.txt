Overview
========
This example shows a simple configuration of BMA400 ultra low power sensor with the following behavior:
 * The sensor shall be in low power mode, if there is no activity detected
 * It should wake up to normal mode once moved. The example reads the data from the internal register
 * If the orientation is changed, the orientation change interrupt is asserted
    - 2 conditions must be met in order to trigger the orientation change interrupt:
        1. At least a certain minimum acceleration change must be visible (in the example 80mg on any axis, based on data from ACC_FILT_LP)
        2. New orientation is detected by a stability condition (signals stay within 80mg threshold for at least 1s)
 * If there is no movement for 200ms and the board is held in another orientation than before, the orientation changed interrupt is fired
 * If the sensor is placed again in horizontal position and there is no movement for 1s, the sensor goes back to low power mode
 * If more precise sensor data with respect to ODR is required, then the user has to configure the respective interrupts and read data accordingly
    
Sensor data print format
========================
 * First value 't':  Sensortime in LSB
 * Values 'ax', 'ay', 'az': acceleration data in LSB, the sensor is configured in 2g mode.
 * Last value 'int': showing interrupt status, where each bit has the following meaning (taken from bma400_defs.h):

 /**\name Interrupt Assertion status macros */
#define BMA400_WAKEUP_INT_ASSERTED      UINT16_C(0x0001)
#define BMA400_ORIENT_CH_INT_ASSERTED   UINT16_C(0x0002)
#define BMA400_GEN1_INT_ASSERTED        UINT16_C(0x0004)
#define BMA400_GEN2_INT_ASSERTED        UINT16_C(0x0008)
#define BMA400_INT_OVERRUN_ASSERTED     UINT16_C(0x0010)
#define BMA400_FIFO_FULL_INT_ASSERTED   UINT16_C(0x0020)
#define BMA400_FIFO_WM_INT_ASSERTED     UINT16_C(0x0040)
#define BMA400_DRDY_INT_ASSERTED        UINT16_C(0x0080)
#define BMA400_STEP_INT_ASSERTED        UINT16_C(0x0300)
#define BMA400_S_TAP_INT_ASSERTED       UINT16_C(0x0400)
#define BMA400_D_TAP_INT_ASSERTED       UINT16_C(0x0800)
#define BMA400_ACT_CH_X_ASSERTED        UINT16_C(0x2000)
#define BMA400_ACT_CH_Y_ASSERTED        UINT16_C(0x4000)
#define BMA400_ACT_CH_Z_ASSERTED        UINT16_C(0x8000)

