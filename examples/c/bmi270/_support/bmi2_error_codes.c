/**
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi2_error_codes.c
 *
 */

/******************************************************************************/
/*!            Header Files                                  */
#include <stdio.h>
#include "bmi2.h"

/*!
 * @brief This internal API prints the execution status
 */
void print_rslt(int8_t rslt)
{
    switch (rslt)
    {
        case BMI2_OK:
            /*! Do nothing */
            break;
		
		/*! This occurs when the user tries to assign value (not address) to a pointer, which has been initialized to NULL. */
        case BMI2_E_NULL_PTR:
            printf("Error [%d] : Null pointer\r\n", rslt);
            break;
			
		/*! This occurs due to read/write operation failure */	
        case BMI2_E_COM_FAIL:
            printf("Error [%d] : Communication failure\r\n", rslt);
            break;
			
		/*! This occurs when the device chip id is incorrectly read */		
        case BMI2_E_DEV_NOT_FOUND:
            printf("Error [%d] : Device not found\r\n", rslt);
            break;
			
		/*! This occurs when there is a mismatch in the requested feature with the available one */		
        case BMI2_E_INVALID_SENSOR:
            printf("Error [%d] : Invalid sensor\r\n", rslt);
            break;
			
		/*! This occurs when the validation of accel self test data is not satisfied */		
        case BMI2_E_SELF_TEST_FAIL:
            printf("Error [%d] : Self test failed\r\n", rslt);
            break;
			
		/*! This occurs when user tries to configure apart from INT1 and INT2 */		
        case BMI2_E_INVALID_INT_PIN:
            printf("Error [%d] : Invalid interrupt pin\r\n", rslt);
            break;
			
		/*! This occurs when the user enabled apart from filtered or unfiltered data from fifo */	
		case BMI2_E_OUT_OF_RANGE:
            printf("Error [%d] : Out of range\r\n", rslt);
            break;	
			
		/*! This occurs when there is an error in accel configuration register which could be one among range, BW or filter performance in reg address 0x40 */	
		case BMI2_E_ACC_INVALID_CFG:
            printf("Error [%d] : Invalid Accel configuration\r\n", rslt);
            break;	
			
		/*! This occurs when there is a error in gyro configuration register which could be one among range, BW or filter performance in reg address 0x42 */	
		case BMI2_E_GYRO_INVALID_CFG:
            printf("Error [%d] : Invalid Gyro configuration\r\n", rslt);
            break;	
			
		/*! This occurs when there is a error in accel and gyro configuration registers which could be one among range, BW or filter performance in reg address 0x40 and 0x42 */	
		case BMI2_E_ACC_GYR_INVALID_CFG:
            printf("Error [%d] : Invalid Accel-Gyro configuration\r\n", rslt);
            break;	
			
		/*! This occurs when failure observed while loading the configuration into the sensor */	
		case BMI2_E_CONFIG_LOAD:
            printf("Error [%d] : Configuration load error\r\n", rslt);
            break;	
			
		/*! This occurs due to failure in writing the correct feature configuration from selected page */	
		case BMI2_E_INVALID_PAGE:
            printf("Error [%d] : Invalid page\r\n", rslt);
            break;	
			
		/*! This occurs due to failure in write of advance power mode configuration register */	
		case BMI2_E_SET_APS_FAIL:
            printf("Error [%d] : APS failure\r\n", rslt);
            break;	
			
		/*! This occurs when the auxiliary interface settings are not enabled properly */	
		case BMI2_E_AUX_INVALID_CFG:
            printf("Error [%d] : Invalid AUX configuration\r\n", rslt);
            break;	
		
  		/*! This occurs when the auxiliary interface buses are engaged while configuring the AUX */
		case BMI2_E_AUX_BUSY:
            printf("Error [%d] : AUX busy\r\n", rslt);
            break;	
			
		/*! This occurs due to failure in assigning the remap axes data for all the axes after change in axis position */
		case BMI2_E_REMAP_ERROR:
            printf("Error [%d] : Remap error\r\n", rslt);
            break;	
		
        /*! This occurs when the reading of user gain update status fails */		
		case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
            printf("Error [%d] : Gyro user gain UPD fail\r\n", rslt);
            break;	
			
		/*! This occurs when the self test process is ongoing or not completed */	
		case BMI2_E_SELF_TEST_NOT_DONE:
            printf("Error [%d] : Self test not done\r\n", rslt);
            break;	
			
		/*! This occurs when the sensor input validity fails */	
		case BMI2_E_INVALID_INPUT:
            printf("Error [%d] : Invalid input\r\n", rslt);
            break;	
			
		/*! This occurs when the feature/sensor validity fails */	
		case BMI2_E_INVALID_STATUS:
            printf("Error [%d] : Invalid status\r\n", rslt);
            break;	
			
		/*! This occurs when the CRT test has failed */	
		case BMI2_E_CRT_ERROR:
            printf("Error [%d] : CRT error\r\n", rslt);
            break;	
			
		/*! This occurs when the self test is already running and another has been initiated */	
		case BMI2_E_ST_ALREADY_RUNNING:
            printf("Error [%d] : ST already running\r\n", rslt);
            break;	
			
		/*! This occurs when download in CRT fails due to wrong address location */	
		case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
            printf("Error [%d] : CRT ready for DL fail abort\r\n", rslt);
            break;	
			
		/*! This occurs when write length exceeds that of the maximum burst length */	
		case BMI2_E_DL_ERROR:
            printf("Error [%d] : DL error\r\n", rslt);
            break;	
			
		/*! This occurs when precondition to start the feature was not completed */	
		case BMI2_E_PRECON_ERROR:
            printf("Error [%d] : Preconditional error\r\n", rslt);
            break;	
			
		/*! This occurs when the device was shaken during CRT test */	
		case BMI2_E_ABORT_ERROR:
            printf("Error [%d] : Abort error\r\n", rslt);
            break;	
			
		/*! This occurs when the write cycle is already running and another has been initiated */	
		case BMI2_E_WRITE_CYCLE_ONGOING:
            printf("Error [%d] : Write cycle ongoing\r\n", rslt);
            break;	
			
		/*! This occurs when ST running is disabled while it's running */	
		case BMI2_E_ST_NOT_RUNING:
            printf("Error [%d] : ST not running\r\n", rslt);
            break;	
			
		/*! This occurs when the sample count exceeds the FOC sample limit and drdy status doesn't get updated */	
		case BMI2_E_DATA_RDY_INT_FAILED:
            printf("Error [%d] : Data ready interrupt failed\r\n", rslt);
            break;	
			
		/*! This occurs when average FOC data is obtained for the wrong axes */	
		case BMI2_E_INVALID_FOC_POSITION:
            printf("Error [%d] : Invalid FOC position\r\n", rslt);
            break;
        default:
            printf("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}




