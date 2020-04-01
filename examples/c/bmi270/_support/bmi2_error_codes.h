/**
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file    bmi2_error_codes.h
 *
 */

#ifndef __BMI2_ERROR_CODES_H__
#define __BMI2_ERROR_CODES_H__

/*! CPP guard */
#ifdef __cplusplus
extern "C"
{
#endif
/******************************************************************************/
/*!              Header Files                                  */
#include "bmi2.h"
/******************************************************************************/
/*!              Functions                                  */

/*!
 *  @brief This internal API is used to print the execution status.
 *
 *  @param[in] rslt     : Error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */ 
void print_rslt(int8_t rslt);

#ifdef __cplusplus
}
#endif  /* End of CPP guard */

#endif  /* __BMI2_ERROR_CODES_H__ */ 