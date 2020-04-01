#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
* Copyright (C) 2019 Bosch Sensortec GmbH
*
* Created 29.11.2019
"""

import sys
import time
import re
import coinespy as BST

# This example works with ApplicationBoard2.0 und BME280 shuttle board
BME280_SHUTTLE_ID = 0x33

# Read sensor specific CONSTANTS from the header file bme280_defs.h
CONSTANTS = {}
with open(r'../../sensorAPI/bme280/bme280_defs.h') as hfile:
    lines = hfile.readlines()
    for line in lines:
        try:
            items = re.findall(r'#define\s+(\w+)\s+\w+(.*)', line)
            CONSTANTS[items[0][0]] = eval(items[0][1])
        except:
            pass
        try:
            items = re.findall(r'#define\s+(\w+)\s+(.*)', line)
            CONSTANTS[items[0][0]] = eval(items[0][1])
        except:
            pass

class BME280CalibData(object):
    def __init__(self, calib_t_p, calib_h):
        self.dig_T1 = calib_t_p[1] * 256 + calib_t_p[0]
        self.dig_T2 = twos_comp(calib_t_p[3] * 256 + calib_t_p[2], 16)
        self.dig_T3 = twos_comp(calib_t_p[5] * 256 + calib_t_p[4], 16)
        self.dig_P1 = calib_t_p[7] * 256 + calib_t_p[6]
        self.dig_P2 = twos_comp(calib_t_p[9] * 256 + calib_t_p[8], 16)
        self.dig_P3 = twos_comp(calib_t_p[11] * 256 + calib_t_p[10], 16)
        self.dig_P4 = twos_comp(calib_t_p[13] * 256 + calib_t_p[12], 16)
        self.dig_P5 = twos_comp(calib_t_p[15] * 256 + calib_t_p[14], 16)
        self.dig_P6 = twos_comp(calib_t_p[17] * 256 + calib_t_p[16], 16)
        self.dig_P7 = twos_comp(calib_t_p[19] * 256 + calib_t_p[18], 16)
        self.dig_P8 = twos_comp(calib_t_p[21] * 256 + calib_t_p[20], 16)
        self.dig_P9 = twos_comp(calib_t_p[23] * 256 + calib_t_p[22], 16)
        self.dig_H1 = calib_t_p[25]
        self.dig_H2 = twos_comp(calib_h[1] * 256 + calib_h[0], 16)
        self.dig_H3 = calib_h[2]
        self.dig_H4 = twos_comp(calib_h[3] * 16 + (calib_h[4] & 0x0F), 12)
        self.dig_H5 = twos_comp(calib_h[5] * 16 + calib_h[4] >> 4, 12)
        self.dig_H6 = twos_comp(calib_h[6], 8)
        self.t_fine = 0

# create single board instance over USB connection
def bst_init_brd_app20():
    BOARD.PCInterfaceConfig(BST.PCINTERFACE.USB)

    # print board info
    bi = BOARD.GetBoardInfo()
    print('BoardInfo: HW/SW ID: 0x%02x/0x%02x, ShuttleID: 0x%02x' %\
                                    (bi.HardwareId, bi.SoftwareId, bi.ShuttleID))
    if bi.HardwareId == 0:
        print('Seems like there is no board commincation. Stop')
        BOARD.ClosePCInterface()
        sys.exit()
    if bi.ShuttleID != BME280_SHUTTLE_ID:
        print('Seems like you are not using BME280 shuttle board. Stop')
        BOARD.ClosePCInterface()
        sys.exit()

# Calculate 2's complement out of given uint value
def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is

def compensate_humidity(raw_hum, calib_data):
    var1 = calib_data.t_fine - 76800.
    var2 = calib_data.dig_H4 * 64. + calib_data.dig_H5 / 16384. * var1
    var3 = raw_hum - var2
    var4 = calib_data.dig_H2 / 65536.
    var5 = 1.0 + calib_data.dig_H3 / 67108864. * var1
    var6 = 1.0  + calib_data.dig_H6 / 67108864. * var1 * var5
    var6 = var3 * var4 * var5 * var6
    humidity = var6 * (1.0 - calib_data.dig_H1 * var6 / 524288.)
    if humidity > 100.:
        return 100.0
    if humidity < 0.0:
        return 0.0
    return humidity

def compensate_temperture(raw_temp, calib_data):
    var1 = (raw_temp / 16384. - calib_data.dig_T1 / 1024.) * calib_data.dig_T2
    var2 = raw_temp / 131072. - calib_data.dig_T1 / 8192.
    var2 = var2 * var2 * calib_data.dig_T3
    calib_data.t_fine = var1 + var2
    temp = (var1 + var2) / 5120.
    if temp > 85:
        return 85
    if temp < -40:
        return -40
    return temp

def compensate_pressure(raw_press, calib_data):
    var1 = calib_data.t_fine / 2. - 64000.
    var2 = var1 * var1 * calib_data.dig_P6 / 32768. + var1 * calib_data.dig_P5 * 2.
    var2 = var2 / 4. + calib_data.dig_P4 * 65536.
    var3 = calib_data.dig_P3 * var1 * var1 / 524288.
    var1 = (var3 + calib_data.dig_P2 * var1) / 524288.
    var1 = (1. + var1 / 32768.) * calib_data.dig_P1
    if var1 == 0:   # avoid exception caused by division by zero
        return 30000.0  # Pressure(min)

    pressure = 1048576. - raw_press
    pressure = (pressure - var2 / 4096.) * 6250. / var1
    var1 = calib_data.dig_P9 * pressure * pressure / 2147483648.
    var2 = pressure * calib_data.dig_P8 / 32768.
    pressure = pressure + (var1 + var2 + calib_data.dig_P7) / 16.
    if pressure < 30000.:
        return 30000.
    if pressure > 110000.0:
        return 110000.0
    return pressure

# ############################################################################
# Main routine.
if __name__ == "__main__":
    # Initialization of the Application Board 2.0
    BOARD = BST.UserApplicationBoard()

    bst_init_brd_app20()

    BME280_CS_PIN = BST.ShuttleBoardPin.COINES_SHUTTLE_PIN_7
    # Note: The I2C address could be changed to 0x77 by setting a HIGH signal to the SDO pin of
    # BME280, however this is not possible with the current implementation of the COINES layer
    BME280_I2C_ADDR = CONSTANTS['BME280_I2C_ADDR_PRIM']

    #myif = 'spi'
    myif = 'i2c'
    print('\n +++ Interface: ' + myif + ' +++\n')

    BOARD.SetVDD(0)
    BOARD.SetVDDIO(0)

    if myif == 'spi':
        BOARD.SensorSPIConfig(BME280_CS_PIN, BST.SPISPEED.SPI1000KBIT, BST.SPIMODE.MODE0)
        BOARD.PinConfig(BME280_CS_PIN, BST.EONOFF.ON, BST.PINMODE.OUTPUT, BST.PINLEVEL.HIGH)
        BME280_IF_DETAIL = BME280_CS_PIN
    else:
        BOARD.SensorI2CConfig(BME280_I2C_ADDR, BST.I2CSPEED.STANDARDMODE)
        BOARD.PinConfig(BME280_CS_PIN, BST.EONOFF.ON, BST.PINMODE.OUTPUT, BST.PINLEVEL.HIGH)
        BME280_IF_DETAIL = BME280_I2C_ADDR

    BOARD.SetVDD(3.3)
    BOARD.SetVDDIO(3.3)
    time.sleep(0.2)

    # Read chip id
    chipid = BOARD.Read(CONSTANTS['BME280_CHIP_ID_ADDR'], 1, BME280_IF_DETAIL)
    print('Chip ID: %#02x' % (chipid[0]))

    # Configure the sensor for operation
    osr_h = CONSTANTS['BME280_OVERSAMPLING_1X'] << CONSTANTS['BME280_CTRL_HUM_POS']
    BOARD.Write(CONSTANTS['BME280_CTRL_HUM_ADDR'], osr_h, BME280_IF_DETAIL)

    fltcoeff = CONSTANTS['BME280_FILTER_COEFF_16'] << CONSTANTS['BME280_FILTER_POS']
    stby = CONSTANTS['BME280_STANDBY_TIME_62_5_MS'] << CONSTANTS['BME280_STANDBY_POS']
    config = fltcoeff | stby
    BOARD.Write(CONSTANTS['BME280_CONFIG_ADDR'], config, BME280_IF_DETAIL)

    osr_p = CONSTANTS['BME280_OVERSAMPLING_16X'] << CONSTANTS['BME280_CTRL_PRESS_POS']
    osr_t = CONSTANTS['BME280_OVERSAMPLING_2X'] << CONSTANTS['BME280_CTRL_TEMP_POS']
    mode = CONSTANTS['BME280_NORMAL_MODE'] << CONSTANTS['BME280_SENSOR_MODE_POS']
    ctrl_meas = osr_p | osr_t | mode
    BOARD.Write(CONSTANTS['BME280_CTRL_MEAS_ADDR'], ctrl_meas, BME280_IF_DETAIL)

    # Get calibration data
    calib_t_p = BOARD.Read(CONSTANTS['BME280_TEMP_PRESS_CALIB_DATA_ADDR'],\
                            CONSTANTS['BME280_TEMP_PRESS_CALIB_DATA_LEN'], BME280_IF_DETAIL)
    calib_h = BOARD.Read(CONSTANTS['BME280_HUMIDITY_CALIB_DATA_ADDR'],\
                                CONSTANTS['BME280_HUMIDITY_CALIB_DATA_LEN'], BME280_IF_DETAIL)
    calib_data = BME280CalibData(calib_t_p, calib_h)

    # Read some samples from the sensor
    print('\nPressure Temp\tHum')
    for i in range(10):
        data = BOARD.Read(CONSTANTS['BME280_DATA_ADDR'], CONSTANTS['BME280_P_T_H_DATA_LEN'],\
                                                                            BME280_IF_DETAIL)
        press_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        hum_raw = (data[6] << 8) | data[7]

        temp = compensate_temperture(temp_raw, calib_data)
        hum = compensate_humidity(hum_raw, calib_data)
        press = compensate_pressure(press_raw, calib_data)

        print('%d\t %.2f\t%.2f' % (press, temp, hum))
        time.sleep(0.5) # 2Hz

    BOARD.SetVDD(0)
    BOARD.SetVDDIO(0)
    BOARD.ClosePCInterface()
