#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
* Copyright (C) 2019 Bosch Sensortec GmbH
*
* Created 27.11.2019
*
* @author: kro1rt
"""

import sys
import time
import coinespy as BST

# This example works with Application Board 2.0 and BMA400 shuttle board
BMA400_SHUTTLE_ID = 0x1A1

BOARD = BST.UserApplicationBoard()

# create single board instance over USB connection
def bst_init_brd_app20():
    global BOARD
    BOARD.PCInterfaceConfig(BST.PCINTERFACE.USB)

    # print board info
    bi = BOARD.GetBoardInfo()
    print('BoardInfo: HW/SW ID: ' + hex(bi.HardwareId) + '/' + \
          hex(bi.SoftwareId) + ', ShuttleID: ' + hex(bi.ShuttleID))
    if bi.HardwareId == 0:
        print('Seems like there is no board commincation. Stop')
        BOARD.ClosePCInterface()
        sys.exit()
    if bi.ShuttleID != BMA400_SHUTTLE_ID:
        print('Seems like you are not using BMA400 shuttle board. Stop')
        BOARD.ClosePCInterface()
        sys.exit()

# Calculate 2's complement out of given uint value
def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val                         # return positive value as is

# ############################################################################
# Main routine.
if __name__ == "__main__":
    bst_init_brd_app20()

    BMA400_CS_PIN = BST.ShuttleBoardPin.COINES_SHUTTLE_PIN_7
    # Note: The I2C address could be changed to 0x15 by setting a HIGH signal to the SDO pin of
    #   BM400, however this is not possible with the current implementation of the COINES layer
    BMA400_I2C_ADDR = 0x14

    myif = 'spi'
    #myif = 'i2c'
    print('\n +++ Interface: ' + myif + ' +++\n')

    BOARD.SetVDD(0)
    BOARD.SetVDDIO(0)

    if myif == 'spi':
        BOARD.SensorSPIConfig(BMA400_CS_PIN, BST.SPISPEED.SPI1000KBIT, BST.SPIMODE.MODE0)
        BOARD.PinConfig(BMA400_CS_PIN, BST.EONOFF.ON, BST.PINMODE.OUTPUT, BST.PINLEVEL.HIGH)
        BMA400_IF_DETAIL = BMA400_CS_PIN
        SPI_DUMMY_BYTE = 1
    else:
        BOARD.SensorI2CConfig(BMA400_I2C_ADDR, BST.I2CSPEED.STANDARDMODE)
        BOARD.PinConfig(BMA400_CS_PIN, BST.EONOFF.ON, BST.PINMODE.OUTPUT, BST.PINLEVEL.HIGH)
        BMA400_IF_DETAIL = BMA400_I2C_ADDR
        SPI_DUMMY_BYTE = 0

    BOARD.SetVDD(3.3)
    BOARD.SetVDDIO(3.3)
    time.sleep(0.2)

    if myif == 'spi':
        # Dummy-read for accelerometer to switch to SPI mode
        BOARD.Read(0x00, 1, BMA400_IF_DETAIL)

    # Read chip id
    chipid = BOARD.Read(0x00, 1 + SPI_DUMMY_BYTE, BMA400_IF_DETAIL)
    print('Chip ID accel: %#02x' % (chipid[0 + SPI_DUMMY_BYTE]))

    # Power up accelerometer
    pwr_before = BOARD.Read(0x19, 1 + SPI_DUMMY_BYTE, BMA400_IF_DETAIL)[0 + SPI_DUMMY_BYTE]
    print('Power settings before power-up (expected: 0x00): 0x%02x' % (pwr_before))
    # Change power mode from sleep to normal mode
    BOARD.Write(0x19, 0x02, BMA400_IF_DETAIL)
    time.sleep(.2)
    pwr_after = BOARD.Read(0x19, 1 + SPI_DUMMY_BYTE, BMA400_IF_DETAIL)[0 + SPI_DUMMY_BYTE]
    print('ACC: Power settings after power-up (expected: 0x02): 0x%02x' % (pwr_after))

    # Change to measurement range 4G
    acc_config1 = BOARD.Read(0x1A, 1 + SPI_DUMMY_BYTE, BMA400_IF_DETAIL)
    acc_config1[0] = acc_config1[0 + SPI_DUMMY_BYTE] & 0x3F    # clear bit 7
    acc_config1[0] = acc_config1[0] | 0x40    # set bit 6
    accel_range = acc_config1[0] >> 6
    BOARD.Write(0x1A, acc_config1[0], BMA400_IF_DETAIL)
    time.sleep(.2)
    print('\nRange: %dG' % (2 ** (accel_range + 1)))

    # Read some samples from the sensor
    a_lsb = [0] * 3
    a_g = [0] * 3
    print('\n  ax\t   ay\t   az')
    for i in range(10):
        data = BOARD.Read(0x04, 6 + SPI_DUMMY_BYTE, BMA400_IF_DETAIL)
        if myif == 'spi':
            data.pop(0)
        a_lsb[0] = twos_comp(data[1] * 256 + data[0], 12)
        a_lsb[1] = twos_comp(data[3] * 256 + data[2], 12)
        a_lsb[2] = twos_comp(data[5] * 256 + data[4], 12)
        a_g = [(val/2048. * 2 ** (accel_range + 1)) for val in a_lsb]

        print('%+.3f\t %+.3f\t %+.3f' % (a_g[0], a_g[1], a_g[2]))
        time.sleep(0.05) # 200Hz ODR

    # Read temperature sensor data. This may be surprisingly high
    # due to self-heating of the ApplicationBoard2
    data = BOARD.Read(0x11, 1 + SPI_DUMMY_BYTE, BMA400_IF_DETAIL)
    temp = twos_comp(data[0 + SPI_DUMMY_BYTE], 8) * 0.5 + 23
    print('\nTemperature: %.2f' % (temp))

    BOARD.SetVDD(0)
    BOARD.SetVDDIO(0)
    BOARD.ClosePCInterface()
