#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
* Copyright (C) 2019 Bosch Sensortec GmbH
*
* Created 21.11.2019
*
* @author: kro1rt
"""

import sys
import time
import coinespy as BST

# This example works with ApplicationBoard2.0 und BMI085 shuttle board
BMI085_SHUTTLE_ID = 0x46

BOARD = BST.UserApplicationBoard()

# This class contains the relevant information about the sensor. 
# In case of BMI08x, there are 2 class initializations neeeded, one
# for accelerometer, the other one for the gyroscope.
class Sensor(object):
    def __init__(self, ident, i2c_addr=None, CSB=None):
        self.ident = ident
        self.i2c_addr = i2c_addr
        self.CSB = CSB

    def get_if_detail(self):
        if self.CSB and not self.i2c_addr:
            return self.CSB
        elif self.i2c_addr and not self.CSB:
            return self.i2c_addr
        else:
            return -1

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
    if bi.ShuttleID != BMI085_SHUTTLE_ID:
        print('Seems like you are not using BMI085 shuttle board. Stop')
        BOARD.ClosePCInterface()
        sys.exit()

# Function to read from the previous selected bus
def brd_read(sensor, reg, num_of_bytes):
    global BOARD
    # Consider the dummy byte in case of SPI mode for the accelerometer
    if (sensor.ident == 'accel') and (sensor.get_if_detail() == sensor.CSB):
        num_of_bytes += 1
    data = BOARD.Read(reg & 0x7F, num_of_bytes, sensor.get_if_detail())
    # Remove the dummy byte in case of SPI mode for the accelerometer
    if (sensor.ident == 'accel') and (sensor.get_if_detail() == sensor.CSB):
        data.pop(0)
    return data

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

    #myif = 'spi'
    myif = 'i2c'
    print('\n +++ Interface: ' + myif + ' +++\n')

    if myif == 'spi':
        bmi08x_accel = Sensor('accel', CSB=BST.ShuttleBoardPin.COINES_SHUTTLE_PIN_8)
        bmi08x_gyro = Sensor('gyro', CSB=BST.ShuttleBoardPin.COINES_SHUTTLE_PIN_14)
    else:
        bmi08x_accel = Sensor('accel', i2c_addr=0x18)
        bmi08x_gyro = Sensor('gyro', i2c_addr=0x68)

    BOARD.SetVDD(0)
    BOARD.SetVDDIO(0)

    if myif == 'spi':
        BOARD.SensorSPIConfig(bmi08x_accel.CSB, BST.SPISPEED.SPI1000KBIT, BST.SPIMODE.MODE0)
        BOARD.SensorSPIConfig(bmi08x_gyro.CSB, BST.SPISPEED.SPI1000KBIT, BST.SPIMODE.MODE0)
        BOARD.PinConfig(bmi08x_accel.CSB, BST.EONOFF.ON, BST.PINMODE.OUTPUT, BST.PINLEVEL.HIGH)
        BOARD.PinConfig(bmi08x_gyro.CSB, BST.EONOFF.ON, BST.PINMODE.OUTPUT, BST.PINLEVEL.HIGH)
        # Set PS pin of gyro to LOW for proper protocol selection
        BOARD.PinConfig(BST.ShuttleBoardPin.COINES_SHUTTLE_PIN_9, BST.EONOFF.ON,\
                                        BST.PINMODE.OUTPUT, BST.PINLEVEL.LOW)
    else:
        BOARD.SensorI2CConfig(bmi08x_accel.i2c_addr, BST.I2CSPEED.STANDARDMODE)
        BOARD.SensorI2CConfig(bmi08x_gyro.i2c_addr, BST.I2CSPEED.STANDARDMODE)
        # Set PS pin of gyro to HIGH for proper protocol selection
        BOARD.PinConfig(BST.ShuttleBoardPin.COINES_SHUTTLE_PIN_9, BST.EONOFF.ON,\
                                        BST.PINMODE.OUTPUT, BST.PINLEVEL.HIGH)

    BOARD.SetVDD(3.3)
    BOARD.SetVDDIO(3.3)
    time.sleep(0.1)

    if myif == 'spi':
        # Dummy-read for accelerometer to switch to spi mode
        brd_read(bmi08x_accel, 0, 1)

    # Read chip id
    chipid = brd_read(bmi08x_accel, 0, 1)
    print('Chip ID accel: ' + hex(chipid[0]))
    chipid = brd_read(bmi08x_gyro, 0, 1)
    print('Chip ID gyro: ' + hex(chipid[0]))

    # Power up accelerometer: Clear ACC_PWR_CONF and write 4 to ACC_PWR_CTRL
    pwr_before = brd_read(bmi08x_accel, 0x7C, 2)
    print('ACC: Power settings before power-up (expected: [3, 0]): %s' % (pwr_before))
    
    # You can write an array, but you need to consider long wait time aftr writing
    # to ensuer changes get effective.
    #BOARD.Write(0x7C, [0, 4], bmi08x_accel.get_if_detail())
    #time.sleep(1)
    #pwr_after = brd_read(bmi08x_accel, 0x7C, 2)
    #print('ACC: Power settings after power-up (expected: [0, 4]): %s' % (pwr_after))
    
    # Writing one byte after the next:
    BOARD.Write(0x7C, 0, bmi08x_accel.get_if_detail())
    BOARD.Write(0x7D, 4, bmi08x_accel.get_if_detail())
    time.sleep(.05)
    pwr_after = brd_read(bmi08x_accel, 0x7C, 2)
    print('ACC: Power settings after power-up (expected: [0, 4]): %s' % (pwr_after))
    
    #DEBUG: accel_range = brd_read(bmi08x_accel, 0x41, 1)[0]
    #DEBUG: print(accel_range)      # Should show 1
    # Set range to 2G (BMI085) / 3G (BMI088)
    BOARD.Write(0x41, 0x00, bmi08x_accel.get_if_detail())
    time.sleep(.05)
    # Read back
    accel_range = brd_read(bmi08x_accel, 0x41, 1)[0]
    print('ACC: Range set to 2G (i.e. should show 0): %d' % (accel_range))      # Should show 0
    
    # Get current range setting of gyro. Certainly the default value, as we have not
    # modified them after startup. Commands given for demonstration purposes.
    gyro_range = brd_read(bmi08x_gyro, 0x0F, 1)[0]
    print('GYRO: Range set to 2000dps (i.e. should show 0): %d' % (gyro_range))

    # Factors required for the converion from LSB to phys. world
    GYRO_FULLRANGE = 2000
    ACCEL_FULLRANGE = 2

    # Read some samples from the sensor
    a_lsb = [0] * 3
    g_lsb = [0] * 3
    a_g = [0] * 3
    g_dps = [0] * 3
    print('\n  ax\t   ay\t   az\t    gx\t     gy\t     gz')
    for i in range(10):
        data = brd_read(bmi08x_accel, 0x12, 6)
        a_lsb[0] = twos_comp(data[1] * 256 + data[0], 16)
        a_lsb[1] = twos_comp(data[3] * 256 + data[2], 16)
        a_lsb[2] = twos_comp(data[5] * 256 + data[4], 16)
        a_g = [(val/32768. * ACCEL_FULLRANGE * 2 ** accel_range) for val in a_lsb]

        data = brd_read(bmi08x_gyro, 0x02, 6)
        g_lsb[0] = twos_comp(data[1] * 256 + data[0], 16)
        g_lsb[1] = twos_comp(data[3] * 256 + data[2], 16)
        g_lsb[2] = twos_comp(data[5] * 256 + data[4], 16)
        g_dps = [(val/32768. * GYRO_FULLRANGE * 2 ** gyro_range) for val in g_lsb]

        print('%+.3f\t %+.3f\t %+.3f\t  %+07.1f  %+07.1f  %+07.1f' %\
                                    (a_g[0], a_g[1], a_g[2], g_dps[0], g_dps[1], g_dps[2]))

    # Read temperature sensor data. This may be surprisingly high
    # due to self-heating of the Application Board 2.0
    data = brd_read(bmi08x_accel, 0x22, 2)
    temp = twos_comp(data[0] * 8 + (data[1] >> 5), 11) * 0.125 + 23
    print('\nTemperature: %.2f' % (temp))

    BOARD.SetVDD(0)
    BOARD.SetVDDIO(0)
    BOARD.ClosePCInterface()
