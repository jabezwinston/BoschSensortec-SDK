#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
* Copyright (C) 2020 Bosch Sensortec GmbH
*
* Created 17.03.2020
*
"""

import time
import coinespy as BST

# This example works with Application Board 2.0 and any sensor shuttle supports
# 16bit SPI register read and write

BOARD = BST.UserApplicationBoard()

# create single board instance over USB connection
def bst_init_brd_app20():
    "Init USB interface"
    BOARD.PCInterfaceConfig(BST.PCINTERFACE.USB)

# ############################################################################
# Main routine.
if __name__ == "__main__":
    bst_init_brd_app20()

    BMA400_CS_PIN = BST.ShuttleBoardPin.COINES_SHUTTLE_PIN_7

    BOARD.SetVDD(0)
    BOARD.SetVDDIO(0)

    # 16bit SPI interface config
    BOARD.Sensor16bitSPIConfig(BMA400_CS_PIN, BST.SPISPEED.SPI1000KBIT, \
                               BST.SPIMODE.MODE0, BST.SPIBITS.SPI16BIT)
    BOARD.PinConfig(BMA400_CS_PIN, BST.EONOFF.ON, BST.PINMODE.OUTPUT, \
                                                BST.PINLEVEL.HIGH)

    BOARD.SetVDD(3.3)
    BOARD.SetVDDIO(3.3)
    time.sleep(0.2)

    reg_addr = 0x0020
    write_data = [0x1011]
    for count in range(5):
        # Writing data
        ret = BOARD.Write(reg_addr, write_data, BMA400_CS_PIN)
        print("\nregister address: ", hex(reg_addr))
        print("write data:", hex(write_data[0]))

        # Read back the data
        reg_data = BOARD.Read(reg_addr, len(write_data), BMA400_CS_PIN)
        data = ' '.join(format(x, '04x') for x in reg_data)
        print("read data: 0x" + data)
        write_data[0] = write_data[0] + 1

    BOARD.SetVDD(0)
    BOARD.SetVDDIO(0)
    BOARD.ClosePCInterface()
