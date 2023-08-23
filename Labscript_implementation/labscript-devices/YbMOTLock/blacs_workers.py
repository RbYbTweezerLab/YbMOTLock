# Editted from FunctionRunner by Oliver Tu, 06/2023                 #
#####################################################################
#                                                                   #
# /labscript_devices/YbMOTLock/blacs_workers.py                       #
#                                                                   #
# Copyright 2019, Monash University and contributors                #
#                                                                   #
# This file is part of labscript_devices, in the labscript suite    #
# (see http://labscriptsuite.org), and is licensed under the        #
# Simplified BSD License. See the license.txt file in the root of   #
# the project for the full license.                                 #
#                                                                   #
#####################################################################
import os
import numpy as np
import labscript_utils.h5_lock
# import h5py
from blacs.tab_base_classes import Worker
from pyBusPirateLite.I2C import *
import struct as st

class YbMOTLockWorker(Worker):

    def init(self):
        self.i2c = I2C(self.comport, 115200)
        self.i2c.resetBP()
        if not self.i2c.BBmode():
            raise TypeError("i2c.BBmode() Failed.")

        if not self.i2c.enter_I2C():
            raise TypeError("i2c.enter_I2C() Failed.")
            
        if not self.i2c.cfg_pins(I2CPins.POWER | I2CPins.PULLUPS):
            raise TypeError("Failed to set I2C peripherals.")
        if not self.i2c.set_speed(I2CSpeed._100KHZ):
            raise TypeError("Failed to set I2C Speed.")
        self.i2c.port.flushInput();
        self.i2c.port.flushOutput();



    def i2c_write_data(self,data):
        self.i2c.send_start_bit()
        print(self.i2c.bulk_trans(len(data),data))
        print("Array sent: ",data)
        if not self.i2c.send_stop_bit():
            raise TypeError("The I2C transfer is broken: Reconnect the Bus Pirate and refresh the Blcas device.")

    def i2c_send(self,startFreq, endFreq, stepnum):
        data_array = [int(self.addr,16)<<1]
        jump_freq = (endFreq - startFreq) / stepnum
        sf_arr = bytearray(st.pack("f", startFreq))
        jf_arr = bytearray(st.pack("f", jump_freq))
        ss_arr = bytearray(st.pack("H", stepnum))
        data_array += [ int("0x%02x" % b,16) for b in sf_arr ]
        data_array += [ int("0x%02x" % b,16) for b in jf_arr ]
        data_array += [ int("0x%02x" % b,16) for b in ss_arr ]
        data_array += [0x11]
        self.i2c_write_data(data_array)
        # print(data_array)

    def program_manual(self, values):
        return {}

    def transition_to_buffered(self, device_name, h5_file, initial_values, fresh):
        return {}

    def transition_to_manual(self):
        return True

    def shutdown(self):
        self.i2c.resetBP()
        return

    def abort_buffered(self):
        return self.transition_to_manual()

    def abort_transition_to_buffered(self):
        return True
