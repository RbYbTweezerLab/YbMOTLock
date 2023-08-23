# Editted from FunctionRunner by Oliver Tu, 06/2023                 #
#####################################################################
#                                                                   #
# /labscript_devices/YbMOTLock/blacs_tabs.py                       #
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

from qtutils.qt.QtCore import *
from qtutils.qt.QtGui import *
from qtutils.qt.QtWidgets import QPushButton

from blacs.tab_base_classes import Worker, define_state
from blacs.tab_base_classes import MODE_MANUAL

from blacs.device_base_class import DeviceTab

from qtutils import UiLoader

class YbMOTLockTab(DeviceTab):

    def initialise_GUI(self):
        connection_object = self.settings['connection_table'].find_by_name(self.device_name)
        connection_table_properties = connection_object.properties
        self.comport = connection_table_properties.get('COM_port', None)
        self.addr = connection_table_properties.get('I2c_address', None)
        layout = self.get_tab_layout()
        ui_filepath = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'YbMOTLockGUI.ui')
        self.ui = UiLoader().load(ui_filepath)
        layout.addWidget(self.ui)

        self.ui.send_pushButton.released.connect(self.valueSend)


    def initialise_workers(self):
        self.create_worker(
            'main_worker',
            'labscript_devices.YbMOTLock.blacs_workers.YbMOTLockWorker',
            {'comport':self.comport,
             'addr':self.addr},
        )
        self.primary_worker = 'main_worker'

    @define_state(MODE_MANUAL, queue_state_indefinitely=True, delete_stale_states=True)
    def valueSend(self):
        try:
            startFreq = float(self.ui.start_lineEdit.text())
        except:
            self.ui.printer.setText("Error: Start Frequency is not a float.")
            self.ui.printer.setStyleSheet("color: red")
            return
        try:
            endFreq = float(self.ui.end_lineEdit.text())
        except:
            self.ui.printer.setText("Error: End Frequency is not a float.")
            self.ui.printer.setStyleSheet("color: red")
            return
        try:
            stepnum = int(self.ui.step_lineEdit.text())
            if stepnum < 1:
                raise ValueError
        except:
            self.ui.printer.setText("Error: Ramp Step Number is not a positive integer.")
            self.ui.printer.setStyleSheet("color: red")
            return
        if stepnum > 65535:
            self.ui.printer.setText("Error: Ramp Step Number should be smaller than 65536.")
            self.ui.printer.setStyleSheet("color: red")
            return            

        self.ui.printer.setText("Array sent")
        self.ui.printer.setStyleSheet("color: black")
        yield(self.queue_work(self.primary_worker, 'i2c_send',startFreq, endFreq, stepnum))
