from labscript import *

from labscript_devices.PulseBlasterUSB import PulseBlasterUSB
from labscript_devices.Eurocard_Synth.labscript_devices import *

# For dummy pseudoclock
# from labscript_devices.DummyPseudoclock.labscript_devices import DummyPseudoclock
# from labscript_devices.DummyIntermediateDevice import DummyIntermediateDevice

#############################################################
#   CONNECTION TABLE
#############################################################

# Use a virtual, or 'dummy', device for the psuedoclock
# DummyPseudoclock(name='pulseblaster_0')

#############################################################
#   PULSEBLASTER
#############################################################

PulseBlasterUSB(name='pulseblaster_0', board_number=0, time_based_stop_workaround=True, time_based_stop_workaround_extra_time=0.5)

YbMOTLock(name='YbMOTLock',COM_port='COM5',I2c_address='0x11')

if __name__ == '__main__':
    start()
    stop(1)

