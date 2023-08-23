import numpy as np
from labscript import Device,set_passed_properties
from labscript_utils import dedent
import labscript_utils.h5_lock, h5py

class YbMOTLock(Device):

    @set_passed_properties(
        property_names={
            'connection_table_properties': [
                'COM_port',
                'I2c_address',
            ]
        }
    )
    def __init__(self, name,COM_port,I2c_address,**kwargs):
        Device.__init__(self, name=name, parent_device=None, connection=None, **kwargs)
        self.BLACS_connection = name

    def generate_code(self, hdf5_file):
        group = self.init_device_group(hdf5_file)
