"""This module is responsible for controlling the relay."""

import serial 
from common import get_config_param

class Relay:
    """Manages the control of the relay."""
    def __init__(self, device:str) -> None:
        """
        Initializes the relay object as a serial device.
        
        Args: 
            device: The device to control.
        """
        self.device = device
        self.serial = serial.Serial(self.device, baudrate=get_config_param('device_configuration','relay_baudrate'))
        
    def health_check(self)-> bool:
        """Checks if the relay is healthy."""
        return self.serial.is_open
    
    def set(self, on:bool = True):
        """ 
        Sets the relay on or off.
        
        Args: 
            on: The state to set the relay.
        """
        with self.serial:
            if on:
                value= bytes.fromhex('65')
            else:
                value= bytes.fromhex('6f')
            self.serial.write(value)
            
    def close(self):
        """ Releases the serial port."""
        self.serial.close()