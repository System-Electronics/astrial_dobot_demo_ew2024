""" This module contains generic functions used by the other modules."""
import yaml 
import os
from typing import Literal
from argparse import ArgumentParser
import logging
import numpy as np
import serial.tools.list_ports

config = None
def parse_arguments():
    """ 
    Parses the command line arguments.
    
    Returns: 
        the parsed arguments.
    """
    parser = ArgumentParser()
    parser.add_argument('--video-device', default=get_config_param('device_configuration','video_device'), help='Device used as video input')
    parser.add_argument('--relay-device', default=get_config_param('device_configuration','relay_device'), help='Device used as relay output')
    parser.add_argument('--robotic-arm-device', default=get_config_param('device_configuration','robotic_arm_device'), help='Device used as robotic arm output')
    args = parser.parse_args()
    return args


def set_up_logging():
    """ Sets up the logging configuration."""
    logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    )
    logger = logging.getLogger()
    return logger

def init_config_param():
    """ Reads the configuration file and stores it in a global variable."""
    global config
    working_dir = os.path.dirname(os.path.realpath(__file__))
    config_file_path = os.path.join(working_dir, r'config.yaml')
    with open(config_file_path, 'r') as file_handler:
        config = yaml.safe_load(file_handler)
        
def get_config_param(
        param_section: str,
        param_key: str) -> str:
    """ 
    Retrieves the specified param in the configuration file
    
    Args:
        param_section: the section of the configuration file.
        param_key: the key of the param in the specified section.
        
    Returns:
        the value of the specified param.
    """
    if config is None:
        return None
    return config[param_section][param_key]

def set_config_param(
    param_section: str,
    param_key: str,
    param: str,
) -> None:
    """ 
    Sets the specified param in the configuration file
    
    Args:
        param_section: the section of the configuration file.
        param_key: the key of the param in the specified section.
        param: the value to set.
    """
    working_dir = os.path.dirname(os.path.realpath(__file__))
    config_file_path = os.path.join(working_dir, r'config.yaml')
    with open(config_file_path, 'r') as file_handler:
        config = yaml.safe_load(file_handler)

    config[param_section][param_key] = param
    with open(config_file_path, 'w') as file_handler:
        file_handler.write(yaml.dump(config, sort_keys=False))
        
def list_serial_connections():
    """ Lists the serial connections available."""
    ports = serial.tools.list_ports.comports()
    for port, desc, hwid in sorted(ports):
        print("{}: {} [{}]".format(port, desc, hwid))
        
def get_image_crop_coordinates(crop_type: Literal['hand_detetction','gui']):
    """
    Retrieves the coordinates to crop the image for the specified crop type. 
    
    Args:
        crop_type: the type of the crop.
    
    Returns:
        a couple of arrays each containing the coordinates to crop the image.
        The first array contains the coordinates to crop the left side of the image, 
        the second array contains the coordinates to crop the right side of the image.
    """
    crop_square_factor = get_config_param('image_configuration',f'crop_square_factor_{crop_type}')
    w = get_config_param('image_configuration','camera_resolution_w')
    h = get_config_param('image_configuration','camera_resolution_h')
        
    if min(w// 2, h) == w // 2:
        w_final = int(w // 2 * crop_square_factor)
        
        w_min_l = w // 4 - w_final // 2
        w_max_l = w_min_l + w_final
        
        w_min_r = w *3// 4  - w_final // 2
        w_max_r = w_min_r + w_final
         
        h_cutoff_min = h//2 - w_final//2
        h_cutoff_max = h_cutoff_min + w_final
        
        return np.array([h_cutoff_min,h_cutoff_max , w_min_l,w_max_l]),\
            np.array([h_cutoff_min,h_cutoff_max, w_min_r,w_max_r])
    
    else:
        h_final = int(h * crop_square_factor)
        h_min = h // 2 - h_final // 2
        h_max = h_min + h_final
        w_cutoff_min = w // 4 - h_final
        w_cutoff_max = w_cutoff_min + h_max
        return np.array([h_min,h_max , w_cutoff_min,w_cutoff_max]),\
               np.array([h_min,h_max, w_cutoff_min + w//2,w_cutoff_max + w//2])