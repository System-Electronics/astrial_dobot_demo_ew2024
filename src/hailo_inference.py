"""This module is responsible for the face detection. It uses the UltraFace model to detect the face in the frame."""
from hailo_platform import (HEF, VDevice, HailoStreamInterface, InferVStreams, ConfigureParams,
                InputVStreamParams, OutputVStreamParams, FormatType)

import numpy as np
import logging
import cv2 as cv

class HailoInference():
    def __init__(self, hef_path):
        """
        Initialize the HailoInference class with the provided HEF model file path.

        Args:
            hef_path (str): Path to the HEF model file.
        """
        self.hef = HEF(hef_path)
        self.target = VDevice()
        self.network_group = self._configure_and_get_network_group(self.hef, self.target)
        self.network_group_params = self.network_group.create_params()
        self.input_vstreams_params, self.output_vstreams_params = self._create_vstream_params(self.network_group)
        self.input_vstream_info, self.output_vstream_info = self._get_and_print_vstream_info(self.hef)

    def _configure_and_get_network_group(self, hef, target):
        """
        Configure the Hailo device and get the network group.

        Args:
            hef (HEF): HEF model object.
            target (VDevice): Hailo device target.

        Returns:
            NetworkGroup: Configured network group.
        """
        configure_params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
        network_group = target.configure(hef, configure_params)[0]
        return network_group
    
    def _create_vstream_params(self, network_group):
        """
        Create input and output stream parameters.

        Args:
            network_group (NetworkGroup): Configured network group.

        Returns:
            InputVStreamParams, OutputVStreamParams: Input and output stream parameters.
        """
        input_vstreams_params = InputVStreamParams.make_from_network_group(network_group, format_type=FormatType.UINT8)
        output_vstreams_params = OutputVStreamParams.make_from_network_group(network_group, format_type=FormatType.FLOAT32)
        return input_vstreams_params, output_vstreams_params
    
    def _get_and_print_vstream_info(self, hef):
        """
        Get and print information about input and output stream layers.

        Args:
            hef (HEF): HEF model object.

        Returns:
            list, list: List of input stream layer information, List of output stream layer information.
        """
        input_vstream_info = hef.get_input_vstream_infos()
        output_vstream_info = hef.get_output_vstream_infos()

        for layer_info in input_vstream_info:
            logging.info('Input layer: {} {}'.format(layer_info.name, layer_info.shape))
        for layer_info in output_vstream_info:
            logging.info('Output layer: {} {}'.format(layer_info.name, layer_info.shape))
        
        return input_vstream_info, output_vstream_info

    def data_preprocessing(self, image):
        """ 
        Transforms the input image accordingly to the model expected input format.
        
        Args: 
            image: the image to preprocess.
        
        Returns: 
            image_input: the preprocessed image.
        """
        image_rgb = cv.cvtColor(image, cv.COLOR_BGR2RGB)
        image_resize = cv.resize(image_rgb, (224, 224), interpolation=cv.INTER_CUBIC)
        image_input = np.expand_dims(image_resize, axis=0).astype(np.uint8)
        return image_input

    def data_postprocessing(self, image, output):
        """ 
        Retrives the coordinates of each finger out of the model output.
        
        Args: 
            image: the image to postprocess.
            
        Returns: 
            A numpy array containing coordinates for each finger.
        """
        coordinates = []
        for data in output.reshape(21,3):
            x,y,z = data
            x = x*image.shape[1]/224
            y = y*image.shape[0]/224
            coordinates.append((x,y,z))
        return np.array(coordinates)
    
    def get_input_shape(self):
        """
        Get the shape of the model's input layer.

        Returns:
            tuple: Shape of the model's input layer.
        """
        return self.hef.get_input_vstream_infos()[0].shape # Assumes that the model has one input

    def get_output_shape(self):
        """
        Get the shape of the model's output layer.

        Returns:
            tuple: Shape of the model's output layer.
        """
        return self.hef.get_output_vstream_infos()[0].shape # Assumes that the model has one output
    
    def run(self, image, output_vstream_info_indexes=[0]):
        """
        Run inference on Hailo-8 device.

        Args:
            image (numpy.ndarray): Image to run inference on.

        Returns:
            numpy.ndarray: Inference output.
        """
        output_l = list(list())
        with InferVStreams(self.network_group, self.input_vstreams_params, self.output_vstreams_params) as infer_pipeline:            
            input_data = {self.input_vstream_info[0].name: image}   # Assumes that the model has one input

            with self.network_group.activate(self.network_group_params):
                output = infer_pipeline.infer(input_data)
                for output_vstream_info_index in output_vstream_info_indexes:
                    output_tmp = output[self.output_vstream_info[output_vstream_info_index].name]
                    output_l.append(output_tmp)

        return output_l

    def run_all(self, image):
        """
        Run inference on Hailo-8 device.

        Args:
            image (numpy.ndarray): Image to run inference on.

        Returns:
            numpy.ndarray: Inference output.
        """
        output = None
        with InferVStreams(self.network_group, self.input_vstreams_params, self.output_vstreams_params) as infer_pipeline:
            input_data = {self.input_vstream_info[0].name: image}   # Assumes that the model has one input

            with self.network_group.activate(self.network_group_params):
                output = infer_pipeline.infer(input_data)


        return output


    def release_device(self):
        """
        Release the Hailo device.
        """
        self.target.release()
        