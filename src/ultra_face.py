"""This module is responsible for the implementation of the UltraFace class, which is responsible for the face detection using the UltraFace model."""
import cv2
import tflite_runtime.interpreter as tflite
import numpy as np
import time
import pathlib
import os

BOX_COLOR = (255, 128, 0)

class UltraFace():
    """ Manages the face detection using the UltraFace model."""
    def __init__(self, model) -> None:
        """
        Initializes the UltraFace object with the given model.
        
        Args:
            model: the model to use for the face detection.
        """
        model_dir = pathlib.Path(os.path.realpath(__file__)).parents[1] / 'models'
        self.model_name = model
        if os.path.exists('/usr/lib/libvx_delegate.so'):
            self.interpreter = tflite.Interpreter(model_path=rf"{model_dir}/{self.model_name}.tflite",
                                                  experimental_delegates=[tflite.load_delegate('/usr/lib/libvx_delegate.so')])
        else:
            self.interpreter = tflite.Interpreter(model_path=rf"{model_dir}/{self.model_name}.tflite")
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.input_scale, self.input_zero_point = self.input_details[0]["quantization"]
        self.detection_threshold = 0.8
        
        self.image = None
        self.image_input = None
        self.boxes = None
        self.scores = None
        self.boxes_coordinates = []
         
    def _data_preprocessing(self):
        """Preprocesses the input data for the model."""
        image_rgb = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        image_resize = cv2.resize(image_rgb, (320, 240))[..., ::-1]
        image_expanded = np.expand_dims(image_resize, axis=0)
        image_norm = (image_expanded - 127.0)/128.0
        self.image_input = (image_norm / self.input_scale + self.input_zero_point).astype(np.uint8)
        
    def _data_postprocessing(self):
        """Postprocesses the output data from the model."""
        conf_mask = self.detection_threshold < self.scores
        self.boxes, self.scores = self.boxes[conf_mask], self.scores[conf_mask]
        self.boxes *= np.tile(self.image.shape[1::-1], 2)
        self.boxes = self.boxes.astype(np.int32)
        
        for i in range(self.boxes.shape[0]):
            
            self.boxes_coordinates.append([self.boxes[i, :]])
            
    def _reset_result(self):
        """Resets the result attributes."""
        self.image = None
        self.image_input = None
        self.boxes = None
        self.scores = None   
        self.boxes_coordinates = []
        
    def detect(self, image):
        """
        Runs the model inference on the given image and returns the face detection result.
        
        Args: 
            image: the image to run the inference on.
        
        Returns:
            the face detection result as a list of boxes coordinates.
        """
        self._reset_result()
        self.image = image.copy()
        self._data_preprocessing()

        self.interpreter.set_tensor(self.input_details[0]['index'], self.image_input)
        self.interpreter.invoke()
        out = self.interpreter.get_tensor(self.output_details[0]['index']).astype(np.float32)

        self.boxes = out[..., 2:]
        self.scores = out[..., 1]
        self._data_postprocessing()
        
        return self.boxes_coordinates