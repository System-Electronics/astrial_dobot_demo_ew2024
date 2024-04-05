""" This module contains the GestureRecognition class which is used to recognize the hand gestures and return the corresponding command type and movement type."""
from typing import Literal, Union
import numpy as np
import math
import threading
from copy import deepcopy
from hand_landmarker import HandLandmark
from common import get_image_crop_coordinates

class Fingers():
    """ Stores the fingers positions and landmarks."""
    def __init__(self) -> None:
        """ Initializes the fingers dictionary and the finger landmarks dictionary."""
        # Initialize the fingers dictionary of tips finger positions
        self.fingers = {            
                        'thumb_position':  None,
                        'index_position':  None,
                        'middle_position': None,
                        'ring_position':   None,
                        'pinky_position':  None, 
                        'wrist_position':  None,
        }
        # Initialize the finger landmarks dictionary with both the tip and pip landmarks
        self.finger_landmarks = {
            'thumb': [HandLandmark.THUMB_TIP.value,HandLandmark.THUMB_IP.value],
            'index': [HandLandmark.INDEX_FINGER_TIP.value,HandLandmark.INDEX_FINGER_PIP.value],
            'middle': [HandLandmark.MIDDLE_FINGER_TIP.value, HandLandmark.MIDDLE_FINGER_PIP.value],
            'ring': [HandLandmark.RING_FINGER_TIP.value,HandLandmark.RING_FINGER_PIP.value],
            'pinky': [HandLandmark.PINKY_TIP.value, HandLandmark.PINKY_PIP.value],
            'wrist': [HandLandmark.WRIST.value]
        }

        self.lock = threading.Lock()

    def set_positions(self, coordinates:np.array):
        """ 
        Safe write of the fingers positions in the fingers dictionary.
        
        Args:
            coordinates: the hand landmarks coordinates.
        """
        with self.lock:
            for key, value in self.finger_landmarks.items():
                self.fingers[f'{key}_position'] = coordinates[value[0]]
            
    def get_positions(self):
        """ 
        Safe read of the fingers positions in the fingers dictionary.
        
        Returns:
            the fingers positions.
        """
        with self.lock:
            fingers = deepcopy(self.fingers)
            if fingers['thumb_position'] is None:
                return None
        return fingers 
    
    
class Gesture():
    """ Stores the detected gestures."""
    def __init__(self) -> None:
        """ Initializes the angle movement ranges for the j1 and j3 joints."""
        self.range_movement_j1 ={'min':115, 'max': 175} 
        self.range_percentage_movement_j1 = {'min':20, 'max': 80} 
        frame_r_coordinates, _ = get_image_crop_coordinates(crop_type='gui')
        self.range_movement_j3_gui ={'min':frame_r_coordinates[0], 'max': frame_r_coordinates[1]}
        frame_r_coordinates, _ = get_image_crop_coordinates(crop_type='hand_detection')
        self.range_movement_j3_hd ={'min':frame_r_coordinates[0], 'max': frame_r_coordinates[1]}
        
        self.movement_j1 = None
        self.movement_j3 = None
        
        self.lock = threading.Lock()
        
    def set_gesture(self, movement_type: str, movement: float):
        """ 
        Safe write of the gesture movement in the gesture dictionary.
        
        Args:
            movement_type: the movement type to store.
            movement: the movement value to store.
        """
        with self.lock:
            if movement_type == 'j1':
                self.movement_j1 = movement
            elif movement_type == 'j3':
                self.movement_j3 = movement
            
    def get_gesture(self, movement_type: str):
        """
        Safe read of the gesture movement in the gesture dictionary.
        
        Args:
            movement_type: the movement type to retrieve.
            
        Returns:
            the movement value.
        """
        with self.lock:
            movement = self.movement_j1 if movement_type == 'j1' else self.movement_j3
        return movement 
        
class GestureRecognition():
    """ Recognizes the hand gestures given the fingers positions."""
    def __init__(self, fingers: type(Fingers), gesture: type(Gesture), blocked_commands: list = []) -> None:
        """ 
        Initializes the fingers, gesture and blocked commands list.
        
        Args:
            fingers: the fingers object to store the fingers positions.
            gesture: the gesture object to store the gesture movements.
            blocked_commands: the list of blocked commands.
        """
        self.fingers = fingers
        self.gesture = gesture
        # Initialize the blocked commands list
        self.blocked_commands = blocked_commands
    
    def _collect_data(self, coordinates:np.array):
        """
        Collects the data from the hand landmarks and store it in the fingers dictionary.
        
        Args:
            coordinates: the hand landmarks coordinates.
        """
        self.fingers.set_positions(coordinates)
            
    def _get_distance(self, a, b) -> float:
        """
        Computes the distance between two points.
        
        Args:
            a: the first point.
            b: the second point.
        
        Returns:
            the distance between the two points.
        """
        return np.linalg.norm(a-b)  
    
    def _translate_to_origin(self, finger_pos, origin) -> np.array:
        """
        Translates the finger position to the origin. 
        
        Args:
            finger_pos: the finger position.
            origin: the origin position.
        
        Returns:
            the translated finger position.
        """
        return np.array([finger_pos - origin])[:,:-1].reshape(2)
        
    def _get_inner_angle(self, vector_a ,vector_b) -> float:
        """
        Computes the inner angle from vector_a to vector_b.
        
        Args:
            vector_a: the first vector.
            vector_b: the second vector.
        
        Returns:
            the inner angle between the two vectors.
        """
        return math.degrees(np.arccos((np.dot(vector_a,vector_b)/(np.linalg.norm(vector_a)*np.linalg.norm(vector_b)))))
        
    def _compute_determinant(self,vector_a,vector_b):
        """
        Computes the determinant from vector_a to vector_b.
        
        Args:
            vector_a: the first vector.
            vector_b: the second vector.
        """
        self.determinant = vector_a[0]*vector_b[1]-vector_a[1]*vector_b[0]
        
    def _is_close(self, a, b, rtol=1e-02, atol=0.0) -> bool:
        """ 
        Checks if two values are close.
        
        Args:
            a: the first value.
            b: the second value.
            rtol: the relative tolerance parameter.
            atol: the absolute tolerance parameter.
    
        Returns:
            True if the two values are close, False otherwise.
        """
        return np.allclose(a, b, rtol, atol)
    
    def _get_centroid(self, vectors) -> Union[np.array,None]:  
        """ 
        Computes the centroid of a set of vectors, as the mean of the vectors.
        
        Args:
            vectors: the set of vectors.
            
        Returns:
            the centroid of the set of vectors.
        """
        vectors = np.array(vectors)
        if vectors.ndim == 3:
            return np.mean(vectors, axis=(0,1))
        if vectors.ndim == 2:
            return np.mean(vectors, axis=0)
    
    def _get_j3_movement(self, coordinates) -> float:
        """
        Collects the data from the hand landmarks, store it in the fingers dictionary and compute the j3 movement type
        as position of the middle finger.
        
        Args:
            coordinates: the hand landmarks coordinates.
        
        Returns:
            the j3 movement value as a percentage.
        """
        
        self._collect_data(coordinates)
 
        mw_dist = self.fingers.fingers['wrist_position'][1]-self.fingers.fingers['middle_position'][1]
        middle_h = self.fingers.fingers['middle_position'][1] + self.gesture.range_movement_j3_hd['min']
        range_movement_j3_max = self.gesture.range_movement_j3_gui['max']- mw_dist
        ranges = np.linspace(self.gesture.range_movement_j3_gui['min'], range_movement_j3_max, 10)
        
        movement = None
        if self._is_close(middle_h,self.gesture.range_movement_j3_gui['min'], 5e-02):
            movement =  100.
        elif self._is_close(middle_h,range_movement_j3_max, 5e-02):
            movement =  0.
        else:
            for i in range(len(ranges)-1):
                if int(middle_h) in range(int(ranges[i]), int(ranges[i+1])):
                    movement = 100 -(((((ranges[i+1]-ranges[i])/2)+ranges[i] - self.gesture.range_movement_j3_gui['min'])*100))/(range_movement_j3_max-self.gesture.range_movement_j3_gui['min'])
                    movement = round(movement,3)
                    break
        self.gesture.set_gesture('j3', movement)
        return movement


    def _get_j1_movement(self, coordinates) -> float:
        """
        Collects the data from the hand landmarks, store it in the fingers dictionary and compute the j1 movement type as the inner angle between 
        the thumb and the wrist.
        
        Args:
            coordinates: the hand landmarks coordinates.
        
        Returns: 
            the j1 movement value as a percentage.
        """
        
        self._collect_data(coordinates)
        thumb_o = self._translate_to_origin(self.fingers.fingers['middle_position'],self.fingers.fingers['wrist_position'])
        wrist_o = np.array([self.fingers.fingers['wrist_position']])[:,:-1].reshape(2)

        inner_angle = self._get_inner_angle(thumb_o, wrist_o)

        movement_real = max(0.,min(100.,self.gesture.range_movement_j1['max'], (inner_angle - self.gesture.range_movement_j1['min'] )*100/(self.gesture.range_movement_j1['max']-self.gesture.range_movement_j1['min'])))
        movement = 0. if movement_real <= self.gesture.range_percentage_movement_j1['min'] else 100. if movement_real >= self.gesture.range_percentage_movement_j1['max'] else 'stop'
        self.gesture.set_gesture('j1', movement_real)
        return movement

    def _retrive_command_type(self, coordinates:np.array) -> str:
        """ 
        Retrieves the command type given the hand landmarks coordinates.
        The command type is determined by the grip type or the j1 and j3 movement type
        grip_close: all the fingers tips are closer to the wrist than the finger pip except for the thumb
        grip_open: all the fingers tips are further from the wrist than the finger pip
        j3, j1 as grip_open
        
        Args:
            coordinates: the hand landmarks coordinates.
        
        Returns:
            the command type.
        """

        finger_distances = {finger: [self._get_distance(coordinates[lm], self.fingers.fingers['wrist_position']) for lm in landmark] for
                            finger, landmark in self.fingers.finger_landmarks.items() if finger != 'wrist'}

        
        if 'grip_close' not in self.blocked_commands and \
            all(distances[0] < distances[1] for finger, distances in finger_distances.items() if finger != 'wrist' and finger != 'thumb'):
            return 'grip_close'
        if 'grip_open' not in self.blocked_commands and \
            all(distances[0] > distances[1] for finger, distances in finger_distances.items() if finger != 'wrist'):
            return 'grip_open'
        if 'j3' not in self.blocked_commands and 'j1' not in self.blocked_commands and\
            all(distances[0] > distances[1] for finger, distances in finger_distances.items() if finger != 'wrist'):
            return 'j3, j1'
    
    def get_command(self,coordinates:np.array) -> list:
        """
        Builds the command as a list of dictionaries with the command type and movement type.
        
        Args:
            coordinates: the hand landmarks coordinates.
            
        Returns:
            the command as a list of dictionaries.
        """
        command_actions = {
            'grip_close': [{'command': 'grip', 'movement': 'close'}],
            'grip_open': [{'command': 'grip', 'movement': 'open'}],
            'j3, j1': [
                {'command': 'j3', 'movement': self._get_j3_movement(coordinates)},
                {'command': 'j1', 'movement': self._get_j1_movement(coordinates)}
            ]
        }

        command_type = self._retrive_command_type(coordinates)
        command = command_actions.get(command_type, [{'command': None,
                                                          'movement': None}]) if command_type not in self.blocked_commands else [{
            'command': None, 'movement': None}]                                        
        return command
            