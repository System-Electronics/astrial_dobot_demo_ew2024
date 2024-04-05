""" This module is responsible for controlling the robot via hand gestures."""
import threading
import cv2 as cv
import pathlib
import os
import numpy as np

from hailo_inference import HailoInference
from gesture_recognition import GestureRecognition, Fingers, Gesture
from relay import Relay
from SEdobot import SEDobot
from common import get_image_crop_coordinates

class Controller:
    """ Stores the command and provides a way to set and get it."""
    def __init__(self):
        """ Initializes the command and the lock."""
        self.lock = threading.Lock()
        self.command = None
        self.command_is_set = threading.Condition()
        
    def set_command(self, command:dict):
        """
        Safe set of the command.
        
        Args: 
            command (dict): The command to be set.
        """
        with self.lock:
            self.command = command
        with self.command_is_set:
            self.command_is_set.notify_all()
            
    def get_command(self):
        """ Safe read of the command."""
        command = None
        with self.command_is_set:
            ret = self.command_is_set.wait(timeout=0.5)
            if ret:
                with self.lock:
                    command = self.command
                    self.command = None

        return command 
    
class HandController(Controller):
    """ Manages the control of the robot via hand gestures by running the hand landmark model inference and the robot control in separate threads."""
    def __init__(self, state_changed, fsm, camera, robot_device, relay_device, model ='hand_landmark_lite'):
        """ 
        Initializes the hand controller and the threads for the hand landmark model inference and the robot control.
        
        Args: 
            state_changed (threading.Condition): The condition variable to notify the state change.
            fsm (Fsm): The finite state machine.
            camera (CameraStreamer): The camera to get the frames from.
            robot_device (str): The robot device to control.
            relay_device (str): The relay device to control.
            model (str): The hand landmark model to use.
        
        """
        super().__init__()
        self.model_name = model
        
        self.fsm = fsm
        self.relay = Relay(relay_device)
        self.camera = camera
        self.robot = SEDobot(robot_device)
        self.state_changed = state_changed

        if not self.relay.health_check():        
            raise ValueError('ERROR: relay not connected') 
        
        if not self.robot.health_check():
            raise ValueError('ERROR: robot not connected')
        
        self.current_commands = {'j1': None, 'j3': None, 'grip': None}
        self.stop_command = [{'command': 'stop', 'movement': 'stop'}]
        self.close_cmd = False
        self.close_hl = False
        self.fingers_r = Fingers()
        self.fingers_l = Fingers()
        self.gesture_r = Gesture()
        self.gesture_l = Gesture()
        self.hl = threading.Thread(target=self._hand_landmark, args=(self.fingers_r, self.fingers_l,self.gesture_r, self.gesture_l,))
        self.cmd = threading.Thread(target=self._command, args=())
        self.hand_detected_r = threading.Event()
        self.hand_detected_l = threading.Event()
        self.hl.start()
        self.cmd.start()
        
    def _hand_landmark(self,fingers_r, fingers_l, gesture_r, gesture_l):
        """
        Runs the hand landmark model inference and the gesture recognition in a loop.
        
        Args:
            fingers_r (Fingers): The right hand fingers.
            fingers_l (Fingers): The left hand fingers.
            gesture_r (Gesture): The right hand gesture.
            gesture_l (Gesture): The left hand gesture.
        """
        model_dir = pathlib.Path(os.path.realpath(__file__)).parents[1] / 'models'
        hlm = HailoInference(hef_path=rf"{model_dir}/{self.model_name}.hef")

        cmdetector_r = GestureRecognition(fingers_r, gesture_r, blocked_commands= ['grip_close', 'grip_open'])
        cmdetector_l = GestureRecognition(fingers_l, gesture_l, blocked_commands= ['j1', 'j3'])

        while not self.close_hl: 
            with self.state_changed:
                ret = self.state_changed.wait(timeout = 0.5)
            if not ret:
                continue
            while self.fsm.current_state() == 'command': 
                image = self.camera.get_frame()
                if image is None:
                    continue
                # Cut the image in half and squarify it
                image_r_coordinates, image_l_coordinates = get_image_crop_coordinates(crop_type='hand_detection')
                image_r, image_l = image[image_r_coordinates[0]:image_r_coordinates[1], image_r_coordinates[2]:image_r_coordinates[3]], \
                                   image[image_l_coordinates[0]:image_l_coordinates[1], image_l_coordinates[2]:image_l_coordinates[3]]

                #CMD LEFT HAND
                coordinates_raw, hand_detected_score = hlm.run(hlm.data_preprocessing(image_l), [0,2])
                if hand_detected_score > 0.8:
                    self.hand_detected_l.set()
                    coordinates = hlm.data_postprocessing(image_l, coordinates_raw)
                    command = cmdetector_l.get_command(coordinates=coordinates)
                    self.set_command(command)
                    
                # CMD RIGHT HAND
                coordinates_raw, hand_detected_score= hlm.run(hlm.data_preprocessing(image_r), [0,2])
                if hand_detected_score > 0.8:
                    self.hand_detected_r.set()
                    coordinates = hlm.data_postprocessing(image_r, coordinates_raw)
                    command = cmdetector_r.get_command(coordinates=coordinates)
                    self.set_command(command)
                else: self.set_command(self.stop_command)

    def _command(self):    
        """ By checking the current state of the fsm, it either set the robot in idle mode or in control mode and sends commands to be executed."""
        self.relay.set(on=False)
        self.robot.go_to_idle()
        from_command = False
        while not self.close_cmd: 
            if not from_command:
                with self.state_changed:
                    ret = self.state_changed.wait(timeout=0.5)
                if not ret:
                    continue
            if self.fsm.current_state() == 'idle':
                self.relay.set(on=False)
                self.robot.go_to_idle()
                from_command = False
            if self.fsm.current_state() == 'command':
                self.relay.set(on=True)
                self.robot.go_to_command()
                while self.fsm.current_state() == 'command' and not self.close_cmd:
                    from_command = True
                    commands = self.get_command()
                    if commands != None:
                        if len(commands) == 1 :
                            if commands[0]['command'] == 'stop':
                                self.robot.execute_stop_command() 
                                continue
                            if commands[0]['command'] == None:
                                continue
                        for command in commands:
                            if command['movement'] == None:
                                continue
                            if self.current_commands[command['command']] == command['movement']:
                                continue
                            self.current_commands[command['command']] = command['movement']
                            self.robot.execute_command(command)
                                
        self.relay.set(on=False)
        self.robot.go_to_idle()
        
    def close(self): 
        """ Stops the threads for hand landmark inference and robot control, releases resources."""
        self.close_cmd = True
        self.close_hl = True
        self.hl.join()
        self.cmd.join()
        self.relay.close()
        
