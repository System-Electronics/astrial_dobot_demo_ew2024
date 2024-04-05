""" This module is responsible for the implementation of the SEDobot class, which is a subclass of the Dobot class."""

from typing import Literal
from pydobot.dobot import Dobot
import numpy as np 
import math
import time

class SEDobot(Dobot):
    """ Manages the communication with the Dobot Magician robot."""
    def __init__(self, port):
        """
        Initializes the SEDobot object. It sets the initial position of the robot,
        the range of movement for the joints and the gripper, and the parameters for the jog and point to point commands.
        It also sets the initial values for the current theta angles and the last joint commands.
        
        Idle position and command position are set with custom positions.
        
        Args: 
            port: the port to communicate with the robot.
        """
        super().__init__(port)
        self.current_theta_z = 0
        self.current_theta_y = 0
        self.last_j1_cmd = (266.4521789550781, -34.411170959472656, 50.0, 0.0) #init with command position
        self.last_j3_cmd = (266.4521789550781, -34.411170959472656, 50.0, 0.0)
        self.theta_z_range = {'min': -80., 'max': 80.}
        self.z_range = {'min': -15., 'max': 100.}
        self.idle_pos = (166.85791015625, -6.324118137359619, 18.088035583496094, 0.0)
        self.command_pos = (266.4521789550781, -34.411170959472656, 50.0, 0.0)

        self.interface.set_jog_joint_params([70, 70, 70, 70], [500, 500, 500, 500])
        self.interface.set_point_to_point_joint_params([70, 70, 70, 70], [500, 500, 500, 500])
        
        
    def health_check(self):
        """
         Verifies if the robot is connected or not.

         Returns:
             a boolean value, True if connected else False
        """
        return self.connected()
    
    def _go_to_position(self, position:np.array):
        """
        Moves the robot to the given position.
        
        Args: 
            position: the position to move the robot to.
        """
        pose = self.get_pose()
        if (np.array(pose[:4])!=position).any():
            self.interface.set_point_to_point_command(1, position[0], position[1], position[2], position[3])
            
    def _get_inner_angle_origin(self, vector_a, axis : Literal['z','y']) -> float:
        """
        Compute the inner angle from vector_a to origin.
        
        Args:
            vector_a: the first vector.
            axis: the axis to compute the angle.

        Returns:
            the inner angle between the two vectors.
        """
        if axis == 'z':
            origin = np.array([1.,0.,0.])
        else:
            origin = np.array([0.,0.,1.])
        return math.degrees(np.arccos((np.dot(origin,vector_a)/(np.linalg.norm(vector_a)*np.linalg.norm(origin)))))
    
    def _compute_determinant_origin(self,vector_a, axis : Literal['z','y']):
        """
        Compute the determinant from vector_a to origin.
        
        Args:
            vector_a: the first vector.
            axis: the axis to compute the determinant.

        Returns:
            the determinant from vector_a to origin
        """
        if axis == 'z':
            origin = np.array([1.,0.,0.])
        else:    
            origin = np.array([0.,0.,1.])
            
        return np.sign(vector_a[0]*origin[1]-vector_a[1]*origin[0])*-1

    def _get_position(self, command:dict) -> np.array:
        """
        Compute the new position for the robot given a command.
        
        if the command is j1 the new position is computed rotating 
        the last j3 command around the z axis.
        if the command is j3 the new position is computed moving on the z axis.
        
        Args:
            command: the command to compute the new position.

        Returns:
            an array with coordinates x,y,z,r in cartesian coordinate system
        """
        if command['command'] == 'j1':
            theta = self.theta_z_range['min'] if command['movement'] == 0. else self.theta_z_range['max']
            m = math.tan(math.radians(theta))
            p = self.last_j3_cmd
            r = math.sqrt((p[0]**2 + p[1]**2))
            A = m**2 + 1
            C = -r**2
            x = math.sqrt((-4*A*C))/(2*A)
            y = m*x
            self.current_theta_z = theta
            self.last_j1_cmd = np.array([x,y,self.last_j3_cmd[2],self.current_theta_z])
            return np.array([x,y,self.last_j3_cmd[2],self.current_theta_z])
            
        elif command['command'] == 'j3':
            new_z = self._compute_new_theta(command)
    
            p = self.last_j1_cmd
            self.last_j3_cmd = np.array([p[0],p[1],new_z,p[3]])
            return np.array([p[0],p[1],new_z,p[3]])
        else:
            raise ValueError(f'Invalid movement type {command}')

    def compute_current_theta(self, axis : Literal['z','y'])-> float:
        """ 
        Computes the current theta for the given axis.
        
        Args: 
            axis: the axis to compute the theta.

        Returns:
            a float for the inner angle
        """
        pose = self.get_pose()
        return self._get_inner_angle_origin(np.array(pose[:3]), axis)*self._compute_determinant_origin(np.array(pose[:3]), axis)
        
    def _compute_new_theta(self, command:dict) -> float:
        """
        Computes the new movement rescaled to the j1 and j3 range.
        
        Args:
            command: the command to compute the new movement.

        Returns:
            The movement rescaled on j1/j3 ranges.
        """
        if command['command'] == 'j1':
            return command['movement']*(self.theta_z_range['max']-self.theta_z_range['min'])/100 + self.theta_z_range['min']
        elif command['command'] == 'j3':
            return command['movement']*(self.z_range['max']-self.z_range['min'])/100 + self.z_range['min']
            
    def go_to_idle(self):
        """ Moves the robot to the idle position."""
        #check grip condition, close it and disable control 
        if not self.interface.get_end_effector_gripper() == (False, False):
            self.interface.set_end_effector_gripper(True,True)
            time.sleep(1)
            self.interface.set_end_effector_gripper(False,False)
            time.sleep(1)
        
        index = self.interface.get_current_queue_index()
        self._go_to_position(self.idle_pos)
        self.wait(queue_index=index, timeout=3)
        self.interface.stop_queue(True)
        self.interface.clear_queue()
        self.interface.start_queue()
    
        
    def go_to_command(self):
        """ Moves the robot to the command position."""
        index = self.interface.get_current_queue_index()
        self._go_to_position(self.command_pos)
        self.interface.set_end_effector_gripper(True, False)
        self.wait(queue_index=index, timeout=3)
        
    def execute_stop_command(self):
        """ Stops the robot and clears the queue."""
        self.interface.stop_queue(True)
        self.interface.clear_queue()
        self.interface.start_queue()
        
        self.last_j1_cmd = self.get_pose()
        
    def execute_command(self, command:dict):
        """
        Executes the given command. It can be a grip, j1 or j3 command.

        Args:
            command: a dict containing the command to be executed
        """
        index = self.interface.get_current_queue_index()
        if command['command'] == 'grip':
            if command['movement'] == 'close':
                self.interface.set_end_effector_gripper(True, True)
            elif command['movement'] == 'open':
                self.interface.set_end_effector_gripper(True, False)
        elif command['command'] == 'j1' or command['command'] == 'j3':
            if command['movement'] == 'stop':
                self.execute_stop_command()
                return 0
            self._go_to_position(self._get_position(command))
        else:
            raise ValueError(f'Invalid command type {command}')
        self.wait(queue_index=index, timeout=0.2)
        return 0

