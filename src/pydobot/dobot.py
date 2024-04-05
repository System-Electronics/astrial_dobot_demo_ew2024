""" This module contains the Dobot class and is used to initialize the Dobot Magician interface and pass commands for
their execution. This module is consistent with the Dobot Magician Communication protocol v.1.1.5"""
import time

from pydobot.interface import Interface


class Dobot:
    """Interfaces with Dobot Magician through serial port"""
    def __init__(self, port):
        """
        Initializes the Dobot object by connecting through serial port, restarting the command queue and setting the
        parameters for the jog and point to point commands.

        Args:
            port: the port to communicate with the robot.
        """
        self.interface = Interface(port)

        self.interface.stop_queue(True)
        self.interface.clear_queue()
        self.interface.start_queue()
    
        self.interface.set_point_to_point_jump_params(10, 10)
        self.interface.set_point_to_point_joint_params([50, 50, 50, 50], [50, 50, 50, 50])
        self.interface.set_point_to_point_coordinate_params(50, 50, 50, 50)
        self.interface.set_point_to_point_common_params(50, 50)
        self.interface.set_point_to_point_jump2_params(5, 5, 5)

        self.interface.set_continous_trajectory_params(50, 50, 50)

    def connected(self):
        """
         Checks if robot is connects through serial port

         Returns:
             a boolean value, True if connected else False
        """
        return self.interface.connected()

    def get_pose(self):
        """
        Retrieves the real-time pose

        Returns:
            a tuple containing 8 values: x,y,z,r,j1,j2,j3,j4
        """
        return self.interface.get_pose()

    def move_to(self, x, y, z, r, wait=True):
        """
        Moves to the absolute coordinate, one axis at a time

        Args:
            x: float indicating the cartesian coordinate system x-axis
            y: float indicating the cartesian coordinate system y-axis
            z: float indicating the cartesian coordinate system z-axis
            r: float indicating the cartesian coordinate system r-axis
            wait: a boolean value, set True if waiting, else False
        """
        self.interface.set_point_to_point_command(3, x, y, z, r)
        if wait:
            self.wait()

    def slide_to(self, x, y, z, r, wait=True):
        """
        Slides to the absolute coordinate, the shortest possible path.

        Args:
            x: float indicating the cartesian coordinate system x-axis
            y: float indicating the cartesian coordinate system y-axis
            z: float indicating the cartesian coordinate system z-axis
            r: float indicating the cartesian coordinate system r-axis
            wait: a boolean value, set True if waiting, else False
        """
        self.interface.set_point_to_point_command(4, x, y, z, r)
        if wait:
            self.wait()

    def move_to_relative(self, x, y, z, r, wait=True):
        """
        Moves to the absolute coordinate, one axis at a time

        Args:
            x: float indicating the cartesian coordinate system x-axis
            y: float indicating the cartesian coordinate system y-axis
            z: float indicating the cartesian coordinate system z-axis
            r: float indicating the cartesian coordinate system r-axis
            wait: a boolean value, set True if waiting, else False
        """
        self.interface.set_point_to_point_command(7, x, y, z, r)
        if wait:
            self.wait()

    def slide_to_relative(self, x, y, z, r, wait=True):
        """
        Slides to the relative coordinate, one axis at a time

        Args:
            x: float indicating the cartesian coordinate system x-axis
            y: float indicating the cartesian coordinate system y-axis
            z: float indicating the cartesian coordinate system z-axis
            r: float indicating the cartesian coordinate system r-axis
            wait: a boolean value, set True if waiting, else False
        """
        self.interface.set_point_to_point_command(6, x, y, z, r)
        if wait:
            self.wait()

    # Wait until the instruction finishes
    def wait(self, queue_index=None, timeout = None):
        """
        Waits until the instruction finishes or timeout expires

        Args:
            queue_index: index of the last processed instruction
            timeout: time threshold
        """

        if queue_index is None:
            queue_index = self.interface.get_current_queue_index()
        if timeout is not None:
            start = time.time()
        while True:
            if self.interface.get_current_queue_index() > queue_index:
                break
            if timeout is not None:
                if (time.time()-start) > timeout:
                    break
            time.sleep(0.1)

