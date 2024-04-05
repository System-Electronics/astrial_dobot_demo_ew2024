""" This module implements a low level interface consistent with the Dobot Magician Communication protocol v.1.1.5"""
import serial
import threading

from pydobot.message import Message


class Interface:
    """ Manages the communication with Dobot"""
    def __init__(self, port):
        """
        Initializes the serial communication

        Args:
            port: the serial port to communicate with the robot.
        """
        threading.Thread.__init__(self)
        self.lock = threading.Lock()

        self.serial = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )

    def send(self, message):
        """
        Sends a message through serial port

        Args:
            message: a string to send as a series of bytes
        """
        self.lock.acquire()
        self.serial.write(message.package())
        self.serial.flush()
        response = Message.read(self.serial)
        self.lock.release()
        return response.params

    def connected(self):
        """
         Checks if robot is connects through serial port

         Returns:
             a boolean value, True if connected else False
        """
        return self.serial.isOpen()

    def get_device_serial_number(self):
        """
        Retrieves the device serial number

        Returns:
            a string indicating the serial number
        """
        request = Message([0xAA, 0xAA], 2, 0, False, False, [], direction='out')
        return self.send(request)

    def set_device_serial_number(self, serial_number):
        """ Sets the device serial number """
        request = Message([0xAA, 0xAA], 2, 0, True, False, [serial_number], direction='out')
        return self.send(request)

    def get_device_name(self):
        """
        Retrieves the device serial number

        Returns:
            a string indicating the serial number
        """
        request = Message([0xAA, 0xAA], 2, 1, False, False, [], direction='out')
        return self.send(request)

    def set_device_name(self, device_name):
        """ Sets the device serial number """
        request = Message([0xAA, 0xAA], 2, 1, True, False, [device_name], direction='out')
        return self.send(request)

    def get_device_version(self):
        """
        Retrieves the device version

        Returns:
            a string indicating the version
        """
        request = Message([0xAA, 0xAA], 2, 2, False, False, [], direction='out')
        return self.send(request)

    def set_sliding_rail_status(self, enable, version):
        """ Sets the device sliding rail status """
        request = Message([0xAA, 0xAA], 2, 3, True, False, [], direction='out')
        return self.send(request)

    # Time in milliseconds since start
    def get_device_time(self):
        """
        Retrieves the device internal time

        Returns:
            a string indicating the internal time
        """
        request = Message([0xAA, 0xAA], 2, 4, False, False, [], direction='out')
        return self.send(request)

    def get_device_id(self):
        """
        Retrieves the device id

        Returns:
            a string indicating the id
        """
        request = Message([0xAA, 0xAA], 2, 5, False, False, [], direction='out')
        return self.send(request)

    def get_pose(self):
        """
        Retrieves the device real time pose

        Returns:
            a tuple containing 8 values: x,y,z,r,j1,j2,j3,j4
        """
        request = Message([0xAA, 0xAA], 2, 10, False, False, [], direction='out')
        return self.send(request)

    def reset_pose(self, manual, rear_arm_angle, front_arm_angle):
        """ Reset the reference value of the real-time pose """
        request = Message([0xAA, 0xAA], 2, 11, True, False, [manual, rear_arm_angle, front_arm_angle], direction='out')
        return self.send(request)

    def get_sliding_rail_pose(self):
        """
        Retrieves the sliding rail pose

        Returns:
            a tuple containing velocity and acceleration for each joint
        """
        request = Message([0xAA, 0xAA], 2, 13, False, False, [], direction='out')
        return self.send(request)

    def get_alarms_state(self):
        """ Retrieves the alarm status """
        request = Message([0xAA, 0xAA], 2, 20, False, False, [], direction='out')
        return self.send(request)

    def clear_alarms_state(self):
        """ Clears all the status of all alarms"""
        request = Message([0xAA, 0xAA], 2, 21, True, False, [], direction='out')
        return self.send(request)

    def get_homing_paramaters(self):
        """
        Retrieves the homing position

        Returns:
            a tuple containing the x,y,z,r cartesian coordinates
        """
        request = Message([0xAA, 0xAA], 2, 30, False, False, [], direction='out')
        return self.send(request)

    def set_homing_parameters(self, x, y, z, r, queue=True):
        request = Message([0xAA, 0xAA], 2, 30, True, queue, [x, y, z, r], direction='out')
        return self.send(request)

    def set_homing_command(self, command, queue=True):
        """ Sets the homing position """
        request = Message([0xAA, 0xAA], 2, 31, True, queue, [command], direction='out')
        return self.send(request)

    # TODO: Reference is wrong here, arm does not send the said value
    def get_auto_leveling(self):
        """Retrieves the automatic leveling results"""
        request = Message([0xAA, 0xAA], 2, 32, False, False, [], direction='out')
        return self.send(request)

    def set_auto_leveling(self, enable, accuracy, queue=True):
        """ Executes the automatic leveling function"""
        request = Message([0xAA, 0xAA], 2, 32, True, queue, [enable, accuracy], direction='out')
        return self.send(request)

    def get_handheld_teaching_mode(self):
        """ Gets the hand-hold teaching trigger mode"""
        request = Message([0xAA, 0xAA], 2, 40, False, False, [], direction='out')
        return self.send(request)

    def set_handheld_teaching_mode(self, mode):
        """ Sets the hand-hold teaching trigger mode"""
        request = Message([0xAA, 0xAA], 2, 40, True, False, [mode], direction='out')
        return self.send(request)

    def get_handheld_teaching_state(self):
        """ Gets the hand-hold teaching trigger state"""
        request = Message([0xAA, 0xAA], 2, 41, False, False, [], direction='out')
        return self.send(request)

    def set_handheld_teaching_state(self, enable):
        """ Sets the hand-hold teaching trigger mode"""
        request = Message([0xAA, 0xAA], 2, 41, True, False, [enable], direction='out')
        return self.send(request)

    def get_handheld_teaching_trigger(self):
        """ Gets the hand-hold teaching trigger single"""
        request = Message([0xAA, 0xAA], 2, 42, False, False, [], direction='out')
        return self.send(request)

    def get_end_effector_params(self):
        """ Gets the offset of the end-effector"""
        request = Message([0xAA, 0xAA], 2, 60, False, False, [], direction='out')
        return self.send(request)

    def set_end_effector_params(self, bias_x, bias_y, bias_z):
        """ Sets the offset of the end-effector"""
        request = Message([0xAA, 0xAA], 2, 60, True, False, [bias_x, bias_y, bias_z], direction='out')
        return self.send(request)

    def get_end_effector_laser(self):
        """ Gets the status of the laser"""
        request = Message([0xAA, 0xAA], 2, 61, False, False, [], direction='out')
        return self.send(request)

    def set_end_effector_laser(self, enable_control, enable_laser, queue=True):
        """ Sets the status of the laser"""
        request = Message([0xAA, 0xAA], 2, 61, True, queue, [enable_control, enable_laser], direction='out')
        return self.send(request)

    def get_end_effector_suction_cup(self):
        """ Gets the status of the air pump"""
        request = Message([0xAA, 0xAA], 2, 62, False, False, [], direction='out')
        return self.send(request)

    def set_end_effector_suction_cup(self, enable_control, enable_suction, queue=True):
        """ Setss the status of the air pump"""
        request = Message([0xAA, 0xAA], 2, 62, True, queue, [enable_control, enable_suction], direction='out')
        return self.send(request)

    def get_end_effector_gripper(self):
        """ Gets the status of the gripper"""
        request = Message([0xAA, 0xAA], 2, 63, False, False, [], direction='out')
        return self.send(request)

    def set_end_effector_gripper(self, enable_control, enable_grip, queue=True):
        """ Sets the status of the air pump"""
        request = Message([0xAA, 0xAA], 2, 63, True, queue, [enable_control, enable_grip], direction='out')
        return self.send(request)

    def get_jog_joint_params(self):
        """
        Retrieves the join parameter when jogging in joints coordinates

        Returns:
            a tuple containing velocity and acceleration for each joint
        """
        request = Message([0xAA, 0xAA], 2, 70, False, False, [], direction='out')
        return self.send(request)

    # TODO: Does not work - but is implemented according to spec. Bad documentation?
    def set_jog_joint_params(self, velocity, acceleration, queue=True):
        """ Sets the join parameter when jogging in joints coordinates"""
        request = Message([0xAA, 0xAA], 2, 70, True, queue, velocity + acceleration, direction='out')
        return self.send(request)

    def get_jog_coordinate_params(self):
        """
        Retrieves the join parameter when jogging in cartesian coordinates

        Returns:
            a tuple containing velocity and acceleration for each joint
        """
        request = Message([0xAA, 0xAA], 2, 71, False, False, [], direction='out')
        return self.send(request)

    # TODO: Does not work - but is implemented according to spec. Bad documentation?
    def set_jog_coordinate_params(self, velocity, acceleration, queue=True):
        """ Sets the join parameter when jogging in cartesian coordinates"""
        request = Message([0xAA, 0xAA], 2, 71, True, queue, velocity + acceleration, direction='out')
        return self.send(request)

    def get_jog_common_params(self):
        """
        Retrieves the velocity ratio and acceleration ratio when jogging

        Returns:
            a tuple containing velocity and acceleration for each joint
        """
        request = Message([0xAA, 0xAA], 2, 72, False, False, [], direction='out')
        return self.send(request)

    # TODO: Does not work - but is implemented according to spec. Bad documentation?
    def set_jog_common_params(self, velocity_ratio, acceleration_ratio, queue=True):
        """ Sets the velocity ratio and acceleration ratio when jogging """
        request = Message([0xAA, 0xAA], 2, 72, True, queue, [velocity_ratio, acceleration_ratio], direction='out')
        return self.send(request)

    def set_jog_command(self, jog_type, command, queue=True):
        """ Executes a jogging command """
        request = Message([0xAA, 0xAA], 2, 73, True, queue, [jog_type, command], direction='out')
        return self.send(request)

    def get_sliding_rail_jog_params(self):
        """
        Retrieves the velocity ratio and acceleration  of the sliding rail when jogging

        Returns:
            a tuple containing velocity and acceleration for each joint
        """
        request = Message([0xAA, 0xAA], 2, 74, False, False, [], direction='out')
        return self.send(request)

    def set_sliding_rail_jog_params(self, velocity, acceleration, queue=True):
        """ Sets the velocity ratio and acceleration of the sliding rail when jogging """
        request = Message([0xAA, 0xAA], 2, 74, True, queue, [velocity, acceleration], direction='out')
        return self.send(request)

    def get_point_to_point_joint_params(self):
        """
        Retrieves the velocity ratio and acceleration of the joint coordinate axis in PTP mode

        Returns:
            a tuple containing velocity and acceleration for each joint
        """
        request = Message([0xAA, 0xAA], 2, 80, False, False, [], direction='out')
        return self.send(request)

    def set_point_to_point_joint_params(self, velocity, acceleration, queue=True):
        """
        Sets the velocity ratio and acceleration ratio of the joint coordinate axis in PTP mode

        Args:
            velocity: a list of velocities, one for each joint
            acceleration: a list of accelerations, one for each joint
        """
        request = Message([0xAA, 0xAA], 2, 80, True, queue, velocity + acceleration, direction='out')
        return self.send(request)

    def get_point_to_point_coordinate_params(self):
        """
        Retrieves the velocity ratio and acceleration of the cartesian coordinate axis in PTP mode

        Returns:
            a tuple containing velocity and acceleration for each joint
        """
        request = Message([0xAA, 0xAA], 2, 81, False, False, [], direction='out')
        return self.send(request)

    def set_point_to_point_coordinate_params(self, coordinate_velocity, effector_velocity, coordinate_acceleration, effector_acceleration, queue=True):
        """ Sets the velocity ratio and acceleration of the cartesian coordinate axis in PTP mode """
        request = Message([0xAA, 0xAA], 2, 81, True, queue, [coordinate_velocity, effector_velocity, coordinate_acceleration, effector_acceleration], direction='out')
        return self.send(request)

    def get_point_to_point_jump_params(self):
        """
        Retrieves the lifting height and the maximum lifting height in JUMP mode

        Returns:
            a tuple of float point containing lifting height and maximum lifting height
        """
        request = Message([0xAA, 0xAA], 2, 82, False, False, [], direction='out')
        return self.send(request)

    def set_point_to_point_jump_params(self, jump_height, z_limit, queue=True):
        """
        Sets the lifting height and the maximum lifting height in JUMP mode

        Args:
            jump_height: float number for lifting height
            z_limit: float number for maximum lifting height
            queue: boolean value, True if add this command to queue else False
        """
        request = Message([0xAA, 0xAA], 2, 82, True, queue, [jump_height, z_limit], direction='out')
        return self.send(request)

    def get_point_to_point_common_params(self):
        """
        Retrieves the velocity ratio and acceleration ratio in PTP mode

        Returns:
            a tuple containing velocity and acceleration ratio
        """
        request = Message([0xAA, 0xAA], 2, 83, False, False, [], direction='out')
        return self.send(request)

    def set_point_to_point_common_params(self, velocity_ratio, acceleration_ratio, queue=True):
        """
        Sets the velocity ratio and the acceleration ratio in PTP mode

        Args:
            velocity_ratio: float value for velocity ratio
            acceleration_ratio: float value for acceleration ratio
            queue: boolean value, True if add this command to queue else False
        """
        request = Message([0xAA, 0xAA], 2, 83, True, queue, [velocity_ratio, acceleration_ratio], direction='out')
        return self.send(request)

    def set_point_to_point_command(self, mode, x, y, z, r, queue=True):
        """
        Executes a PTP command

        Args:
            mode: int value PTP mode (0-9)
            x: float indicating the cartesian coordinate system x-axis
            y: float indicating the cartesian coordinate system y-axis
            z: float indicating the cartesian coordinate system z-axis
            r: float indicating the cartesian coordinate system r-axis
            queue: boolean value, True if add this command to queue else False
        """
        request = Message([0xAA, 0xAA], 2, 84, True, queue, [mode, x, y, z, r], direction='out')
        return self.send(request)

    def get_point_to_point_sliding_rail_params(self):
        """
        Retrieves the velocity and acceleration of the Sliding rail s in PTP mode

        Returns:
            a tuple containing velocity and acceleration
        """
        request = Message([0xAA, 0xAA], 2, 85, False, False, [], direction='out')
        return self.send(request)

    def set_point_to_point_sliding_rail_params(self, velocity, acceleration, queue=True):
        """
        Sets the velocity and acceleration of the Sliding rail s in PTP mode

        Args:
            velocity: float value for velocity
            acceleration: float value for acceleration
            queue: boolean value, True if add this command to queue else False
        """
        request = Message([0xAA, 0xAA], 2, 85, True, queue, [velocity, acceleration], direction='out')
        return self.send(request)

    def set_point_to_point_sliding_rail_command(self, mode, x, y, z, r, l, queue=True):
        """
        Executes a PTP command with the sliding rail

        Args:
            mode: int value PTP mode (0-9)
            x: float indicating the cartesian coordinate system x-axis
            y: float indicating the cartesian coordinate system y-axis
            z: float indicating the cartesian coordinate system z-axis
            r: float indicating the cartesian coordinate system r-axis
            l: float indicating the distance that sliding rail moves
            queue: boolean value, True if add this command to queue else False
        """
        request = Message([0xAA, 0xAA], 2, 86, True, queue, [mode, x, y, z, r, l], direction='out')
        return self.send(request)

    def get_point_to_point_jump2_params(self):
        """
        Retrieves extended parameters in JUMP mode

        Returns:
            a tuple containing lifting height of starting point, lifting height of end point, maximum lifting height
        """
        request = Message([0xAA, 0xAA], 2, 87, False, False, [], direction='out')
        return self.send(request)

    def set_point_to_point_jump2_params(self, start_height, end_height, z_limit, queue=True):
        """Sets the extended parameters in JUMP mode"""
        request = Message([0xAA, 0xAA], 2, 87, True, queue, [start_height, end_height, z_limit], direction='out')
        return self.send(request)

    # TODO: Reference is ambigious here - needs testing
    def set_point_to_point_po_command(self, mode, x, y, z, r, queue=True):
        request = Message([0xAA, 0xAA], 2, 88, True, queue, [mode, x, y, z, r], direction='out')
        return self.send(request)

    # TODO: Reference is ambigious here - needs testing
    def set_point_to_point_sliding_rail_po_command(self, ratio, address, level, queue=True):
        request = Message([0xAA, 0xAA], 2, 89, True, queue, [ratio, address, level], direction='out')
        return self.send(request)

    def get_continous_trajectory_params(self):
        request = Message([0xAA, 0xAA], 2, 90, False, False, [], direction='out')
        return self.send(request)

    def set_continous_trajectory_params(self, max_planned_acceleration, max_junction_velocity, acceleration, queue=True):
        request = Message([0xAA, 0xAA], 2, 90, True, queue, [max_planned_acceleration, max_junction_velocity, acceleration, 0], direction='out')
        return self.send(request)

    def set_continous_trajectory_real_time_params(self, max_planned_acceleration, max_junction_velocity, period, queue=True):
        request = Message([0xAA, 0xAA], 2, 90, True, queue, [max_planned_acceleration, max_junction_velocity, period, 1], direction='out')
        return self.send(request)

    def set_continous_trajectory_command(self, mode, x, y, z, velocity, queue=True):
        request = Message([0xAA, 0xAA], 2, 91, True, queue, [mode, x, y, z, velocity], direction='out')
        return self.send(request)

    def set_continous_trajectory_laser_engraver_command(self, mode, x, y, z, power, queue=True):
        request = Message([0xAA, 0xAA], 2, 92, True, queue, [mode, x, y, z, power], direction='out')
        return self.send(request)

    def get_arc_params(self):
        request = Message([0xAA, 0xAA], 2, 100, False, False, [], direction='out')
        return self.send(request)

    def set_arc_params(self, coordinate_velocity, effector_velocity, coordinate_acceleration, effector_acceleration, queue=True):
        request = Message([0xAA, 0xAA], 2, 100, True, queue, [coordinate_velocity, effector_velocity, coordinate_acceleration, effector_acceleration], direction='out')
        return self.send(request)

    def set_arc_command(self, circumference_point, ending_point, queue=True):
        request = Message([0xAA, 0xAA], 2, 101, True, queue, circumference_point + ending_point, direction='out')
        return self.send(request)

    def wait(self, milliseconds, queue=True):
        request = Message([0xAA, 0xAA], 2, 110, True, queue, [milliseconds], direction='out')
        return self.send(request)

    def set_trigger_command(self, address, mode, condition, threshold, queue=True):
        request = Message([0xAA, 0xAA], 2, 120, True, queue, [address, mode, condition, threshold], direction='out')
        return self.send(request)

    def get_io_multiplexing(self):
        request = Message([0xAA, 0xAA], 2, 130, False, False, [], direction='out')
        return self.send(request)

    def set_io_multiplexing(self, address, multiplex, queue=True):
        request = Message([0xAA, 0xAA], 2, 130, True, queue, [address, multiplex], direction='out')
        return self.send(request)

    def get_io_do(self):
        request = Message([0xAA, 0xAA], 2, 131, False, False, [], direction='out')
        return self.send(request)

    def set_io_do(self, address, level, queue=True):
        request = Message([0xAA, 0xAA], 2, 131, True, queue, [address, level], direction='out')
        return self.send(request)

    def get_io_pwm(self):
        request = Message([0xAA, 0xAA], 2, 132, False, False, [], direction='out')
        return self.send(request)

    def set_io_pwm(self, address, frequency, duty_cycle, queue=True):
        request = Message([0xAA, 0xAA], 2, 132, True, queue, [address, frequency, duty_cycle], direction='out')
        return self.send(request)

    def get_io_di(self):
        request = Message([0xAA, 0xAA], 2, 133, False, False, [], direction='out')
        return self.send(request)

    def get_io_adc(self):
        request = Message([0xAA, 0xAA], 2, 134, False, False, [], direction='out')
        return self.send(request)

    def set_extended_motor_velocity(self, index, enable, speed, queue=True):
        request = Message([0xAA, 0xAA], 2, 135, True, queue, [index, enable, speed], direction='out')
        return self.send(request)

    def get_color_sensor(self, index):
        request = Message([0xAA, 0xAA], 2, 137, False, False, [], direction='out')
        return self.send(request)

    def set_color_sensor(self, index, enable, port, version, queue=True):
        request = Message([0xAA, 0xAA], 2, 137, True, queue, [enable, port, version], direction='out')
        return self.send(request)

    def get_ir_switch(self, index):
        request = Message([0xAA, 0xAA], 2, 138, False, False, [], direction='out')
        return self.send(request)

    def set_ir_switch(self, index, enable, port, version, queue=True):
        request = Message([0xAA, 0xAA], 2, 138, True, queue, [enable, port, version], direction='out')
        return self.send(request)

    def get_angle_sensor_static_error(self, index):
        request = Message([0xAA, 0xAA], 2, 140, False, False, [], direction='out')
        return self.send(request)

    def set_angle_sensor_static_error(self, index, rear_arm_angle_error, front_arm_angle_error):
        request = Message([0xAA, 0xAA], 2, 140, True, False, [rear_arm_angle_error, front_arm_angle_error], direction='out')
        return self.send(request)

    def get_wifi_status(self):
        request = Message([0xAA, 0xAA], 2, 150, False, False, [], direction='out')
        return self.send(request)

    def set_wifi_status(self, index, enable):
        request = Message([0xAA, 0xAA], 2, 150, True, False, [enable], direction='out')
        return self.send(request)

    def get_wifi_ssid(self):
        request = Message([0xAA, 0xAA], 2, 151, False, False, [], direction='out')
        return self.send(request)

    def set_wifi_ssid(self, index, ssid):
        request = Message([0xAA, 0xAA], 2, 151, True, False, [ssid], direction='out')
        return self.send(request)

    def get_wifi_password(self):
        request = Message([0xAA, 0xAA], 2, 152, False, False, [], direction='out')
        return self.send(request)

    def set_wifi_password(self, index, ssid):
        request = Message([0xAA, 0xAA], 2, 152, True, False, [ssid], direction='out')
        return self.send(request)

    def get_wifi_address(self):
        request = Message([0xAA, 0xAA], 2, 153, False, False, [], direction='out')
        return self.send(request)

    # 192.168.1.1 = a.b.c.d
    def set_wifi_address(self, index, use_dhcp, a, b, c, d):
        request = Message([0xAA, 0xAA], 2, 153, True, False, [use_dhcp, a, b, c, d], direction='out')
        return self.send(request)

    def get_wifi_netmask(self):
        request = Message([0xAA, 0xAA], 2, 154, False, False, [], direction='out')
        return self.send(request)

    # 255.255.255.0 = a.b.c.d
    def set_wifi_netmask(self, index, a, b, c, d):
        request = Message([0xAA, 0xAA], 2, 154, True, False, [a, b, c, d], direction='out')
        return self.send(request)

    def get_wifi_gateway(self):
        request = Message([0xAA, 0xAA], 2, 155, False, False, [], direction='out')
        return self.send(request)

    # 192.168.1.1 = a.b.c.d
    def set_wifi_gateway(self, index, use_dhcp, a, b, c, d):
        request = Message([0xAA, 0xAA], 2, 155, True, False, [use_dhcp, a, b, c, d], direction='out')
        return self.send(request)

    def get_wifi_dns(self):
        request = Message([0xAA, 0xAA], 2, 156, False, False, [], direction='out')
        return self.send(request)

    # 192.168.1.1 = a.b.c.d
    def set_wifi_dns(self, index, use_dhcp, a, b, c, d):
        request = Message([0xAA, 0xAA], 2, 156, True, False, [use_dhcp, a, b, c, d], direction='out')
        return self.send(request)

    def get_wifi_connect_status(self):
        request = Message([0xAA, 0xAA], 2, 157, False, False, [], direction='out')
        return self.send(request)

    def set_lost_step_params(self, param):
        request = Message([0xAA, 0xAA], 2, 170, True, False, [param], direction='out')
        return self.send(request)

    def set_lost_step_command(self):
        request = Message([0xAA, 0xAA], 2, 171, True, False, [], direction='out')
        return self.send(request)

    def start_queue(self):
        request = Message([0xAA, 0xAA], 2, 240, True, False, [], direction='out')
        return self.send(request)

    def stop_queue(self, force=False):
        request = Message([0xAA, 0xAA], 2, 242 if force else 241, True, False, [], direction='out')
        return self.send(request)

    def start_queue_download(self, total_loop, line_per_loop):
        request = Message([0xAA, 0xAA], 2, 243, True, False, [total_loop, line_per_loop], direction='out')
        return self.send(request)

    def stop_queue_download(self):
        request = Message([0xAA, 0xAA], 2, 244, True, False, [], direction='out')
        return self.send(request)

    def clear_queue(self):
        request = Message([0xAA, 0xAA], 2, 245, True, False, [], direction='out')
        return self.send(request)

    def get_current_queue_index(self):
        request = Message([0xAA, 0xAA], 2, 246, True, False, [], direction='out')
        return self.send(request)
