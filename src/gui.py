""" This module contains the Gui class, which is responsible for the graphical user interface of the application."""
import cv2 as cv
import numpy as np
import os
import pathlib
import math
from common import get_config_param, get_image_crop_coordinates

class Gui():
    """ Manages the graphical user interface of the application. It displays the camera feed and the icons that represent the hand gestures and the face detection."""
    def __init__(self, fsm, camera, hand_detected_r, hand_detected_l, fingers_r, fingers_l, gesture_r, face_detected) -> None:
        """ 
        Initializes the Gui object, including the icons and the colors used in the interface.
        
        Args:
            fsm: The finite state machine object.
            camera: The camera object.
            hand_detected_r: The event associated with the right hand detected.
            hand_detected_l: The event associated with the left hand detected.
            fingers_r: The object containing the right hand fingers positions.
            fingers_l: The object containing the left hand fingers positions.
            gesture_r: The object containing the right hand gesture.
            face_detected: The event associated with the face detected.
        """
        self.fsm = fsm
        self.camera = camera
        self.default_layout = 'idle'
        self.hand_detected_r = hand_detected_r
        self.hand_detected_l = hand_detected_l
        self.fingers_r = fingers_r
        self.fingers_l = fingers_l
        self.gesture_r = gesture_r
        self.padding = 10
        self.face_detected = face_detected
        
        self.frame_r_coordinates, self.frame_l_coordinates = get_image_crop_coordinates(crop_type='gui')
        self.frame_r_coordinates_hd, self.frame_l_coordinates_hd = get_image_crop_coordinates(crop_type='hand_detection')
        self.screen_dim = (get_config_param('image_configuration','screen_resolution_h'),
                           get_config_param('image_configuration','screen_resolution_w'))
        self.camera_resolution = (get_config_param('image_configuration','camera_resolution_h'),
                                get_config_param('image_configuration','camera_resolution_w'))
        
        #load and process icons 
        icon_dir = pathlib.Path(os.path.realpath(__file__)).parents[0] / 'icons'
        
        self.icon_arrow_up = cv.imread(rf'{icon_dir}/arrow_up.png', -1)
        self.icon_arrow_up_half = cv.imread(rf'{icon_dir}/arrow_up_half.png', -1)
        self.icon_arrow_up_double = cv.imread(rf'{icon_dir}/arrow_up_double.png', -1)
        self.icon_arrow_up_red = cv.imread(rf'{icon_dir}/arrow_up_red.png', -1)
        
        self.icon_arrow_down = cv.imread(rf'{icon_dir}/arrow_down.png', -1)
        self.icon_arrow_down_half = cv.imread(rf'{icon_dir}/arrow_down_half.png', -1)
        self.icon_arrow_down_double = cv.imread(rf'{icon_dir}/arrow_down_double.png', -1)
        self.icon_arrow_down_red = cv.imread(rf'{icon_dir}/arrow_down_red.png', -1)
        
        icon_grip_open = cv.imread(rf'{icon_dir}/grip_open_icon.png', -1)
        rf_open = self._get_reshape_factor(icon_grip_open, self.frame_l_coordinates_hd[1])
        icon_grip_open = cv.resize(icon_grip_open, (0,0), fx=rf_open, fy=rf_open)
        a_channel = np.ones(icon_grip_open.shape, dtype=float)/1.2
        self.icon_grip_open = icon_grip_open*a_channel
        self.icon_grip_open_alpha_i = self.icon_grip_open[:, :, 3] / 255.0 
        self.icon_grip_open_alpha_f = 1.0 - self.icon_grip_open_alpha_i 
        self.icon_grip_open_uneven_padding_w = 0 if (self.frame_l_coordinates_hd[3]-self.frame_l_coordinates_hd[2])//2 %2 == 0 else 1 
        self.icon_grip_open_hmin = self.frame_l_coordinates_hd[1] + self.padding//2
        self.icon_grip_open_hmax = self.frame_l_coordinates_hd[1] + self.padding//2 + self.icon_grip_open.shape[0]
        self.icon_grip_open_wmin = (self.frame_l_coordinates_hd[3]-self.frame_l_coordinates_hd[2])//2 + self.frame_l_coordinates_hd[2] + self.padding
        self.icon_grip_open_wmax = (self.frame_l_coordinates_hd[3]-self.frame_l_coordinates_hd[2])//2 + self.frame_l_coordinates_hd[2] + self.padding + self.icon_grip_open.shape[1] + self.icon_grip_open_uneven_padding_w
        
        
        icon_grip_close = cv.imread(rf'{icon_dir}/grip_close_icon.png', -1)
        rf_close = self._get_reshape_factor(icon_grip_close, self.frame_l_coordinates_hd[1])
        icon_grip_close = cv.resize(icon_grip_close, (0,0), fx=rf_close, fy=rf_close)
        a_channel = np.ones(icon_grip_close.shape, dtype=float)/1.2
        self.icon_grip_close = icon_grip_close*a_channel
        self.icon_grip_close_alpha_i = self.icon_grip_close[:, :, 3] / 255.0 
        self.icon_grip_close_alpha_f = 1.0 - self.icon_grip_close_alpha_i 
        self.icon_grip_close_uneven_padding_w = 0 if (self.frame_l_coordinates_hd[3]-self.frame_l_coordinates_hd[2])//2 %2 == 0 else 1
        self.icon_grip_close_hmin = self.frame_l_coordinates_hd[1] + self.padding//2 
        self.icon_grip_close_hmax = self.frame_l_coordinates_hd[1] + self.padding//2 + self.icon_grip_close.shape[0]
        self.icon_grip_close_wmin = (self.frame_l_coordinates_hd[3]-self.frame_l_coordinates_hd[2])//2 + self.frame_l_coordinates_hd[2] - self.padding - self.icon_grip_close.shape[1]
        self.icon_grip_close_wmax = (self.frame_l_coordinates_hd[3]-self.frame_l_coordinates_hd[2])//2 + self.frame_l_coordinates_hd[2] - self.padding + self.icon_grip_close_uneven_padding_w
        
        j1_movement = cv.imread(rf'{icon_dir}/j1_movement_icon.png', -1)
        rf_j1 = self._get_reshape_factor(j1_movement, self.frame_r_coordinates_hd[1])
        j1_movement = cv.resize(j1_movement, (0,0), fx=rf_j1, fy=rf_j1)
        a_channel = np.ones(j1_movement.shape, dtype=float)/1.2
        self.icon_j1_movement = j1_movement*a_channel
        self.icon_j1_alpha_i = self.icon_j1_movement[:, :, 3] / 255.0 
        self.icon_j1_alpha_f = 1.0 - self.icon_j1_alpha_i 
        self.icon_j1_uneven_padding_w = 0 if (self.frame_r_coordinates_hd[3]-self.frame_r_coordinates_hd[2])//2 %2 == 0 else 1
        self.icon_j1_hmin = self.frame_r_coordinates_hd[1] + self.padding//2
        self.icon_j1_hmax = self.frame_r_coordinates_hd[1] + self.padding//2 + self.icon_j1_movement.shape[0]
        self.icon_j1_wmin = (self.frame_r_coordinates_hd[3]-self.frame_r_coordinates_hd[2])//2 + self.frame_r_coordinates_hd[2] + self.padding
        self.icon_j1_wmax = (self.frame_r_coordinates_hd[3]-self.frame_r_coordinates_hd[2])//2 + self.frame_r_coordinates_hd[2] + self.padding + self.icon_j1_movement.shape[1] + self.icon_j1_uneven_padding_w
        
        
        j3_movement = cv.imread(rf'{icon_dir}/j3_movement_icon.png', -1)
        rf_j3 = self._get_reshape_factor(j3_movement, self.frame_r_coordinates_hd[1])
        j3_movement = cv.resize(j3_movement, (0,0), fx=rf_j3, fy=rf_j3)
        a_channel = np.ones(j3_movement.shape, dtype=float)/1.2
        self.icon_j3_movement = j3_movement*a_channel
        self.icon_j3_alpha_i = self.icon_j3_movement[:, :, 3] / 255.0 
        self.icon_j3_alpha_f = 1.0 - self.icon_j3_alpha_i 
        self.icon_j3_uneven_padding_w = 0 if (self.frame_r_coordinates_hd[3]-self.frame_r_coordinates_hd[2])//2 %2 == 0 else 1
        self.icon_j3_hmin = self.frame_r_coordinates_hd[1] + self.padding//2
        self.icon_j3_hmax = self.frame_r_coordinates_hd[1] + self.padding//2 + self.icon_j3_movement.shape[0]
        self.icon_j3_wmin = (self.frame_r_coordinates_hd[3]-self.frame_r_coordinates_hd[2])//2 + self.frame_r_coordinates_hd[2] - self.padding - self.icon_j3_movement.shape[1]
        self.icon_j3_wmax = (self.frame_r_coordinates_hd[3]-self.frame_r_coordinates_hd[2])//2 + self.frame_r_coordinates_hd[2] - self.padding + self.icon_j3_uneven_padding_w
        
        icon_face = cv.imread(rf'{icon_dir}/icon_face.png', -1)
        rf_face = self._get_reshape_factor(icon_face, 0)
        icon_face = cv.resize(icon_face, (0,0), fx=rf_face, fy=rf_face)
        a_channel = np.ones(icon_face.shape, dtype=float)/1.5
        self.icon_face = (icon_face*a_channel).astype(np.uint8)
        self.icon_face_uneven_padding_w = 0 if self.icon_face.shape[1]%2 == 0 else 1
        self.icon_face_uneven_padding_h = 0 if self.icon_face.shape[0]%2 == 0 else 1
        self.icon_face_alpha_i = self.icon_face[:, :, 3] / 255.0 
        self.icon_face_alpha_f = 1.0 - self.icon_face_alpha_i 
        self.icon_face_green = self.icon_face.copy()
        conv_hsv_gray = cv.cvtColor(self.icon_face_green, cv.COLOR_BGR2GRAY)
        _, mask = cv.threshold(conv_hsv_gray, 0, 255,cv.THRESH_BINARY_INV |cv.THRESH_OTSU)
        icon_face_green_tmp = self.icon_face_green[:,:,:-1]
        icon_face_green_tmp[mask == 255] = [0,255,0]
        self.icon_face_green[:,:,:-1] = icon_face_green_tmp[:,:,:]
        self.icon_face_hmin = (self.camera_resolution[0]//2) - (self.icon_face.shape[0]//2)
        self.icon_face_hmax = (self.camera_resolution[0]//2) + (self.icon_face.shape[0]//2) + self.icon_face_uneven_padding_h
        self.icon_face_wmin = (self.camera_resolution[1]//2) - (self.icon_face.shape[1]//2)
        self.icon_face_wmax = (self.camera_resolution[1]//2) + (self.icon_face.shape[1]//2) + self.icon_face_uneven_padding_w
    

        #set elements color: BGR format
        self.rect_hand_detected_color =  	(6,159,11)
        self.rect_hand_not_detected_color = (144, 144, 144)
        self.led_face_detected_color =  	(6,159,11)
        self.led_face_not_detected_color = (0, 0, 255)
        self.fingers_tip_color =  	(67,102,122)
        self.hand_center_color =  	(67,102,122)
        self.rotation_parabola_color_deactive =   	(142,107,97)
        self.rotation_parabola_color_active = (0,255,0)
        self.rotation_parabola_color_stop = (0,0,255)
        self.point_rotation_color = (109,70,59)
        
        self.is_close = False
        
        
    def _add_right_hand_icon(self, frame, finger_pos):
        """
        Adds icons to the frame that represent the right hand gestures. Arrow icons are associated with the j3 movement, 
        and the rotation icon is associated with the j1 movement.
        
        Args:
            frame: The frame to which the icons are added.
            finger_pos: The positions of the fingers.
        """
        hand_center = np.array([(finger_pos['wrist_position'][0]-finger_pos['middle_position'][0])//2 + finger_pos['middle_position'][0], 
                                (finger_pos['wrist_position'][1]-finger_pos['middle_position'][1])//2 + finger_pos['middle_position'][1]]
                            ).astype(int)
        hand_center_x, hand_center_y = hand_center  
        hand_center_x = hand_center_x + self.frame_r_coordinates_hd[2]
        hand_center_y = hand_center_y + self.frame_r_coordinates_hd[0]
        cv.circle(frame, center=(int(hand_center_x) ,int(hand_center_y)), radius=4, color=self.hand_center_color, thickness=-1)
        
        # ARRROW ICONS
        j3_movement = self.gesture_r.get_gesture('j3')
        if j3_movement is not None:

            # Right Hand Icon reshaping:
            # The arrow_up icon is reshaped given the middle finger position, gets smaller as the hand moves up
            # The arrow_down icon is reshaped given the wrist position, gets smaller as the hand moves down
            if j3_movement == 0.:
                icon_arrow_down = self.icon_arrow_down_red
                icon_arrow_up = self.icon_arrow_up_half
            elif j3_movement == 100.:
                icon_arrow_down = self.icon_arrow_down_half
                icon_arrow_up = self.icon_arrow_up_red
            elif j3_movement == 50.:
                icon_arrow_down = self.icon_arrow_down
                icon_arrow_up = self.icon_arrow_up
            elif j3_movement < 50. and j3_movement > 0.:
                icon_arrow_down = self.icon_arrow_down_double
                icon_arrow_up = self.icon_arrow_up_half
            elif j3_movement > 50. and j3_movement < 100.:
                icon_arrow_down = self.icon_arrow_down_half
                icon_arrow_up = self.icon_arrow_up_double
            
            alpha_i = icon_arrow_up[:, :, 3] / 255.0 
            alpha_f = 1.0 - alpha_i 
            if icon_arrow_up.shape[1]%2 == 0:
                uneven_padding_w = 0
            else:
                uneven_padding_w = 1
            ranges = [hand_center_y - icon_arrow_up.shape[0], hand_center_y,
                    hand_center_x - icon_arrow_up.shape[1]//2, hand_center_x + icon_arrow_up.shape[1]//2 + uneven_padding_w]
            self._add_icon(frame, icon_arrow_up, alpha_i, alpha_f,ranges)

            alpha_i = icon_arrow_down[:, :, 3] / 255.0 
            alpha_f = 1.0 - alpha_i 
            if icon_arrow_down.shape[1]%2 == 0:
                uneven_padding_w = 0
            else:
                uneven_padding_w = 1
            ranges = [hand_center_y, hand_center_y + icon_arrow_down.shape[0], 
                    hand_center_x - icon_arrow_down.shape[1]//2, hand_center_x + icon_arrow_down.shape[1]//2 + uneven_padding_w]
            self._add_icon(frame, icon_arrow_down, alpha_i, alpha_f, ranges)

        
        #Rotation Icon:        
        j1_movement = self.gesture_r.get_gesture('j1')
        if j1_movement is not None: # might be None
            start_x = max(self.frame_r_coordinates[2],hand_center_x - (hand_center_y-finger_pos['middle_position'][1] + self.frame_r_coordinates[0])/4)
            start_y = min(self.frame_r_coordinates[1],(hand_center_y-finger_pos['middle_position'][1] + self.frame_r_coordinates[0])/4 + hand_center_y)
            middle_x = hand_center_x
            middle_y = hand_center_y
            end_x = min(self.frame_r_coordinates[3],hand_center_x + (hand_center_y-finger_pos['middle_position'][1] + self.frame_r_coordinates[0])/4)
            end_y =  min(self.frame_r_coordinates[1],(hand_center_y-finger_pos['middle_position'][1] + self.frame_r_coordinates[0])/4 + hand_center_y)
            
            points = np.array([[ start_x, start_y],    # start point
                                [middle_x , middle_y], # middle point                                                                                                                                         
                                [end_x, end_y]])       # end point
            
            coeffs = np.polyfit(points[:,0], points[:,1], 2)
            poly = np.poly1d(coeffs)
            xarr = np.arange(start_x, end_x)
            yarr = poly(xarr)
            parab_pts = np.array([xarr, yarr],dtype=np.int32).T
            limit_min = (parab_pts[-1][0]-parab_pts[0][0])*self.gesture_r.range_percentage_movement_j1['min']/100 + parab_pts[0][0]
            limit_max = (parab_pts[-1][0]-parab_pts[0][0])*self.gesture_r.range_percentage_movement_j1['max']/100 + parab_pts[0][0]
            cv.polylines(frame,
                         [parab_pts[np.where((parab_pts[:,0] <= limit_min), True, False)]],
                         False,
                         self.rotation_parabola_color_active if 100-j1_movement <= self.gesture_r.range_percentage_movement_j1['min'] 
                         else self.rotation_parabola_color_deactive,
                         3)
            cv.polylines(frame,
                         [parab_pts[np.where((parab_pts[:,0] >= limit_max), True, False)]],
                         False,
                         self.rotation_parabola_color_active if 100-j1_movement >= self.gesture_r.range_percentage_movement_j1['max'] 
                         else self.rotation_parabola_color_deactive,
                         3)
            cv.polylines(frame,
                         [parab_pts[np.where((parab_pts[:,0] > limit_min) & (parab_pts[:,0] < limit_max), True, False)]], 
                         False, 
                         self.rotation_parabola_color_stop if 100-j1_movement > self.gesture_r.range_percentage_movement_j1['min'] 
                         and 100-j1_movement < self.gesture_r.range_percentage_movement_j1['max'] else self.rotation_parabola_color_deactive,
                         3)
            tr_r = np.array([[int(20*math.cos(math.radians(-20)) + xarr[0]-5), int(20*math.sin(math.radians(-20)) + poly(xarr[0]-5))],
                            [int(xarr[0]-5), int(poly(xarr[0]-5))],
                            [int(20*math.cos(math.radians(-110)) + xarr[0]-5),int(20*math.sin(math.radians(-110)) + poly(xarr[0]-5))]])
            cv.drawContours(frame, [tr_r], 0, self.rotation_parabola_color_deactive, -1)
            
            tr_l = np.array([[int(20*math.cos(math.radians(-160)) + xarr[-1]+5), int(20*math.sin(math.radians(-160)) + poly(xarr[-1]+5))],
                            [int(xarr[-1]+5), int(poly(xarr[-1]+5))],
                            [int(20*math.cos(math.radians(-70)) + xarr[-1]+5),int(20*math.sin(math.radians(-70)) + poly(xarr[-1]+5))]])
            cv.drawContours(frame, [tr_l], 0, self.rotation_parabola_color_deactive, -1)            
            
            cv.circle(frame,
                    center=(int(np.percentile(xarr, 100-j1_movement)),
                            int(poly(np.percentile(xarr, j1_movement)))),
                    radius=10, 
                    color=self.point_rotation_color,
                    thickness=-1) 
        
    def _get_reshape_factor(self, icon, frame_l_max_h):
        """ 
        Computes the reshape factor for the icon given the maximum height of the frame, so that the icon fits in the frame.
        
        Args:
            icon: The icon to be reshaped.
            frame_l_max_h: The maximum height of the frame.
        """
        camera_h = self.camera_resolution[0]
        
        reshape_factor = 1
        while True:
            if (camera_h - frame_l_max_h - self.padding) >= icon.shape[0]*reshape_factor:
                return reshape_factor
            reshape_factor -= 0.1

    @staticmethod
    def _add_icon(frame, icon, alpha_i, alpha_f, ranges):
        """ 
        Computes the alpha blending of the icon and the frame, and adds the icon to the frame.
        
        Args:
            frame: The frame to which the icon is added.
            icon: The icon to be added.
            alpha_i: The alpha value of the icon.
            alpha_f: The alpha value of the frame.
            ranges: The ranges of the frame where the icon is added.
        """
        h_min, h_max, w_min, w_max = ranges
        for c in range(0, 3):
            frame[h_min:h_max, w_min:w_max, c] = \
            alpha_i * icon[:, :, c] + alpha_f * frame[h_min:h_max, w_min:w_max, c]
            
    def _add_right_legend_icon(self,frame):
        """
        Adds the icons that represent the right hand gestures to the frame.
        
        Args:
            frame: The frame to which the icons are added.
        """
        range= [self.icon_j1_hmin, self.icon_j1_hmax,self.icon_j1_wmin, self.icon_j1_wmax,]
        self._add_icon(frame, self.icon_j1_movement, self.icon_j1_alpha_i, self.icon_j1_alpha_f, range)

        range = [self.icon_j3_hmin, self.icon_j3_hmax, self.icon_j3_wmin, self.icon_j3_wmax]
        self._add_icon(frame, self.icon_j3_movement, self.icon_j3_alpha_i, self.icon_j3_alpha_f, range)
    
    def _add_left_legend_icon(self, frame):
        """
        Adds the icons that represent the left hand gestures to the frame.
        
        Args:
            frame: The frame to which the icons are added.
        """
        range = [self.icon_grip_open_hmin,self.icon_grip_open_hmax, self.icon_grip_open_wmin,self.icon_grip_open_wmax]
        self._add_icon(frame, self.icon_grip_open, self.icon_grip_open_alpha_i, self.icon_grip_open_alpha_f, range)
        
        range = [self.icon_grip_close_hmin,self.icon_grip_close_hmax, self.icon_grip_close_wmin,self.icon_grip_close_wmax]
        self._add_icon(frame, self.icon_grip_close, self.icon_grip_close_alpha_i, self.icon_grip_close_alpha_f, range)
            
    
    def _add_iddle_icon(self, frame):
        """
        Adds the icon that represents the idle state to the frame.
        
        Args:
            frame: The frame to which the icon is added.
        """
        if self.face_detected.is_set():
            self.face_detected.clear()
            icon_face = self.icon_face_green
        else: icon_face = self.icon_face
        range = [self.icon_face_hmin,self.icon_face_hmax, self.icon_face_wmin,self.icon_face_wmax]
        self._add_icon(frame, icon_face, self.icon_face_alpha_i, self.icon_face_alpha_f, range)
        
    
    def run(self):
        """ Creates the window and displays the camera feed."""
        cv.namedWindow("Demo", cv.WND_PROP_FULLSCREEN)
        cv.setWindowProperty("Demo",cv.WND_PROP_FULLSCREEN,cv.WINDOW_FULLSCREEN)
        cv.resizeWindow('Demo', self.screen_dim[0],self.screen_dim[1])
        while not self.is_close:
            
            if self.default_layout != self.fsm.current_state():
                self.default_layout = self.fsm.current_state()
            frame = self.camera.get_frame()
            if frame is None:
                    continue
            if self.default_layout == 'command':
                if self.hand_detected_r.is_set():
                    rect_color = self.rect_hand_detected_color   
                    self.hand_detected_r.clear()
                    finger_r_pos = self.fingers_r.get_positions()
                    if finger_r_pos is None:
                        continue
                    for finger in finger_r_pos.keys():
                        x,y,_ = finger_r_pos[finger]
                        cv.circle(frame, 
                                  center=(int(x) + self.frame_r_coordinates_hd[2], 
                                          int(y) + self.frame_r_coordinates_hd[0]), 
                                  radius=4,
                                  color=self.fingers_tip_color, 
                                  thickness=-1)  
                    #icons set only if right hand is detected
                    self._add_right_hand_icon(frame, finger_r_pos)
                else:
                    rect_color = self.rect_hand_not_detected_color
                cv.rectangle(frame, 
                                (self.frame_r_coordinates[2] , self.frame_r_coordinates[0]), 
                                (self.frame_r_coordinates[3] , self.frame_r_coordinates[1]), 
                                rect_color, 
                                2)
                cv.rectangle(frame, 
                                (self.frame_r_coordinates_hd[2] , self.frame_r_coordinates_hd[0]), 
                                (self.frame_r_coordinates_hd[3] , self.frame_r_coordinates_hd[1]), 
                                self.rect_hand_not_detected_color, 
                                2)
                
                if self.hand_detected_l.is_set():
                    rect_color = self.rect_hand_detected_color
                    self.hand_detected_l.clear()    
                    
                    finger_l_pos = self.fingers_l.get_positions()
                    if finger_l_pos is None:
                        continue
                    for finger in finger_l_pos.keys():
                        x,y,_ = finger_l_pos[finger]
                        cv.circle(frame,
                                  center=(int(x) + self.frame_l_coordinates_hd[2],
                                          int(y)+ self.frame_l_coordinates_hd[0]), 
                                  radius=4, 
                                  color=self.fingers_tip_color, 
                                  thickness=-1) 
                else: 
                    rect_color = self.rect_hand_not_detected_color
                cv.rectangle(frame,
                                (self.frame_l_coordinates[2], self.frame_l_coordinates[0]),
                                (self.frame_l_coordinates[3], self.frame_l_coordinates[1]),
                                rect_color, 
                                2)
                cv.rectangle(frame,
                                (self.frame_l_coordinates_hd[2], self.frame_l_coordinates_hd[0]),
                                (self.frame_l_coordinates_hd[3], self.frame_l_coordinates_hd[1]),
                                self.rect_hand_not_detected_color, 
                                2)
                # icons always set
                self._add_left_legend_icon(frame)
                self._add_right_legend_icon(frame)
            
            if self.default_layout == 'idle':
                self._add_iddle_icon(frame)

            frame = cv.flip(frame, 1)
            
            cv.imshow("Demo", frame)
            cv.pollKey()

    def close(self):
        """ Closes the Gui """
        self.is_close = True
        cv.destroyAllWindows()
        
