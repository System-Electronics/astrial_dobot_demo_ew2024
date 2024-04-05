import cv2 as cv
import threading
import common

class CameraStreamer:
    """ Manages the camera stream """
    def __init__(self, video_device: str) -> None:
        """ 
        Initializes the camera streamer with the given video device, and starts the thread to capture the frames.
        
        Args:
            video_device: the video device to capture the frames from.
        """
        camera_resolution_w = common.get_config_param('image_configuration','camera_resolution_w')
        camera_resolution_h = common.get_config_param('image_configuration','camera_resolution_h')
        pipeline = 'v4l2src device={} ! image/jpeg, width={}, height={}, \
        framerate=60/1 ! jpegdec! appsink'.format(video_device, camera_resolution_w, camera_resolution_h)
        self.video_device = video_device
        self.cap = cv.VideoCapture(pipeline, cv.CAP_GSTREAMER)
        self.frame = None
        self.thread = threading.Thread(target=self.run)
        self.lock = threading.Lock()
        self.cond = threading.Condition()

        
        self.thread.start()
        
    def health_check(self)-> bool:
        """ 
        Checks if the camera is available or not.
        
        Returns: 
            True if the camera is available, False otherwise.
        """
        if not self.cap.isOpened():
            #print("Cannot open camera {}".format(self.video_device))
            return False
        return True
    
    def run(self):
        """ 
        Captures frames from the camera, updates the frame attribute 
        and notifies the waiting threads that a new frame is available.
        """
        if not self.health_check:
            raise Exception("Camera not available")
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                #print("no frame received from cap {}".format(self.video_device))  
                break
            with self.lock:
                self.frame = frame
                with self.cond:
                    self.cond.notify_all()

            
    def get_frame(self):
        """ 
        Reads the frame attribute and returns a copy of it.
        
        Returns:
            a copy of the frame attribute.
        """
        frame = None
        with self.lock:
            if self.frame is not None:
                frame = self.frame.copy()
        return frame

    def wait_new_frame(self, timeout = 0.4):
        """
        Waits for a new frame to be available.
        
        Args:
            timeout: the time to wait for a new frame to be available (seconds).
            
        Returns: 
            True if a new frame is available, False otherwise.
        """
        with self.cond:
            return self.cond.wait(timeout)


    def close(self):
        """ Closes the camera stream."""
        self.cap.release()
