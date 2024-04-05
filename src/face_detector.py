""" This module is responsible for the face detection. It uses the UltraFace model to detect the face in the frame."""
import threading
from ultra_face import UltraFace

class FaceDetector:
    """ Runs the face detection using the UltraFace model and updates the FSM state accordingly."""
    def __init__(self, fsm, camera, model: str = 'ultraface_slim_uint8_float32'):
        """ 
        Initializes the face detector with the given FSM, camera, and model, and starts the thread to run the face detection.
        
        Args:
            fsm: the FSM to update the state.
            camera: the camera to capture the frames from.
            model: the model to use for the face detection.
        """
        self.fsm = fsm
        self.camera = camera
        self.close_thread = False
        self.face_detected = threading.Event()
        self.thread = threading.Thread(name = 'face_detection', target = self.run, args=())

        if model == 'ultraface_slim_uint8_float32':
            self.model = UltraFace(model = model)
        else: 
            raise ValueError('ERROR: model not supported')
        self.thread.start()
        self.face_detection_min_frame = 30 # 1 seconds at 60fps
        
    def run(self):
        """ Calls the face detection and updates the FSM state accordingly."""
        self.fsm.transition() #move to idle
        count_face_detected = 0 
        count_frame = 0
        state = None
        
        while not self.close_thread: 
            self.camera.wait_new_frame()
            count_frame += 1
            if count_frame % 3 == 0:
                image = self.camera.get_frame()
                if image.shape[1] > self.model.input_details[0]['shape'][2] and image.shape[1] % \
                        self.model.input_details[0]['shape'][2] == 0:
                    input_image = image[:, image.shape[1] // 4:3 * image.shape[1] // 4]
                else:
                    input_image = image
                face_detected = self._face_detection(input_image)
                if face_detected:
                    self.face_detected.set() 
                if state is None: 
                    state = face_detected
                count_face_detected = count_face_detected +1 if state == face_detected else 1
                state = face_detected
                if count_face_detected >= self.face_detection_min_frame:
                    self.fsm.transition(face_detected)

    def _face_detection(self, image) -> bool: 
        """ 
        Runs model inference to detect the face in the image and computes the area of the bounding box.
        
        Args:
            image: the image to run the inference on.
            
        Returns:
            True if the face is detected, False otherwise.
        """
        #Check if the face is detected by checking the boxes dimension
        boxes_coordinates = self.model.detect(image)
        for box_coordinate in boxes_coordinates:
            x1, y1, x2, y2 = box_coordinate[0] 
            area = abs(x2-x1)*abs(y2-y1)
            #print(f"face area: {area}") #debug
            if int(area) in range(20000, 70000):
                return True
        return False
    
    def close(self):
        """ Closes the face detector."""
        self.close_thread = True
        self.thread.join()
        
    
