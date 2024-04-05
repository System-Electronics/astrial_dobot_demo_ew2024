""" This is the main module of the application and is aimed to launch each thread with dedicated functionality """
import threading
import signal
import sys
import dotenv

from camera_streamer import CameraStreamer
from face_detector import FaceDetector
from common import parse_arguments, init_config_param, get_config_param
from hand_controller import HandController
from metrics_monitor import MetricsMonitor
from SEfsm import Fsm
from gui import Gui 


fsm = None
camera = None
fd = None
cm = None
gui = None
monitor = None

def signal_handler(sig, frame):
    """Handles the interrupt signal given by the user via ctrl+c on the keyboard."""
    global fsm, camera, fd, cm, gui, monitor
    if fsm:
        fsm.close()
    if camera:
        camera.close()
    if fd:
        fd.close()
    if monitor:
        monitor.close()
    if cm:
        cm.close()
    if gui:
        gui.close()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)
dotenv.load_dotenv(dotenv.find_dotenv(filename='hailo.env'))

def main():
    """ 
    Loads the configuration parameters, parses the arguments, and initializes 
    the FSM, camera, face detector, HandController, MetricsMonitor as separate threads.
    Finally it initializes the GUI and runs it.
    """
    global fsm, camera, fd, cm, gui, monitor
    init_config_param()
    args = parse_arguments()
    state_changed = threading.Condition()

    fsm = Fsm(state_changed)
    camera = CameraStreamer(args.video_device)
    if get_config_param('iot_platform_configuration', 'use_kalpa_platform') == 'True':
        monitor = MetricsMonitor(fsm)
    fd = FaceDetector(fsm, camera, model='ultraface_slim_uint8_float32')
    cm = HandController(state_changed, fsm, camera, args.robotic_arm_device, args.relay_device,
                        model='hand_landmark_lite')
    gui = Gui(fsm, camera, cm.hand_detected_r, cm.hand_detected_l, cm.fingers_r, cm.fingers_l, cm.gesture_r,
              fd.face_detected)
    gui.run()

    sys.exit(0)

if __name__ == '__main__':
    main()

