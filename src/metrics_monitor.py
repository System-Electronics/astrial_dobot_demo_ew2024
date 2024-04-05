""" This module manages the interface with Riseberg and implements the monitoring feature"""
from threading import Timer, Thread
from scheduler_mon_pb2 import ProtoMon
import os
import psutil
import dotenv
from pathlib import Path
from astarte.device import DeviceMqtt
from datetime import datetime, timezone

# find dotenv file from the root of the project
dotenv.load_dotenv(dotenv.find_dotenv(filename='riseberg.env'))
_ROOT_DIR = Path(__file__).parent.absolute()
_INTERFACES_DIR = _ROOT_DIR.joinpath("interfaces")
_DEVICE_ID = os.getenv("RISEBERG_DEVICE_ID")
_REALM = os.getenv("RISEBERG_REALM")
_CREDENTIAL_SECRET = os.getenv("RISEBERG_CREDENTIAL_SECRET")
_PAIRING_URL = os.getenv("RISEBERG_PAIRING_URL")
_PERSISTENCY_DIR = _ROOT_DIR.joinpath("../persistencyData")


class RepeatTimer(Timer):
    """ Manages the sampling of data """
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)


class MetricsMonitor:
    """ Manages the collection of data and their communication via MQTT messaging protocol"""
    def __init__(self, fsm, utilization_time_interval=15) -> None:
        """
        Initializes the communication thread by defining an MQTT device and a timer

        Args:
            utilization_time_interval: sampling time
        """
        self.fsm = fsm
        self.device = DeviceMqtt(
            device_id=_DEVICE_ID,
            realm=_REALM,
            credentials_secret=_CREDENTIAL_SECRET,
            pairing_base_url=_PAIRING_URL,
            persistency_dir=_PERSISTENCY_DIR,
        )
        self.board_metrics = "it.systemelectronics.device.BoardMetrics"
        self.device.add_interfaces_from_dir(_INTERFACES_DIR)
        self.device.connect()
        self.timer = RepeatTimer(utilization_time_interval, self.send_utilization_info)
        self.timer.start()
        self.main_thread_run = True
        self.main_thread = Thread(target=self.thr_send_state, args=())
        self.main_thread.start()


    def get_cpu_utilization(self, interval=1) -> float:
        """ Retrieves the percentage of cpu used so far

         Returns:
             a float value for percentage of cpu used
         """
        return psutil.cpu_percent(interval=interval)

    def get_ram_utilization(self) -> dict:
        """ Retrieves details on ram utilization so far

         Returns:
             a dict with details on ram utilization
         """
        ret = {
            'ramperc': psutil.virtual_memory().percent,
            'ramused': psutil.virtual_memory().used / 1024 / 1024,
            'ramfree': psutil.virtual_memory().free / 1024 / 1024,
            'ramavail': psutil.virtual_memory().available / 1024 / 1024
        }
        return ret

    def get_hailo_utilization(self) -> float:
        """ Retrieves performances on hailo

         Returns:
             a float value for performances on hailo
         """
        curr_pid = os.getpid()
        file = next((f for f in os.listdir('/tmp/hmon_files') if f.startswith(str(curr_pid))), None)
        if file:
            with open(f'/tmp/hmon_files/{file}', 'rb') as file:
                mon = ProtoMon()
                mon.ParseFromString(file.read())
                return mon.device_infos[0].utilization
        return None

    def send_utilization_info(self):
        """ Builds the message with performance data end sends it via MQTT protocol"""
        if self.device.is_connected():
            metrics = {
                "/robot/hailoperc": self.get_hailo_utilization(),
                "/system/cpuperc": self.get_cpu_utilization(),
                "/system/ramperc": self.get_ram_utilization()['ramperc'],
                "/system/ramused": self.get_ram_utilization()['ramused'],
                "/system/ramfree": self.get_ram_utilization()['ramfree'],
                "/system/ramavail": self.get_ram_utilization()['ramavail']
            }

            #now = datetime.now(tz=timezone.utc)

            [self.device.send(self.board_metrics, metric, value) for metric, value in metrics.items() if
             value is not None]

    def send_state(self, state: str):
        """
        Sends the system state via MQTT protocol

        Args:
            state: the current state of fsm
        """
        if self.device.is_connected():
            self.device.send(self.board_metrics, "/robot/robotState", state)

    def thr_send_state(self):
        """ Checks whether the fsm state is changed and sends MQTT messages accordingly """
        last_state = None
        while self.main_thread_run:
            self.fsm.wait_change(timeout=5)
            state = self.fsm.current_state()
            if state != last_state:
                self.send_state(state)
                last_state = state
            if self.fsm.current_state() == 'stop':
                self.main_thread_run = False

    def close(self):
        """ Closes the metrics monitor"""
        self.timer.cancel()
        self.main_thread_run = False
        self.main_thread.join()
        self.device.disconnect()
