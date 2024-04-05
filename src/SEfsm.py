"""This module is responsible for the implementation of the 
finite state machine for the application. It is a subclass of the FiniteStateMachineMixin 
class from the fsm module. It defines the states and the transitions between them. """
import fsm

class Fsm(fsm.FiniteStateMachineMixin):
    """ Manages the finite state machine for the application."""
    def __init__(self, state_changed) -> None:
        """
        Initializes the finite state machine with the state changed condition.
        
        Args:
            state_changed: the condition to notify the waiting threads when the state changes.
        """
        self.state_machine = {
            'start': ('idle','stop'),
            'idle': ('idle','command','stop'),
            'command': ('command','idle','stop'),
            'stop': 'stop'
        }

        self.state = 'start'
        self.state_changed = state_changed

    def on_change_state(self, previous_state, next_state, **kwargs):
        """ 
        When the state changes, it notifies the waiting threads.
        
        Args:
            previous_state: the previous state.
            next_state: the next state.
            kwargs: additional arguments.
        """
        if previous_state != next_state:
            with self.state_changed:
                self.state_changed.notify_all()
        return super().on_change_state(previous_state, next_state, **kwargs)
    
    def transition(self, face_detected = None):
        """ 
        Transitions the state machine to the next state based on the face detection result.
        
        Args:
            face_detected: the result of the face detection.
        """
        if self.state == 'stop':
            return
        if face_detected == True: 
            self.change_state("command")
        else:
            self.change_state("idle")

    def wait_change(self, timeout = None):
        """ 
        Waits for the state to change.
        
        Args: 
            timeout: the time to wait for the state to change (seconds).
        """
        with self.state_changed:
            ret = self.state_changed.wait(timeout=timeout)
        return ret

    def close(self):
        """Goes to the stop state and notifies the waiting threads."""
        self.change_state('stop')
        with self.state_changed:
            ret = self.state_changed.wait(timeout=0.5)
        with self.state_changed:
            self.state_changed.notify_all()