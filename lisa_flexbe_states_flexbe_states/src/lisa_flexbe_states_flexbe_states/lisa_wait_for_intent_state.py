#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from threading import Lock

# from lisa.something .... like ar_conveyor_launch.srv import EnableGrasping, EnableGraspingResponse
from ar_conveyor_launch.srv import EnableGrasping, EnableGraspingResponse
# from lisa_actionlib_msgs.msg import LisaUtterAction, LisaUtterGoal


class LisaWaitForState(EventState):
    '''
    Wait for an intent via a service request before continuing.

    -- intent_name 	string 	The intent  to listen on

    <= continue 			Given time has passed.
    <= preempt 				Example for a failure outcome.

    '''

    def __init__(self, intent_name):
    # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(LisaWaitForState, self).__init__(outcomes = ['continue', 'preempt'], output_keys=['part_type'])

        # Store the name of the service I expect to be called
        self._service_name = intent_name
        self._srv = None

        # internal state variable
        self._active = False
        self._triggered = False

        # Syncronization variable
        self._lock = Lock()

        # return value
        self._part_type = None

    def execute(self, userdata):
    # This method is called periodically while the state is active.
    # Main purpose is to check state conditions and trigger a corresponding outcome.
    # If no outcome is returned, the state will stay active.

        self._lock.acquire()
        if self._triggered:
            self._lock.release()
            # TODO: what is part_type in my case????
            userdata.part_type = self._part_type
            # Logger.loginfo('Executed continue')
            return 'continue'
        self._lock.release()


    def on_enter(self, userdata):
    # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
    # It is primarily used to start actions which are associated with this state.

    # The following code is just for illustrating how the behavior logger works.
    # Text logged by the behavior logger is sent to the operator and displayed in the GUI.
        self._srv = rospy.Service(self._service_name, EnableGrasping, self._on_trigger)
        self._lock.acquire()
        self._active = True
        self._triggered = False
        self._lock.release()

    def _on_trigger(self, request):
        self._part_type = request.part_type
        Logger.loginfo('Triggered, part_type='  + str(request.part_type))
        response = EnableGraspingResponse()
        self._lock.acquire()
        if not self._active or self._triggered:
            response = None
            response.success = False
            Logger.logwarn('triggered should be  active or not yet triggered')
        else:
            self._triggered = True
            response.success = True
            # Logger.loginfo('Triggered, with success, response is ' + str(response))
        self._lock.release()
        return response

    def on_exit(self, userdata):
    # This method is called when an outcome is returned and another state gets active.
    # It can be used to stop possibly running processes started by on_enter.
        Logger.loginfo('On Exit')
        self._lock.acquire()
        self._active = False
        self._lock.release()
        self._srv.shutdown()

    def on_start(self):
    # This method is called when the behavior is started.
    # If possible, it is generally better to initialize used resources in the constructor
    # because if anything failed, the behavior would not even be started.
        pass


    def on_stop(self):
    # This method is called whenever the behavior stops execution, also if it is cancelled.
    # Use this event to clean up things like claimed resources.
        # Nothing to do in this example.
        pass
