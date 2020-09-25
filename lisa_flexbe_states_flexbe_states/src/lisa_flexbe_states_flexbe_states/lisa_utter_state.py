#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger

from lisa_interaction_msgs.srv import UtterService


# TODO: change name in WaitLisaIntent
class LisaUtterState(EventState):
    '''
    Utter a sentence and wait for a fixed time (end of uttering is not yet available)

    -- text_to_utter 	string 	A text to utter before waiting for intent (default empty text)
    -- context_id, 	string 	A idenitifier of a larger dialogue session
    -- wait_time 	float 	wait time before exit (the end of uttering is not yet implemented, this is a fix timeout). If set to 0 (default) this is a non blocking state (it call the service and exit before waiting).

    <= sent 				An intent has detected
    <= preempt 					Example for a failure outcome.
    <= timeouted 					Example for a failure outcome.
    <= error 					Example for a failure outcome.

    '''

    def __init__(self, text_to_utter, context_id=None, wait_time=0): #, payload_keys):
    # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(LisaUtterState, self).__init__(outcomes = ['done', 'preempt', 'timeouted', 'error'], output_keys=['error_reason']) 

	# Internal state
	self._service_called = False

	# servces parameters to be called 
	self._text = text_to_utter
	self._context_id = context_id
	self._wait_time = wait_time

	# return values
	self._error_reason = ''

	# waiting for the service to be called
	self._utter_service = '/lisa/service/say'
	rospy.wait_for_service(self._utter_service, timeout=5)

    def execute(self, userdata):
    # This method is called periodically while the state is active.
    # Main purpose is to check state conditions and trigger a corresponding outcome.
    # If no outcome is returned, the state will stay active.
	elapsed = rospy.get_rostime() - self._start_time;
	if not self._service_called:
		Logger.logwarn('execute: {} not called, reason: ->{}<-'.format(self._utter_service, self._error_reason))
		return 'error'
	# !!! self._wait_time is 0, exit immediately!
	if self._wait_time == 0.0: # (or tts done..)
		Logger.loginfo('execute: done ')
		return 'done'
	elif (self._wait_time > 0.0 and elapsed.to_sec() > self._wait_time):
		Logger.loginfo('execute: Timeouted after {} s'.format(elapsed.to_sec()))
		return 'done' # TODO: at the moment no tts finished information is available. MisUsing timeout for this purpose		
		# return 'timeouted'
	# elif tts finished: return 'done' TODO: to be implemented a topic or similar for waiting a finished tts session

    def on_enter(self, userdata):
    # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
	Logger.loginfo('+-+-+-+- ENTERING  LisaUtterState')
	self._start_time = rospy.get_rostime()
	try:
		utter = rospy.ServiceProxy(self._utter_service, UtterService)
		self._service_called = utter(context_id=self._context_id, text=self._text, canbeenqued=False).success
		Logger.loginfo("Called {} and executed? {}".format(self._utter_service, self._service_called ))
	except rospy.ServiceException as e:
		self._service_called = False
		self._error_reason = str(e)
		Logger.logwarn("Failed call service {} error: {}".format(self._utter_service, e))

	# TODO: add subscribe method for finished uttering
	#Logger.loginfo('Subscribing topic ' + self._topic_name)
	#self._sub_intent = rospy.Subscriber(self._topic_name, IntentMessage, self._on_intent_received)		

    def on_exit(self, userdata):
    # This method is called when an outcome is returned and another state gets active.
    # It can be used to stop possibly running processes started by on_enter.
	userdata.error_reason = self._error_reason

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

