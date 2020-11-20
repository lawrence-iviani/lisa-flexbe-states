#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from threading import Lock

from lisa_interaction_msgs.msg import TtsSessionEnded
from lisa_interaction_msgs.srv import UtterService


class LisaUtterState(EventState):
    '''
    Utter a sentence and exit after a wait_time or the end of the uttering action. If context_id is given this will be used to associate the interaction
    to a specific context otherwise an anonymous interaction is started.

    
    -- context_id, 	string 	A idenitifier of a larger dialogue session.
    -- wait_time 	float 	wait time before exit (the end of uttering is not yet implemented, this is a fix timeout). If set to 0 (default) this is a non blocking state (it calls the service and exit before waiting).

    ># text_to_utter 	string 	A text to utter before waiting for intent (default empty text).
    #> error_reason 	string 	An eventual error.

    <= done 			Uttering has finished without any error.
    <= preempt 			Preempted by the user.
    <= timeouted 		Timeout triggered before ending of utterance (this can be used to slow down the state machine while uttering).
    <= error 			An error happend, more details in error_reason

    '''


    def __init__(self, context_id=None, wait_time=0): #, payload_keys):
    # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(LisaUtterState, self).__init__(outcomes = ['done', 'preempt', 'timeouted', 'error'], 
				     			input_keys=['text_to_utter'],
							output_keys=['error_reason']) 

	# Internal state
	self._active = False
	self._service_called = False
	self._tts_ended	= False

	# Store the name of the topic to listen
	self._topic_tts_ended = '/lisa/tts/finished'
	self._sub_tts_ended = None
	
	# servces parameters to be called 
	
	self._context_id = context_id
	self._wait_time = wait_time

	# return values
	self._error_reason = ''

	# waiting for the service to be available
	self._utter_service = '/lisa/service/say'
	rospy.wait_for_service(self._utter_service, timeout=5)

	# Syncronization variable
        self._lock = Lock()


    def execute(self, userdata):
    # This method is called periodically while the state is active.
    # Main purpose is to check state conditions and trigger a corresponding outcome.
    # If no outcome is returned, the state will stay active.
	
	self._lock.acquire()	
	elapsed = rospy.get_rostime() - self._start_time;
	if not self._service_called:
		Logger.logwarn('execute: {} not called, reason: ->{}<-'.format(self._utter_service, self._error_reason))
		self._lock.release()	
		return 'error'
	# !!! self._wait_time is 0, exit immediately!
	if self._wait_time == 0.0 or self._tts_ended:
		Logger.loginfo('execute: done ')
		self._lock.release()	
		return 'done'
	elif (self._wait_time > 0.0 and elapsed.to_sec() > self._wait_time):
		Logger.loginfo('execute: Timeouted after {} s'.format(elapsed.to_sec()))	
		self._lock.release()	
		return 'timeouted'
	self._lock.release()	

    def on_enter(self, userdata):
    # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
	Logger.loginfo('+-+-+-+- ENTERING  LisaUtterState')
	
#	assert hasattr(userdata,  'text_to_utter'), "userdata has not field text_to_utter. userdata ->{}<-".format(userdata)
	self._text = str(userdata.text_to_utter) if hasattr(userdata,  'text_to_utter') else "No text to utter"
	self._start_time = rospy.get_rostime()

	Logger.loginfo('Subscribing topic: {}'.format(self._topic_tts_ended))
	self._sub_tts_ended = rospy.Subscriber(self._topic_tts_ended, TtsSessionEnded, self._tts_ended_received)		

	try:
		utter = rospy.ServiceProxy(self._utter_service, UtterService)
		self._service_called = utter(context_id=self._context_id, text=self._text, canbeenqued=False).success
		Logger.loginfo("Called {} and executed? {}".format(self._utter_service, self._service_called ))
	except rospy.ServiceException as e:
		self._service_called = False
		self._error_reason = str(e)
		Logger.logwarn("Failed call service {} error: {}".format(self._utter_service, e))
	self._lock.acquire()
	self._active = True		
	self._lock.release()

    def _tts_ended_received(self, data):
	self._lock.acquire()
	if not self._active or self._tts_ended:
		Logger.logwarn('Request \n{}\nNot active or already triggered'.format(data))
	# TODO: check context, check if it is what expected
	# if not context == self._context_id:
	Logger.loginfo("TTS ended with data: ".format(data))
	self._tts_ended = True
	self._lock.release()

    def on_exit(self, userdata):
    # This method is called when an outcome is returned and another state gets active.
    # It can be used to stop possibly running processes started by on_enter.
	self._lock.acquire()
	userdata.error_reason = self._error_reason
	self._sub_tts_ended.unregister()
	self._active = False
	self._lock.release()
	

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

