#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from threading import Lock

from lisa_interaction_msgs.msg import  IntentMessage, IntentNotRecognizedMessage, TtsSessionEnded
from lisa_interaction_msgs.srv import InteractService

# TODO: change name in WaitLisaIntent
class LisaUtterAndWaitForIntentState(EventState):
    '''
    Wait for an intent via a service request before continuing.

    -- intent_name 	string 	The intent  to listen on
    -- text_to_utter 	string 	A text to utter before waiting for intent (default empty text)
    -- context_id, 	string 	A idenitifier of a larger dialogue session
    -- intents		list	A list of string containing intent to listen. Empty list, will listen to all available intents
    -- wait_time 	float 	wait time out in sec (default 0 no timeout)

    <= intent_recognized 	An intent has detected
    <= intent_not_recognized	Speech was detected but no intent associated
    <= preempt 			Example for a failure outcome.
    <= timeouted 		A time out in waiting the intent has happend

    '''
# TODO: add context? Waiting for input snowboy?
# I think i need a library of common functions...
    def __init__(self, intent_name, text_to_utter='', context_id=None, intents = [], wait_time=0): #, payload_keys):
    # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(LisaUtterAndWaitForIntentState, self).__init__(outcomes = ['intent_recognized', 'intent_not_recognized', 'preempt', 'timeouted', 'error'], output_keys=['payload', 'original_sentence', 'error_reason']) 

        # Store the name of the topic to listen
	self._topic_recognized = '/lisa/intent/'
	self._topic_not_recognized = '/lisa/intent/not_recognized'
	self._sub_intent = None # subscriber object

	# local variable
	self._intent_name = intent_name
	self._wait_time = wait_time
	self._context_id = context_id
	self._text = text_to_utter
	self._intents = intents

        # internal state variable
	self._service_called = False
        self._active = False
        self._triggered = False
	self._intent_recognized = False

        # Syncronization variable
        self._lock = Lock()

        # init return value
        self._retval_dict = {}
	self._error_reason = ''
	self._original_sentence = ''

	# waiting for the service to be called
	self._interact_service = '/lisa/service/interact'
	rospy.wait_for_service(self._interact_service, timeout=5)

    def execute(self, userdata):
    # This method is called periodically while the state is active.
    # Main purpose is to check state conditions and trigger a corresponding outcome.
    # If no outcome is returned, the state will stay active.
	if not self._service_called:
		Logger.logwarn('execute: {} not called, reason: ->{}<-'.format(self._interact_service, self._error_reason))
		return 'error'

	self._lock.acquire()
	elapsed = rospy.get_rostime() - self._start_time;
	# !!! self._wait_time is 0, no timeout!
	if (self._wait_time > 0.0 and elapsed.to_sec() > self._wait_time):
		Logger.loginfo('execute: Timeouted after {} s'.format(elapsed.to_sec()))
		self._lock.release()	
		return 'timeouted'

	if self._triggered:
		if self._intent_recognized:
			userdata.payload = self._retval_dict
			userdata.original_sentence = self._original_sentence
			Logger.loginfo('execute: intent recognized, \nuserdata:->{}<-'.format(userdata))
			self._lock.release()			
			return 'intent_recognized'
		else:
			Logger.loginfo('execute: intent not recognized, \nuserdata:->{}<-'.format(userdata))
			self._lock.release()			
			return 'intent_not_recognized'
	self._lock.release()

    def on_enter(self, userdata):
    # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
	Logger.loginfo('+-+-+-+- ENTERING  LisaUtterAndWaitForIntentState')
	# TODO: add utter via calling 
	Logger.loginfo('Subscribing topics: {} and {}'.format(self._topic_recognized, self._topic_not_recognized))
	self._sub_intent = rospy.Subscriber(self._topic_recognized, IntentMessage, self._on_intent_received)		
	self._sub_intent_not_recognized = rospy.Subscriber(self._topic_not_recognized, IntentNotRecognizedMessage, self._on_intent_not_recognized_received)
	
	# initiate 
	self._start_time = rospy.get_rostime()
	try:
		interact = rospy.ServiceProxy(self._interact_service, InteractService)
		self._service_called = interact(context_id=self._context_id, text=self._text, canbeenqued=False, intents=self._intents).success
		Logger.loginfo("Called {} and executed? {}".format(self._interact_service, self._service_called ))		
	except rospy.ServiceException as e:
		self._service_called = False
		self._error_reason = str(e)
		Logger.logwarn("Failed call service {} error: {}".format(self._interact_service, e))

        self._lock.acquire()
        self._active = True
        self._triggered = False
        self._lock.release()

    def _get_payload(self, pay_load):
	# doesnt lock! it expect locking at call level
        if isinstance(pay_load, list):
		Logger.loginfo('Payload is {}'.format(str(pay_load)))
		for value in pay_load:
			k_v = value.split("=")
			if len(k_v) != 2:
				# response.success = False
            			Logger.logwarn('Payload in an unknown form , two tokens was expected find {}, skipping'.format(value, len(k_v)))
				# self._retval_dict = {}
				continue
			self._retval_dict[k_v[0]] = k_v[1]
			# Logger.loginfo('Payload, got {} -> {}' .format(k_v[0], self._retval_dict[k_v[0]]) )
	else:
		assert False
        Logger.loginfo('Decoded payload={}'  + str(self._retval_dict))

    def _on_intent_not_recognized_received(self, data):
	Logger.loginfo('Received intent not recognized'.format( data))
	context = data.context_id	

	# TODO: check context, check if it is what expected
	# if not context == self._context_id:

	self._lock.acquire()
	if not self._active or self._triggered:
		Logger.logwarn('Request \n{}\nNot active or already triggered'.format(data))
	else:
		# string context_id
		# string original_input
		# to retrieve here the value to return, intent,  payload etc.
		self._original_sentence = str(data.original_input) # TODO: temporary solution
		self._triggered = True
		self._intent_recognized = False
		Logger.loginfo('Not intended voacal input :\n->{}<-'.format(self._original_sentence))
	self._lock.release()

    # for topic subscription
    def _on_intent_received(self, data):
        Logger.loginfo('Received intent: {}'.format( data))

	self._lock.acquire()

        context = data.context_id
        original_input = data.original_input
        intent_name = data.intent_name
        confidence = data.confidence
        pay_load = data.pay_load
	# TODO: check context, check if it is what expected
	# if not context == self._context_id:
	# check intent is what expected
	if intent_name != self._intent_name:
		Logger.logwarn('Triggered, intent_name={} but waiting for {}'.format(intent_name, self._intent_name)  )
		self._lock.release()
		return
	if not self._active or self._triggered:
		Logger.logwarn('Request \n{}\nNot active or already triggered'.format(data))
	else:
		# to retrieve here the value to return, intent,  payload etc.
		self._original_sentence = original_input
		self._get_payload(data.pay_load)
		self._triggered = True
		self._intent_recognized = True
		Logger.loginfo('Received message:\n{}'.format(data))
	self._lock.release()

    def on_exit(self, userdata):
    # This method is called when an outcome is returned and another state gets active.
    # It can be used to stop possibly running processes started by on_enter.
	# clean shutdown
        self._sub_intent.unregister()
	self._lock.acquire()
        self._active = False
	userdata.error_reason = self._error_reason
	Logger.loginfo('Exit with userdata ->{}<-'.format(userdata))
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

