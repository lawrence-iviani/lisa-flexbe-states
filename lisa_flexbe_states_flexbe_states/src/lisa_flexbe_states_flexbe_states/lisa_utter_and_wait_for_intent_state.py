#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from threading import Lock

from lisa_interaction_msgs.msg import  IntentMessage, IntentNotRecognizedMessage, TtsSessionEnded
from lisa_interaction_msgs.srv import InteractService

try:
    from lisa_mqtt_ros_bridge.dialogue import ANONYMOUS_PREFIX, WAKEUPWORD_PREFIX
except Exception as e:
    ANONYMOUS_PREFIX = "Anonymous_" # This must be the same as defined in lisa_mqtt_ros_bridge.dialogue
    WAKEUPWORD_PREFIX = "Wakeupword_"


class LisaUtterAndWaitForIntentState(EventState):
    '''
    Wait for an intent via a service request before continuing, if a timeout is specified will exit after the wait_time (sec.)
    Intents recognized or not are emitted via a ROS topic. If context_id is given this will be used to associate the interaction
    to a specific context, otherwise an anonymous interaction is started.

    
    -- context_id, 	string 	A idenitifier of a larger dialogue session.
    -- intents		list	A list of string containing intent to listen. Empty list, will listen to all available intents.
    -- wait_time 	float 	wait time out in sec (default 0 no timeout and wait indefinitely). This include also the uttering time (text_to_utter is not empty).

    ># text_to_utter 	string 	A text to utter before waiting for intent (default empty text).

    #> payload 			dict 	The payload of the intent in the form of a dictonary (key/value) where the value is always a string.
    #> original_sentence 	string 	The original sentence.
    #> error_reason 		string 	An eventual error as string.
    #> intent_recognized 	string 	The intent recognized if any
   

    <= intent_recognized 	An intent has detected (payload if any and original_sentence are returned).
    <= intent_not_recognized	Speech was detected but no intent associated (original_sentence is returned).
    <= preempt 			Preempted by the user.
    <= timeouted 		A time out in waiting the intent has happend.
    <= error 			An error happend, more details in error_reason.

    '''
# -- 
    def __init__(self, context_id=None, 
                       intents = [], 
                       wait_time=0): 
        # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(LisaUtterAndWaitForIntentState, self).__init__(outcomes = ['intent_recognized', 'intent_not_recognized', 'preempt', 'timeouted', 'error'],
							     input_keys=['text_to_utter'],
 							     output_keys=['payload', 'original_sentence', 'error_reason', 'intent_recognized']) 

        # Store the name of the topic to listen
	self._topic_recognized = '/lisa/intent/'
	self._topic_not_recognized = '/lisa/intent/not_recognized'
	self._sub_intent = None # subscriber object
	self._sub_intent_not_recognized = None

	# local variable
	self._wait_time = wait_time
	self._context_id = context_id
	
	assert isinstance(intents, list), "Intents must be a list of string, instead of {}. Intents: {}".format(intents.__class__, intents)
	self._intents = intents

        # internal state variable
	self._service_called = False
        self._active = False
        self._triggered = False
	self._intent_recognized = None

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
			userdata.intent_recognized = self._intent_recognized
			Logger.loginfo('execute: intent recognized, \nuserdata:->{}<-'.format(userdata))
			self._lock.release()			
			return 'intent_recognized'
		else:
			userdata.payload = self._retval_dict
			userdata.original_sentence = self._original_sentence
			userdata.intent_recognized = None
			Logger.loginfo('execute: intent not recognized, \nuserdata:->{}<-'.format(userdata))
			self._lock.release()			
			return 'intent_not_recognized'
	self._lock.release()

    def on_enter(self, userdata):
    # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
	Logger.loginfo('+-+-+-+- ENTERING  LisaUtterAndWaitForIntentState')
	assert isinstance(userdata.text_to_utter,  str)
	self._text = userdata.text_to_utter

	Logger.loginfo('Subscribing topics: {} and {}'.format(self._topic_recognized, self._topic_not_recognized))
	self._sub_intent = rospy.Subscriber(self._topic_recognized, IntentMessage, self._on_intent_received)		
	self._sub_intent_not_recognized = rospy.Subscriber(self._topic_not_recognized, IntentNotRecognizedMessage, self._on_intent_not_recognized_received)
	
	# initiate 
	self._start_time = rospy.get_rostime()
	try:
		interact = rospy.ServiceProxy(self._interact_service, InteractService)
		# with call will produce the follow behaviour:
		# - remove an eventual on going session(canbediscarded=False) and 
		# - start immediately(canbeenqued=False)
		self._service_called = interact(context_id=self._context_id, text=self._text, canbeenqued=False, canbediscarded=False, intents=self._intents).success
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
	# Receive a payload in the format attribute_name=value 
	# return a dict in the form {'attribute_name': str(value)}
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

	self._lock.acquire()
	context = data.context_id	
	Logger.loginfo('Received intent not recognized: data: ->{}<-'.format(data))

	if not self._active or self._triggered:
		Logger.logwarn('Request \n{}\nNot active or already triggered'.format(data))
	elif not self._is_right_context(context):
		Logger.logwarn('Received {} as context!!!!!!!!, expected {}'.format(context, self._context_id))	
	else:
		self._original_sentence = str(data.original_input) 
		self._triggered = True
		self._intent_recognized = None
		Logger.loginfo('input :\n->{}<- is not recognized as valid or expected intent'.format(self._original_sentence))
	self._lock.release()

    def _is_right_context(self, context):
	Logger.logdebug('context={}  self._context_id={}'.format(context, self._context_id))
	# accept all the follow contexts
	if self._context_id is None:
		return True 
	elif (context == self._context_id):
		return True
	elif context.startswith(ANONYMOUS_PREFIX): 
		return True	
	elif context.startswith(WAKEUPWORD_PREFIX):
		return True
	else:
		return False

    # for topic subscription
    def _on_intent_received(self, data):
        Logger.loginfo('Received intent: {}'.format( data))

	self._lock.acquire()

        context = data.context_id
        original_input = data.original_input
        intent_name = data.intent_name
        confidence = data.confidence
        pay_load = data.pay_load

	if not self._active or self._triggered:
		Logger.logwarn('Request \n{}\nNot active or already triggered'.format(data))
	elif not self._is_right_context(context):
		Logger.logwarn('Received {} as context!!!!!!!!, expected {}'.format(context, self._context_id))
	# check intent is what expected, if no filter(s) ignore the check
	elif len(self._intents) and intent_name not in self._intents:
		Logger.logwarn('Triggered, intent_name={} but waiting for {}'.format(intent_name, self._intents)  )
	else:
		# to retrieve here the value to return, intent,  payload etc.
		self._original_sentence = original_input
		self._get_payload(data.pay_load)
		self._triggered = True
		self._intent_recognized = intent_name
		Logger.loginfo('Received message:\n{}'.format(data))
	self._lock.release()

    def on_exit(self, userdata):
    # This method is called when an outcome is returned and another state gets active.
    # It can be used to stop possibly running processes started by on_enter.
	# clean shutdown
        self._sub_intent.unregister()
	self._sub_intent_not_recognized.unregister()
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

