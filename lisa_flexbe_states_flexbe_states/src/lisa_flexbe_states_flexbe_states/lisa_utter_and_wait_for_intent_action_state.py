#!/usr/bin/env python
import rospy
from flexbe_core import EventState, Logger
from threading import Lock

# from lisa.something .... like ar_conveyor_launch.srv import EnableGrasping, EnableGraspingResponse
# from ar_conveyor_launch.srv import EnableGrasping, EnableGraspingResponse
# from lisa_actionlib_msgs.msg import LisaUtterAction, LisaUtterGoal
from lisa_actionlib_msgs.srv import  IntentServiceResponse, IntentService
from lisa_interaction_msgs.msg import IntentMessage


# TODO: change name in WaitLisaIntent
class LisaUtterAndWaitForIntentState(EventState):
    '''
    A minimum dialogue block, utter a sentence and wait for an intent to be detected (or timeout if set)

    -- intent_name 	string 	The intent  to listen on
    -- text_to_utter 	string 	A text to utter before waiting for intent (default empty text)
    -- context_id 	string A string to identify the dialogue_context. None will create an anonymous session
    -- wait_time 	float 	wait time out in sec (default 0 no timeout)

    <= intent_recognized 				An intent has detected
    <= intent_not_recognized	Speech was detected but no intent associated
    <= preempt 					Example for a failure outcome.
    <= timeouted 

    '''
# TODO: add context? Waiting for input snowboy?
# I think i need a library of common functions...
    def __init__(self, intent_name, text_to_utter='', context_id=None, wait_time=0): #, payload_keys):
    # Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
        super(LisaUtterAndWaitForIntentState, self).__init__(outcomes = ['intent_recognized', 'intent_not_recognized', 'preempt', 'timeouted', 'error'], output_keys=['payload', 'original_sentence', 'error_reason']) 

        # Store the name of the topic to listen
	self._topic_name = '/lisa/intent/'
	self._sub_intent = None # subscriber object

	# local variable
	self._intent_name = intent_name
	self._wait_time = wait_time

        # internal state variable
        self._active = False
        self._triggered = False

        # Syncronization variable
        self._lock = Lock()

        # init return value
        self._retval_dict = {}

    def execute(self, userdata):
    # This method is called periodically while the state is active.
    # Main purpose is to check state conditions and trigger a corresponding outcome.
    # If no outcome is returned, the state will stay active.

	self._lock.acquire()
	elapsed = rospy.get_rostime() - self._start_time;
	# !!! self._wait_time is 0, no timeout!
	if (self._wait_time > 0.0 and elapsed.to_sec() > self._wait_time):
		self._lock.release()	
		return 'timeouted'

	if self._triggered:
		userdata.payload = self._retval_dict
		Logger.loginfo('Execute done, ret val={}'.format(userdata))
		self._lock.release()			
		return 'continue'
	self._lock.release()

    def on_enter(self, userdata):
    # This method is called when the state becomes active, i.e. a transition from another state to this one is taken.

	# TODO: add utter via calling 
	Logger.loginfo('Subscribing topic ' + self._topic_name)
	self._sub_intent = rospy.Subscriber(self._topic_name, IntentMessage, self._on_intent_received)		

        self._lock.acquire()
        self._start_time = rospy.get_rostime()
        self._active = True
        self._triggered = False
        self._lock.release()

    def _get_payload(self, pay_load):
	# doesnt lock! it expect locking at call level
        if isinstance(pay_load, list):
		Logger.loginfo(str(pay_load))
		for value in pay_load:
			k_v = value.split("=")
			if len(k_v) != 2:
				# response.success = False
            			Logger.logwarn('Payload in an unknown form , two tokens was expected find {}, skipping'.format(value, len(k_v)))
				# self._retval_dict = {}
				continue
			self._retval_dict[k_v[0]] = k_v[1]
			Logger.loginfo('Payload, got {} -> {}' .format(k_v[0], self._retval_dict[k_v[0]]) )
	else:
		assert False
        Logger.loginfo('Triggered, part_type='  + str(self._retval_dict))

    # for topic subscription
    def _on_intent_received(self, data):
        Logger.loginfo('Received intent: {}'.format( data))

        context = data.context_id
        original_input = data.original_input
        intent_name = data.intent_name
        confidence = data.confidence
        pay_load = data.pay_load
	# TODO: check context, chechk if it is what expected

	# check intent is what expected
	if intent_name != self._intent_name:
		Logger.logwarn('Triggered, intent_name={} but waiting for {}'.format(intent_name, self._intent_name)  )
		self._lock.release()
		return
        
        self._lock.acquire()
	# to retrieve here the payload
        self._get_payload(data.pay_load)
        if not self._active or self._triggered:
            Logger.logwarn('Request \n{}\nNot active or already triggered'.format(data))
        else:
            self._triggered = True
            Logger.loginfo('Request \n{}\ntriggered'.format(data))
        self._lock.release()

    # for service only!
    def _on_service_triggered(self, request):
        response = IntentServiceResponse()
        if request.intent_name != self._intent_name:
		Logger.logwarn('Triggered, intent_name={} but waiting for {}'.format(request.intent_name, self._intent_name)  )
		response.success = False
		return response
        self._lock.acquire()
	# to retrieve here the payload
        self._get_payload(request.pay_load)
        if not self._active or self._triggered:
            response.success = False
            Logger.logwarn('Request \n{}\nNot active or already triggered\n{}'.format(request, response))
        else:
            self._triggered = True
            response.success = True
            Logger.loginfo('Request \n{}\ntriggered\n{}'.format(request, response))
        self._lock.release()
        return response

    def on_exit(self, userdata):
    # This method is called when an outcome is returned and another state gets active.
    # It can be used to stop possibly running processes started by on_enter.
        Logger.loginfo('On Exit')
	# clean shutdown
        self._sub_intent.unregister()
	self._lock.acquire()
        self._active = False
	Logger.loginfo('Exit with {}'.format(userdata))
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

