#!/usr/bin/env python
import rospy
from flexbe_core import Logger, EventState
# from threading import Lock



class LisaGetPayloadKeyState(EventState):
    '''
    Utter a result from a previous intent recognition block. It is used for debug purposes	.

    
    -- payload_key, 	string 	A idenitifier of a larger dialogue session.


    ># text_to_utter 	string 	A text to utter before waiting for intent (default empty text).
    #> error_reason 	string 	An eventual error.

    <= done 			Key extracted
    <= error 			An error happend, the key was not present or not valid

    '''

    def __init__(self, payload_key):
	# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(LisaGetPayloadKeyState, self).__init__(outcomes = ['done', 'error'],  input_keys=['payload'], output_keys=['payload_value']) 
		self._key = payload_key
		self._retval = None

    def execute(self, userdata):
		userdata.payload_value = self._retval
		if self._retval is None:
			Logger.loginfo('LisaGetPayloadKeyState: Error finding value for key={}'.format(self._key, self._retval))
			return 'error'	
		else:
			Logger.loginfo('LisaGetPayloadKeyState: found key={} with value={}'.format(self._key, self._retval))			
			return 'done'

    def on_enter(self, userdata):
		Logger.loginfo('LisaGetPayloadKeyState: on_enter with  userdata={}, searching for key={}'.format(userdata, self._key,))	
		Logger.loginfo('LisaGetPayloadKeyState: on_enter hasattr(userdata, payload)={} and self._key in userdata.payload={}'.format(hasattr(userdata, 'payload'), self._key in userdata.payload,))	

		if hasattr(userdata, 'payload') and self._key in userdata.payload:
			self._retval = userdata.payload[self._key]
		else:
			self._retval = None


		
	

