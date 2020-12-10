#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from lisa_interaction_msgs.msg import LisaUtterAction, LisaUtterGoal


class LisaUtterActionStateWithUserkey(EventState):
	'''
	An uttering action is performed with high priority (all running dialogue are dropped immediately), this has to be used 
	to perform an urgent prioritary announcement, possibly out of the context.

    	-- wait_time 		float 	wait time before exit (the end of uttering is not yet implemented, this is a fix timeout). If set to 0 (default) exit when the uttering is finished.

	#> text_to_utter	string  Sentence to be uttered.
	#> error_reason 	string 	An eventual error.

	<= uttered_all 		the entire string was uttered.
	<= timeout		A time out occurs during the utterance.
	<= error 		An error happend, more details in error_reason
	'''

	def __init__(self, wait_time=0):
		# See example_state.py for basic explanations.
		super(LisaUtterActionStateWithUserkey, self).__init__(outcomes = ['uttered_all', 'timeout', 'command_error'], 
								input_keys = ['text_to_utter'],
								output_keys = ['error_reason'])
		self._topic = '/lisa/say'
		self._client = ProxyActionClient({self._topic: LisaUtterAction}) # pass required clients as dict (topic: type)
		# It may happen that the action client fails to send the action goal.
		self._error = False
		self._error_reason = ''
		self._wait_time = wait_time
		if wait_time:
			Logger.logwarn('time out not yet implemented for LisaUtterActionState')


	def execute(self, userdata):
		# While this state is active, check if the action has been finished and evaluate the result.
		# Check if the client failed to send the goal.
		if self._error:
			Logger.logwarn('command_error. error_reason: ' + str(self._error_reason))
			userdata.error_reason = self._error_reason
			return 'command_error'
		if self._client.has_feedback(self._topic):
			pass #Logger.logdebug('Progress {}'.format(self._client.get_feedback(self._topic)))

		# Check if the action has been finished
		if self._client.has_result(self._topic):
			result = self._client.get_result(self._topic)
			userdata.error_reason = ''#'uttered_all: result: %s' % str(result)			
			Logger.loginfo('uttered_all: result: %s' % str(result))		
			return 'uttered_all'
		# TODO: get time for TIMEOUT, ros time something
		# cancel and timeout

		# Check if the client failed to send the goal.

		# todo: timeout
		#if timeout:
	    #	return 'timeout'

		# If the action has not yet finished, no outcome will be returned and the state stays active.


	def on_enter(self, userdata):
		self._sentence = userdata.text_to_utter
		# Create the goal.
		goal = LisaUtterGoal()
		goal.sentence = self._sentence
		# TODO: get time for timeot, ros time something

		# Send the goal.
		self._error = False # make sure to reset the error state since a previous state execution might have failed
		try:
			Logger.loginfo('Send \ntopic: {}\ngoal:{}\n '.format(self._topic, goal))
			self._client.send_goal(self._topic, goal)
		except Exception as e:
			# Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
			# Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
			self._error_reason = 'Failed to send the sentece command:\n%s' % str(e)			
			Logger.logwarn(self._error_reason)			
			self._error = True


	def on_exit(self, userdata):
		# Make sure that the action is not running when leaving this state.
		# A situation where the action would still be active is for example when the operator manually triggers an outcome.

		if not self._client.has_result(self._topic):
			self._client.cancel(self._topic)
			self._error_reason = 'Cancelled active action goal.'
			Logger.loginfo(self._error_reason)
			
