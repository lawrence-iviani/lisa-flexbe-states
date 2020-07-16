#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

# example import of required action
# from chores.msg import DoDishesAction, DoDishesGoal
from lisa_actionlib_msgs.msg import LisaUtterAction, LisaUtterGoal


class LisaUtterActionState(EventState):
	'''
	An actionlib actions to utter a specific sentence
	The example is based on the DoDishes-example of actionlib (see http://wiki.ros.org/actionlib).

	-- sentence string  Sentence to be uttered.

	#> result_message 		string 	operation result.

	<= uttered_all 		the entire string was uttered.
	<= timeout		    A time out occurs during the utterance.
	<= command_error	Cannot send the action goal.

	'''

	def __init__(self, sentence):
		# See example_state.py for basic explanations.
		super(LisaUtterActionState, self).__init__(outcomes = ['uttered_all', 'timeout', 'command_error'],
												   output_keys = ['result_message'])

		# Create the action client when building the behavior.
		# This will cause the behavior to wait for the client before starting execution
		# and will trigger a timeout error if it is not available.
		# Using the proxy client provides asynchronous access to the result and status
		# and makes sure only one client is used, no matter how often this state is used in a behavior.
		# self._topic = 'do_dishes'
		self._topic = '/lisa/say'
		self._client = ProxyActionClient({self._topic: LisaUtterAction}) # pass required clients as dict (topic: type)
		self._sentence = sentence
		# It may happen that the action client fails to send the action goal.
		self._error = False


	def execute(self, userdata):
		# While this state is active, check if the action has been finished and evaluate the result.

		# Check if the client failed to send the goal.
		if self._error:
			return 'command_error'

		if self._client.has_feedback(self._topic):
			Logger.loginfo(self._client.get_feedback(self._topic))

		# Check if the action has been finished
		if self._client.has_result(self._topic):
			result = self._client.get_result(self._topic)
			Logger.loginfo('result: %s' % str(result))
			# dishes_cleaned = result.total_dishes_cleaned

			# In this example, we also provide the amount of cleaned dishes as output key.
			userdata.result_message = 'something i did'
			# TODO: get time for timeot, ros time something
			# Based on the result, decide which outcome to trigger.
			#if dishes_cleaned > self._dishes_to_do:
			#	Logger.loginfo('cleaned_enough ')
			#	return 'cleaned_enough'
			#else:
			#	Logger.loginfo('cleaned_some ')
			#	return 'cleaned_some'
			# Based on the result, decide which outcome to trigger.

			# other possibilities, timeout, error (not recognized command, other?)
			return 'uttered_all'
		# TODO: get time for TIMEOUT, ros time something
		# cancel and timeout

		# Check if the client failed to send the goal.

		# todo: timeout
		#if timeout:
	    #	return 'timeout'

		# If the action has not yet finished, no outcome will be returned and the state stays active.


	def on_enter(self, userdata):
		# When entering this state, we send the action goal once to let the robot start its work.

		# As documented above, we get the specification of which dishwasher to use as input key.
		# This enables a previous state to make this decision during runtime and provide the ID as its own output key.
		# dishwasher_id = userdata.dishwasher

		# Create the goal.
		goal = LisaUtterGoal()
		goal.sentence = self._sentence
		# TODO: get time for timeot, ros time something

		# Send the goal.
		self._error = False # make sure to reset the error state since a previous state execution might have failed
		try:
			Logger.loginfo('Send \ntopic: {}\n{}goal:\n '.format(self._topic, goal))
			self._client.send_goal(self._topic, goal)
		except Exception as e:
			# Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
			# Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
			Logger.logwarn('Failed to send the sentece command:\n%s' % str(e))
			self._error = True


	def on_exit(self, userdata):
		# Make sure that the action is not running when leaving this state.
		# A situation where the action would still be active is for example when the operator manually triggers an outcome.

		if not self._client.has_result(self._topic):
			self._client.cancel(self._topic)
			Logger.loginfo('Cancelled active action goal.')
