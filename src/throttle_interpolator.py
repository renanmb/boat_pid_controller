#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64
from boat_controller.msg import Drive

# import some utils.
import numpy as np
import copy as copy

# Initial comments: this code is done considering the use in a boat with both steering: differential drive steering and steering by with traditional servo or by rotating the pod drive.  
# With you use differential drive just ignore the servo related topics.
# all parameters are from a parameters server
class InterpolateThrottle:
    def __init__(self):

        # Allow our topics to be dynamic.
        self.rpm_input_topic   = rospy.get_param('~rpm_input_topic', '/vesc/commands/motor/unsmoothed_speed')
        self.rpm_output_left_topic  = rospy.get_param('~rpm_output_left_topic', '/vesc/commands/motor/speed_left')
        self.rpm_output_right_topic  = rospy.get_param('~rpm_output_right_topic', '/vesc/commands/motor/speed_right')
        
        # Boat version with servo steering
        self.servo_input_topic   = rospy.get_param('~servo_input_topic', '/vesc/commands/servo/unsmoothed_position')
        self.servo_output_topic  = rospy.get_param('~servo_output_topic', '/vesc/commands/servo/position')
# ---------------------------------------------------------------------------------------------------
        
        # Motor parameters
        self.max_rpm_acceleration = rospy.get_param('/vesc/max_rpm_acceleration')
        self.max_rpm = rospy.get_param('/vesc/vesc_driver/speed_max')
        self.min_rpm = rospy.get_param('/vesc/vesc_driver/speed_min')
        self.throttle_smoother_rate = rospy.get_param('/vesc/throttle_smoother_rate')
        self.rpm_to_erpm_gain = rospy.get_param('/vesc/rpm_to_erpm_gain')
        # ---------------------------------------------------------------------------------------
#----------------------------------------------------------------------------------------------
        # Boat version with servo steering
        self.max_servo_speed = rospy.get_param('/vesc/max_servo_speed')
        self.steering_angle_to_servo_gain = rospy.get_param('/vesc/steering_angle_to_servo_gain')
        self.servo_smoother_rate = rospy.get_param('/vesc/servo_smoother_rate')
        self.max_servo = rospy.get_param('/vesc/vesc_driver/servo_max')
        self.min_servo = rospy.get_param('/vesc/vesc_driver/servo_min')
# ---------------------------------------------------------------------------------------------------------------
        
        # Variables
        self.last_rpm = 0
        self.desired_rpm = self.last_rpm
        
        self.last_servo = rospy.get_param('/vesc/steering_angle_to_servo_offset')
        self.desired_servo_position = self.last_servo

        # Create topic subscribers and publishers
        self.rpm_output_left = rospy.Publisher(self.rpm_output_left_topic, Float64,queue_size=1)
        self.rpm_output_right = rospy.Publisher(self.rpm_output_right_topic, Float64,queue_size=1)
        self.servo_output = rospy.Publisher(self.servo_output_topic, Float64,queue_size=1)
        
        rospy.Subscriber(self.rpm_input_topic, Drive, self._process_throttle_command)
        rospy.Subscriber(self.servo_input_topic, Float64, self._process_servo_command)
        
# --------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------       
        # limiting calculations
        # Smooth the servo steering ratio, does require adequate servo gain, speed and smoother_rate. May change from project to project
        self.max_delta_servo = abs(self.steering_angle_to_servo_gain * self.max_servo_speed / self.servo_smoother_rate)
        rospy.Timer(rospy.Duration(1.0/self.servo_smoother_rate), self._publish_servo_command)
       
        # Smooth the acceleration curve, does require a rpm_to_erpm_gain and twiling on the throttle_smoother_rate. 
        # throttle_smoother_rate = Hz (messages per second)
        # max_rpm_acceleration = rpm change per second
        #rpm_to_erpm_gain = (number of magnetic poles/2)
        self.max_delta_rpm = abs(self.rpm_to_erpm_gain * self.max_rpm_acceleration / self.throttle_smoother_rate) # Change: speed_to_erpm_gain to rpm_to_erpm_gain, max_acceleration to max_rpm_acceleration (rate of change of rpm) 
        rospy.Timer(rospy.Duration(1.0/self.max_delta_rpm), self._publish_throttle_left_command)
        rospy.Timer(rospy.Duration(1.0/self.max_delta_rpm), self._publish_throttle_right_command)
        # The idea here is that we will have a max delta for the rpm change in the acceleration, the VESC works with ERPM so for example:
        # I want a change of 7(rpm_to_erpm_gain)*100(max_rpm_acceleration)/100(throttle_smoother_rate)= 7 ERPM per message ------------------------ 
        # It means that if I have a topic updating the ERPM every 100 Hz and I want a maximum acceleration of 100 rpm per second I will have a limiting delta of 7 ERPM per each message.
       # ---------------------------------------------------------------------------------------------
        
        # run the node
        self._run()

        # Keep the node alive
    def _run(self):
        rospy.spin()

# ---------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------

# Throttle function --- Publishes and Subscribe the RPM command 
    def _publish_throttle_left_command(self, evt):
        desired_delta = self.desired_rpm_left-self.last_rpm_left
        clipped_delta = max(min(desired_delta, self.max_delta_rpm), -self.max_delta_rpm)
        smoothed_rpm = self.last_rpm_left + clipped_delta
        self.last_rpm_left = smoothed_rpm         
        # print self.desired_rpm, smoothed_rpm
        self.rpm_output_left.publish(Float64(smoothed_rpm))
        
    def _publish_throttle_right_command(self, evt):
        desired_delta = self.desired_rpm_right-self.last_rpm_right
        clipped_delta = max(min(desired_delta, self.max_delta_rpm), -self.max_delta_rpm)
        smoothed_rpm = self.last_rpm_right + clipped_delta
        self.last_rpm_right = smoothed_rpm         
        # print self.desired_rpm, smoothed_rpm
        self.rpm_output_right.publish(Float64(smoothed_rpm))
            
    def _process_throttle_command(self,msg):
        input_rpm_left = msg.data
        input_rpm_right = msg.data
        # Do some sanity clipping
        input_rpm_left = min(max(input_rpm_left, self.min_rpm), self.max_rpm)
        input_rpm_right = min(max(input_rpm_right, self.min_rpm), self.max_rpm)
        self.desired_rpm_left = input_rpm_left
        self.desired_rpm_right = input_rpm_right

# ---------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------

    def _publish_servo_command(self, evt):
        desired_delta = self.desired_servo_position-self.last_servo
        clipped_delta = max(min(desired_delta, self.max_delta_servo), -self.max_delta_servo)
        smoothed_servo = self.last_servo + clipped_delta
        self.last_servo = smoothed_servo         
        self.servo_output.publish(Float64(smoothed_servo))

    def _process_servo_command(self,msg):
        input_servo = msg.data
        # Do some sanity clipping
        input_servo = min(max(input_servo, self.min_servo), self.max_servo)
        # set the target servo position
        self.desired_servo_position = input_servo

# Boilerplate node spin up. 
if __name__ == '__main__':
	rospy.init_node('Throttle_Interpolator')
	try:
        #rospy.init_node('Throttle_Interpolator')
        #p = InterpolateThrottle()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
