#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import copysign

class VoiceController:
    def __init__(self):
        rospy.init_node('voice_control')
        
        rospy.on_shutdown(self.cleanup)
        
        # Define relevant rospy parameters
        self.max_speed = rospy.get_param("~max_speed", 0.4)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.5)
        self.speed_linear = rospy.get_param("~speed_linear", 0.1)
        self.speed_angular = rospy.get_param("~speed_angular", 0.5)
        self.linear_delta = rospy.get_param("~linear_delta", 0.1)
        self.angular_delta = rospy.get_param("~angular_delta", 0.4)
        
        # Set the rate
        self.rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(self.rate)
        
        # A flag to determine whether or not voice control is paused
        self.paused = False
        
        # Initialize output msg.
        self.cmd_vel = Twist()

        # Publish the Twist message to the cmd_vel topic
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Subscribe to get voice data
        rospy.Subscriber('/grammar_data', String, self.speech_callback)
        
        # A mapping from keywords or phrases to commands
        self.keywords_to_command = {'stop': ['stop', 'halt', 'abort', 'kill', 'off', 'shut down', 'turn off'],
                                    'slower': ['slow down', 'slower'],
                                    'faster': ['speed up', 'faster'],
                                    'forward': ['forward', 'ahead', 'straight'],
                                    'backward': ['back', 'backward', 'back up'],
                                    'rotate left': ['rotate left'],
                                    'rotate right': ['rotate right'],
                                    'turn left': ['turn left'],
                                    'turn right': ['turn right'],
                                    'quarter': ['quarter speed'],
                                    'half': ['half speed'],
                                    'full': ['full speed'],
                                    'pause': ['pause voice'],
                                    'continue': ['continue voice']}
        
        rospy.loginfo("[VOICE CONTROL] Ready")
        
        # We have to keep publishing the cmd_vel message if we want the robot to keep moving.
        while not rospy.is_shutdown():
            self.pub.publish(self.cmd_vel)
            r.sleep()                       
            
    def get_command(self, data):
        for (command, keywords) in self.keywords_to_command.iteritems():
            for word in keywords:
                if data.lower().strip() == word:
                    return command
        
    def speech_callback(self, msg):
        # Get the motion command from the recognized phrase
        command = self.get_command(msg.data)
        
        # Log the command to the screen
        rospy.loginfo("[VOICE CONTROL] Command: " + str(command))
        
        #Pause or continue
        if command == 'pause':
            self.paused = True
            rospy.loginfo("[VOICE CONTROL] Paused")
        elif command == 'continue':
            self.paused = False
            rospy.loginfo("[VOICE CONTROL] Continuing")
        
        # If voice control is paused, just return
        if self.paused:
            return       
        
        if command == 'forward':    
            self.cmd_vel.linear.x = self.speed_linear
            self.cmd_vel.angular.z = 0
            
        elif command == 'rotate left':
            self.cmd_vel.linear.x = 0
            self.cmd_vel.angular.z = self.speed_angular
                
        elif command == 'rotate right':  
            self.cmd_vel.linear.x = 0      
            self.cmd_vel.angular.z = -self.speed_angular
            
        elif command == 'turn left':
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.angular.z += self.angular_delta
            else:        
                self.cmd_vel.angular.z = self.speed_angular
                
        elif command == 'turn right':    
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.angular.z -= self.angular_delta
            else:        
                self.cmd_vel.angular.z = -self.speed_angular
                
        elif command == 'backward':
            self.cmd_vel.linear.x = -self.speed_linear
            self.cmd_vel.angular.z = 0
            
        elif command == 'stop': 
            # Stop the robot!  Publish a Twist message consisting of all zeros.         
            self.cmd_vel = Twist()
        
        elif command == 'faster':
            self.speed_linear += self.linear_delta
            self.speed_angular += self.angular_delta
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.linear.x += copysign(self.linear_delta, self.cmd_vel.linear.x)
            if self.cmd_vel.angular.z != 0:
                self.cmd_vel.angular.z += copysign(self.angular_delta, self.cmd_vel.angular.z)
            
        elif command == 'slower':
            self.speed_linear -= self.linear_delta
            self.speed_angular -= self.angular_delta
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.linear.x -= copysign(self.linear_delta, self.cmd_vel.linear.x)
            if self.cmd_vel.angular.z != 0:
                self.cmd_vel.angular.z -= copysign(self.angular_delta, self.cmd_vel.angular.z)
                
        elif command in ['quarter', 'half', 'full']:
            if command == 'quarter':
                self.speed_linear = copysign(self.max_speed / 4, self.speed_linear)
        
            elif command == 'half':
                self.speed_linear = copysign(self.max_speed / 2, self.speed_linear)
            
            elif command == 'full':
                self.speed_linear = copysign(self.max_speed, self.speed_linear)
            
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.linear.x = copysign(self.speed_linear, self.cmd_vel.linear.x)

            if self.cmd_vel.angular.z != 0:
                self.cmd_vel.angular.z = copysign(self.speed_angular, self.cmd_vel.angular.z)
                
        else:
            return

        self.cmd_vel.linear.x = min(self.max_speed, max(-self.max_speed, self.cmd_vel.linear.x))
        self.cmd_vel.angular.z = min(self.max_angular_speed, max(-self.max_angular_speed, self.cmd_vel.angular.z))

    def cleanup(self):
        # When shutting down be sure to stop
        twist = Twist()
        self.pub.publish(twist)
        rospy.sleep(1)

if __name__=="__main__":
    try:
        VoiceController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[VOICE CONTROL] Terminated")