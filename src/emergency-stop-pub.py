#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import Joy # for gaming controller emergency stop
from std_msgs.msg import Bool # publishes simple Boolean if emergency stop pressed


class EmergencyStopPub:
    def __init__(self):
        rospy.init_node('emergency_stop_pub')
        self.emergency_stop_button_index = 4
        
        self.emergency_stop_pub = rospy.Publisher('/emergency_stop_topic', Bool, queue_size=1, latch=True)
        self.emergency_stop_pub.publish(Bool(data=False))

        self.emergency_stop_button_sub = rospy.Subscriber('/joy', Joy, self.emergency_stop_callback)
        

    def emergency_stop_callback(self, emergency_stop_msg):
        emergency_button_state = emergency_stop_msg.buttons[self.emergency_stop_button_index]

        emergency_stop_command = Bool(data=bool(emergency_button_state))
        self.emergency_stop_pub.publish(emergency_stop_command)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        emergencyStopPub = EmergencyStopPub()
        emergencyStopPub.run()
    except rospy.ROSInterruptException:
        pass