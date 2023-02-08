#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from gpiozero import Buzzer

def buzzerModuleCallback(data):
    # Если значение 1, тогда запустить пищалку
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def buzzerModuleListener():
    rospy.init_node('buzzer_nodule', anonymous=True)
    rospy.Subscriber("kvantbot/buzzer_module", Float32, buzzerModuleCallback)
    rospy.spin()

if __name__ == '__main__':
    try:
        buzzerModuleListener()
    except rospy.ROSInterruptException:
        pass