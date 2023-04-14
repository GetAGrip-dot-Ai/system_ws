#!/usr/bin/env python3


import rospy
from system_ws.srv import harvest, harvestResponse
from std_msgs.msg import Int16
import random

class Nodo:
    """node that sends all positive results to test the state machine"""
    def __init__(self):
        # Params
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(30)

        # Publishers
        self.ee_pub = rospy.Publisher("/end_effector/harvest_rsp", Int16,queue_size=10)
        
        # services 
        self.manip_serv = rospy.Service("/manipulation/harvest", harvest, self.give_success)
        self.percep_serv = rospy.Service("/perception/harvest", harvest, self.give_success)

    def give_success(self, req):

        return harvestResponse(reply=1)

    def start(self):
        while not rospy.is_shutdown():

            self.ee_pub.publish(1) # always publish success
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("tester_node")
    my_node = Nodo()
    my_node.start()