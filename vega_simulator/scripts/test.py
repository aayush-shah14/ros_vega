#!/usr/bin/env python3
import random,argparse,sys
parser = argparse.ArgumentParser()
import rospy
import math
from std_msgs.msg import Float32MultiArray, Float32
from time import sleep
import numpy as np

class Controller:

    def __init__(self,n_):
        self.pub=rospy.Publisher("/actual_system/force",Float32MultiArray,queue_size=1)
        jj=np.zeros(shape=(2,1))
        self.msg=Float32MultiArray()
        self.msg.data=jj
        while not rospy.is_shutdown():
            self.pub.publish(self.msg)
        rospy.spin()


if __name__ == '__main__':
    parser.add_argument("--nsections",type=str)
    args = parser.parse_args()
    print(args.nsections)
    rospy.init_node('Controller_node', anonymous = True)
    s = Controller(int(args.nsections))