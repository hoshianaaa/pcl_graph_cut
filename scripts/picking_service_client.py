#!/usr/bin/env python
import rospy
import time

from std_srvs.srv import Empty

rospy.wait_for_service('/prePick')
prePick = rospy.ServiceProxy('/prePick', Empty)

rospy.wait_for_service('/afterPick')
afterPick = rospy.ServiceProxy('/afterPick', Empty)

while True:
  print("pre pick up")
  prePick()

  time.sleep(0.5)

  print("after pick up")
  afterPick()

  time.sleep(0.5)
