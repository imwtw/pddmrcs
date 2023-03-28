#!/usr/bin/env python

from control_server import *


def main():
    rospy.init_node('NODE_NAME', anonymous=False)
    action_client_ = actionlib.SimpleActionClient('move_base', MoveBaseAction) #???????
    while not rospy.is_shutdown():
        try:
            print('wait for server?')
            action_client_.wait_for_server()
            print('create goal?')
            goal_tmp = MoveBaseGoal()
            goal_tmp.target_pose.pose.position.x = 10
            goal_tmp.target_pose.pose.position.y = 10
            goal_tmp.target_pose.pose.position.z = 0
            action_client_.send_goal(goal_tmp)
            print('wait for result?')
            action_client_.wait_for_result()
            print('result>?')
        except rospy.ROSInterruptException:
            print('govno ne rabotaet')
    pass

if __name__ == '__main__':
    main()