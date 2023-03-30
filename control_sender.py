#!/usr/bin/env python

from control_server import *

def ask():
    return input().split(' ')

def main():
    rospy.init_node('control_sender_node', anonymous=False)
    action_client_ = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    try:
        while not rospy.is_shutdown():
            print('\nenter x y: ')
            xy = ask()
            print(xy)
            print('wait for server')
            action_client_.wait_for_server()
            print('create goal')
            goal_tmp = MoveBaseGoal()
            goal_tmp.target_pose.pose.position.x = float(xy[0])
            goal_tmp.target_pose.pose.position.y = float(xy[1])
            goal_tmp.target_pose.pose.position.z = 0
            action_client_.send_goal(goal_tmp)
            print('wait for result')
            print(action_client_.wait_for_result())
            print('go next')
            rospy.sleep(1)
    except rospy.ROSInterruptException or KeyboardInterrupt:
        print('govno ne rabotaet')   

if __name__ == '__main__':
    main()