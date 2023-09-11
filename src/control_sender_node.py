#!/usr/bin/env python3

from control_server_node import *


SCENE_PATH = '/home/wtw-ub/workspace/pedsim/src/pedsim_simulator/scenarios/hospital.xml'
flag_direct = True

def ask(args):
    return input(args)

def main():
    rospy.init_node('control_sender_node', anonymous=False)
    action_client_ = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    waypoints = []
    try:
        scene = open(SCENE_PATH)
        for line in scene.readlines():
            if '<waypoint ' in line:
                id_start = line.find('id=\"') + len('id=\"')
                id_end = line.find('\"', id_start)
                id = line[id_start:id_end]

                x_start = line.find('x=\"') + len('x=\"')
                x_end = line.find('\"', x_start)
                x = float(line[x_start:x_end])

                y_start = line.find('y=\"') + len('y=\"')
                y_end = line.find('\"', y_start)
                y = float(line[y_start:y_end])

                waypoints.append({'id': id,
                                  'x': x,
                                  'y': y})
        # print(waypoints)
        print('waypoints loaded')
    except Exception as e:
        print(e)
        print('no waypoints sorry')
    
    try:
        while not rospy.is_shutdown():
            print('\ncontrol sender')
            queue_ = []

            input_ = ask('enter point or \'click\': ').strip().split(' ')
            if input_[0]=='click':
                clicked_point = rospy.wait_for_message('/clicked_point', PointStamped)
                print(clicked_point.point)
                print('create goal')
                goal_tmp = MoveBaseGoal()
                goal_tmp.target_pose.pose.position.x = clicked_point.point.x
                goal_tmp.target_pose.pose.position.y = clicked_point.point.y
                goal_tmp.target_pose.pose.position.z = 0
                queue_.insert(0, goal_tmp)
            else:
                print('trying to interpret as \'x y\'')
                try:
                    xy = input_
                    print('create goal')
                    goal_tmp = MoveBaseGoal()
                    goal_tmp.target_pose.pose.position.x = float(xy[0])
                    goal_tmp.target_pose.pose.position.y = float(xy[1])
                    goal_tmp.target_pose.pose.position.z = 0
                    queue_.insert(0, goal_tmp)
                except Exception:
                    print('no luck with that') 
                    print('trying to interpret as sequence')
                    while len(input_):
                        goal_id = input_.pop(0)
                        for index in range(len(waypoints)):
                            if goal_id == waypoints[index]['id']:
                                # чек можно ли по прямой. если да то
                                if flag_direct:
                                    print('next waypoint: ', waypoints[index])
                                    print('create goal')
                                    goal_tmp = MoveBaseGoal()
                                    goal_tmp.target_pose.pose.position.x = waypoints[index]['x']
                                    goal_tmp.target_pose.pose.position.y = waypoints[index]['y']
                                    goal_tmp.target_pose.pose.position.z = 0
                                    queue_.insert(0, goal_tmp)
                                else:
                                    # если нет то найти маршрут
                                    # и все запихнуть в очередь
                                    pass
                                break
      
            while len(queue_):
                goal_ = queue_.pop()
                action_client_.send_goal(goal_)
                print('wait for server')
                action_client_.wait_for_server() 
                print('wait for result')
                print(action_client_.wait_for_result())
                print('next')
            # rospy.sleep(1)
    except rospy.ROSInterruptException or KeyboardInterrupt:
        print('ne rabotaet')   

if __name__ == '__main__':
    main()