#!/usr/bin/env python3

from control_server_node import *
from astar import AStar

SCENE_PATH = '/home/wtw-ub/workspace/pedsim/src/pedsim_simulator/scenarios/hospital.xml'
flag_direct = True

def ask(args):
    return input(args)

class astar_graph(AStar):

    def __init__(self, waypoints):
        self.graph = []
        for waypoint_index in range(len(waypoints)):
            graph_node = waypoints[waypoint_index]
            graph_node['neighbours'] = []
            for other_waypoint_index in range(len(waypoints)):
                if other_waypoint_index == waypoint_index: continue
                if ((not waypoints[other_waypoint_index]['x'] == waypoints[waypoint_index]['x']) and 
                    (not waypoints[other_waypoint_index]['y'] == waypoints[waypoint_index]['y'])): continue      
                if (('room' in waypoints[waypoint_index]['id']) and 
                    ('entry' not in waypoints[waypoint_index]['id']) and 
                    ('room' in waypoints[other_waypoint_index]['id']) and
                    ('entry' not in waypoints[other_waypoint_index]['id'])): 
                    continue
                graph_node['neighbours'].append(waypoints[other_waypoint_index]['id'])
            self.graph.append(graph_node)
        self.clear_neighbours()
        self.nodes = []
        for dict_node_index in range(len(self.graph)):
            self.nodes.append(dict_node_index)

    def clear_neighbours(self):
        for graph_node in self.graph:
            min_index_x_up = None
            min_index_x_down = None
            min_index_y_up = None
            min_index_y_down = None
            min_dist_x_up = 1000
            min_dist_x_down = 1000
            min_dist_y_up = 1000
            min_dist_y_down = 1000
            for neighbour_id in graph_node['neighbours']:
                neighbour_index = self.node_index_from_id(neighbour_id)
                if (self.graph[neighbour_index]['y'] == graph_node['y']):
                    if (self.graph[neighbour_index]['x'] > graph_node['x']):
                        dist_x = self.graph[neighbour_index]['x'] - graph_node['x']
                        if (min_dist_x_up > dist_x): 
                            min_dist_x_up = dist_x
                            min_index_x_up = neighbour_index
                    elif (self.graph[neighbour_index]['x'] < graph_node['x']):
                        dist_x = graph_node['x'] - self.graph[neighbour_index]['x']
                        if (min_dist_x_down > dist_x): 
                            min_dist_x_down = dist_x
                            min_index_x_down = neighbour_index
                elif (self.graph[neighbour_index]['x'] == graph_node['x']):
                    if (self.graph[neighbour_index]['y'] > graph_node['y']):
                        dist_y = self.graph[neighbour_index]['y'] - graph_node['y']
                        if (min_dist_y_up > dist_y): 
                            min_dist_y_up = dist_y
                            min_index_y_up = neighbour_index
                    elif (self.graph[neighbour_index]['y'] < graph_node['y']):
                        dist_y = -(self.graph[neighbour_index]['y'] - graph_node['y'])
                        if (min_dist_y_down > dist_y): 
                            min_dist_y_down = dist_y
                            min_index_y_down = neighbour_index
            cleared_neigbours = []
            for neighbour_index in range(len(self.graph)):
                if neighbour_index in [min_index_x_up, min_index_x_down, min_index_y_up, min_index_y_down]:
                    cleared_neigbours.append(self.graph[neighbour_index]['id'])
            # print(f'neighbours of {graph_node["id"]}: {cleared_neigbours}')
            graph_node['neighbours'] = cleared_neigbours
            




    def node_index_from_id(self, id):
        for node_index in range(len(self.graph)):
            if self.graph[node_index]['id'] == id: 
                return node_index
            
    def node_from_id(self, id):
        for node_index in range(len(self.graph)):
            if self.graph[node_index]['id'] == id: 
                return self.graph[node_index]

    # original class mistake
    def neighbors(self, node_index):
        for node_id in self.graph[node_index]['neighbours']:
            yield self.node_index_from_id(node_id)
            # yield self.node_from_id(node_id)

    def distance_between(self, node_index_1, node_index_2):
        x1 = self.graph[node_index_1]['x']
        y1 = self.graph[node_index_1]['y']
        x2 = self.graph[node_index_2]['x']
        y2 = self.graph[node_index_2]['y']      
        distance = math.sqrt((x1-x2)**2 + (y1-y2)**2)
        return distance
            
    def heuristic_cost_estimate(self, current, goal):
        return 1
    
    def is_goal_reached(self, current, goal):
        return current == goal

    pass

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
        for wp in waypoints: print(wp)
        print('waypoints loaded')
    except Exception as e:
        print(e)
        print('no waypoints sorry')
    
    astar_graph_ = astar_graph(waypoints)
    
    last_waypoint = 0
    for wp in waypoints:
        if wp['id'] == 'turn_1': 
            last_waypoint = wp
            break

    try:
        while not rospy.is_shutdown():
            print('\ncontrol sender')
            queue_ = []
            queue_names = []
            input_ = ask('enter point or \'click\': ').strip().split(' ')
            if input_[0]=='click':
                print('waiting for point in /clicked_point ...')
                clicked_point = rospy.wait_for_message('/clicked_point', PointStamped)
                print(clicked_point.point)
                print('create goal')
                goal_tmp = MoveBaseGoal()
                goal_tmp.target_pose.pose.position.x = clicked_point.point.x
                goal_tmp.target_pose.pose.position.y = clicked_point.point.y
                goal_tmp.target_pose.pose.position.z = 0
                queue_.insert(0, goal_tmp)
                queue_names.insert(0, 'clicked_point')
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
                    queue_names.insert(0, 'x.y point')
                except Exception:
                    print('trying to interpret as sequence')
                    while len(input_):
                        goal_id = input_.pop(0)
                        for index in range(len(waypoints)):
                            if goal_id == waypoints[index]['id']:
                                print('added goal waypoint: ', waypoints[index]['id'])
                                try:
                                    path = astar_graph_.astar(astar_graph_.node_index_from_id(last_waypoint['id']), astar_graph_.node_index_from_id(waypoints[index]['id']))
                                    print('full path: ')
                                    for node_index in path: 
                                        print(f'\tnode #{node_index}: {astar_graph_.graph[node_index]["id"]}')
                                        goal_tmp = MoveBaseGoal()
                                        goal_tmp.target_pose.pose.position.x = astar_graph_.graph[node_index]['x']
                                        goal_tmp.target_pose.pose.position.y = astar_graph_.graph[node_index]['y']
                                        goal_tmp.target_pose.pose.position.z = 0
                                        queue_.insert(0, goal_tmp)
                                        queue_names.insert(0, astar_graph_.graph[node_index]['id'])
                                    last_waypoint = waypoints[index]
                                    break
                                except Exception as e: 
                                    print(f'no path found: {e}')
                                    print('trying straight line...')
                                    last_waypoint = waypoints[index]
                                    goal_tmp = MoveBaseGoal()
                                    goal_tmp.target_pose.pose.position.x = waypoints[index]['x']
                                    goal_tmp.target_pose.pose.position.y = waypoints[index]['y']
                                    goal_tmp.target_pose.pose.position.z = 0
                                    queue_.insert(0, goal_tmp)
                                    queue_names.insert(0, astar_graph_.graph[index]['id'])
                                    break
                        
            while len(queue_):
                goal_ = queue_.pop()
                goal_name = queue_names.pop()
                action_client_.send_goal(goal_)
                action_client_.wait_for_server() 
                print(f"{goal_name} reached: ", end="", flush=True)
                print(f"{action_client_.wait_for_result()}")
    except rospy.ROSInterruptException or KeyboardInterrupt:
        print('ne rabotaet')   

if __name__ == '__main__':
    main()