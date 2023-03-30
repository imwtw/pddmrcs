#!/usr/bin/env python

import math
import numpy
import rospy
import actionlib
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Inertia, Twist, Accel, Wrench, Pose, Point, Quaternion
from pedsim_msgs.msg import AgentStates, AgentState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from simple_pid import PID


RATE = 40
GOAL_REACH_TOLERANCE = .5
POSE_TOLERANCE_DEFAULT = 1
MAX_LINEAR_VEL = 1
MAX_ANGULAR_VEL = 1
RELAXATION_TIME = .5
SAFE_DISTANCE_DEFAULT = 1
OBSTACLE_FORCE_CONST = .8


NODE_NAME = 'sfm_controller_node'
ACTION_NAME = 'move_base'
COMMAND_TOPIC = '/pedbot/control/cmd_vel'
AGENTS_TOPIC = '/pedsim_simulator/simulated_agents'
ODOMETRY_TOPIC = '/pedsim_simulator/robot_position'
TF_TOPIC = '/tf'
LASER_SCAN_TOPIC = '/scan_filtered'
HUMAN_MASS = 0
ROBOT_MASS = 1
MAX_DIST = 5

hui_znaet = 1

class sfm_controller():
   
    # init
    def __init__(self) -> None:

        # switches
        self.is_ok = True
        self.goal_set = False
        self.loaded = False

        # names
        self.node_name = NODE_NAME
        self.action_name = ACTION_NAME
        self.command_topic = COMMAND_TOPIC
        self.odometry_topic = ODOMETRY_TOPIC
        self.tf_topic = TF_TOPIC
        self.agents_topic = AGENTS_TOPIC
        self.scan_topic = LASER_SCAN_TOPIC

        # internal values
        self.pose_tolerance = POSE_TOLERANCE_DEFAULT
        self.mass = ROBOT_MASS
        self.max_linear_vel = MAX_LINEAR_VEL
        self.max_angular_vel = MAX_ANGULAR_VEL
        self.relaxation_time = RELAXATION_TIME
        self.safe_distance = SAFE_DISTANCE_DEFAULT
        self.obstacle_force_const = OBSTACLE_FORCE_CONST
        self.lambda_importance = 2
        self.gamma = 0.35
        self.n = 2
        self.n_prime = 3
        self.force_factor_desired = 1.0
        self.force_factor_social = 2.1
        self.force_factor_obstacle = 10

        # PID
        # self.angular_velocity_controller = PID(0.25, 0.1, 0.0001, setpoint=0)
        self.angular_velocity_controller = PID(Kp = 0.6, Ki = 0, Kd = 0, setpoint=0, output_limits = (-0.75, 0.75))

        # empty structures
        self.inertia = Inertia()
        self.current_goal_pose = Pose()
        self.current_pose_pose = Pose()
        self.desired_velocity_twist = Twist()
        self.current_velocity_twist = Twist()
        self.current_acceleration_accel = Accel()
        self.current_force_wrench = Wrench()
        self.current_surroundings_agents = []
        self.current_laser_ranges = numpy.zeros(360)

        # goal tmp
        self.current_goal_pose.position.x = 0
        self.current_goal_pose.position.y = 0

        # ros things
        rospy.init_node(self.node_name, anonymous=False, disable_signals=False)
        print('init ros node')
        self.action_server = actionlib.SimpleActionServer(
            self.action_name,
            MoveBaseAction,
            execute_cb=self.callback_sas,
            auto_start=False)
        self.action_server.start()
        self.publisher = rospy.Publisher(self.command_topic, Twist, queue_size=10)
        # self.subscriber_odom = rospy.Subscriber(self.odometry_topic, Odometry, self.callback_sub_odom) #not working
        self.subscriber_tf = rospy.Subscriber(self.tf_topic, TFMessage, self.callback_sub_tf)
        self.subscriber_agents = rospy.Subscriber(self.agents_topic, AgentStates, self.callback_sub_agents)
        self.subscriber_scan = rospy.Subscriber(self.scan_topic, LaserScan, self.callback_sub_scan)

        # init functions
        self.load_change()

    # check if goal is reached
    def goal_reached(self) -> bool:
        # return False
        pose_na = numpy.array([self.current_pose_pose.position.x, self.current_pose_pose.position.y], numpy.dtype("float64"))
        goal_na = numpy.array([self.current_goal_pose.position.x, self.current_goal_pose.position.y], numpy.dtype("float64"))
        return numpy.linalg.norm(pose_na - goal_na, ord=2) <= GOAL_REACH_TOLERANCE

    # simple action server execution callback
    def callback_sas(self, goal):
        print('cb: sas')
        cancel_move_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        cancel_msg = GoalID()
        cancel_move_pub.publish(cancel_msg)
        self.goal_set = True
        self.current_goal_pose = goal.target_pose.pose
        self.run()
        self.action_server.set_succeeded()      
        
    # odometry callback - works incorrect in simulator itself
    def callback_sub_odom(self, called_data):
        # print('cb: odom')
        self.current_pose_pose = called_data.pose.pose
        self.current_pose_covariance = called_data.pose.covariance
        self.current_velocity_twist = called_data.twist.twist

    # tf callback
    def callback_sub_tf(self, called_data):
        # print('cb: tf')
        cur_transform = called_data.transforms[0].transform
        self.current_pose_pose.position.x = cur_transform.translation.x
        self.current_pose_pose.position.y = cur_transform.translation.y
        self.current_pose_pose.position.z = cur_transform.translation.z
        self.current_pose_pose.orientation = cur_transform.rotation

    # agent states callback
    def callback_sub_agents(self, called_data):
        # print('cb: agen')
        self.current_surroundings_agents = []
        for agent_state in called_data.agent_states:
            if self.is_close(agent_state.pose):                  
                self.current_surroundings_agents.append(agent_state)
    
    # lidar callback
    def callback_sub_scan(self, called_data):
        # print('cb: scan')
        self.current_laser_ranges = called_data.ranges

    # check if sth is close to us
    def is_close(self, pose: Pose) -> bool:
        # это только для симулятора, потому что в жизни мы не детектим агентов сквозь стены и все такое
        distance_squared = (self.current_pose_pose.position.x - pose.position.x)**2 + (self.current_pose_pose.position.y - pose.position.y)**2
        return (distance_squared < MAX_DIST**2)

    # call if load is changed (to remake into callback)
    def load_change(self):
        self.calculate_inertia()

    # weigh our load
    def calculate_human_mass(self) -> float:
        # HUMAN_MASS -> var
        if HUMAN_MASS: 
            self.loaded = True
            return HUMAN_MASS
        self.loaded = False
        return 0

    # reassign inertia
    def calculate_inertia(self) -> Inertia:
        self.inertia.m = self.calculate_human_mass() + self.mass
        self.inertia.izz = hui_znaet * self.inertia.m
        # print(f'{self.inertia}')
        return self.inertia
    
    # calculate potential force from goal waypoint - maximum_velocity
    def calculate_goal_force(self):
        goal_dist_x = self.current_goal_pose.position.x - self.current_pose_pose.position.x
        goal_dist_y = self.current_goal_pose.position.y - self.current_pose_pose.position.y
        goal_dist = numpy.array([goal_dist_x, goal_dist_y, 0], numpy.dtype("float64"))
        goal_dist_norm = numpy.linalg.norm(goal_dist, ord=2)
        goal_dist_unit = numpy.array([0,0,0], numpy.dtype("float64"))
        if goal_dist_norm:
            goal_dist_unit = goal_dist / goal_dist_norm
        robot_vel = numpy.array([self.current_velocity_twist.linear.x, self.current_velocity_twist.linear.y, 0], numpy.dtype("float64"))
        robot_vel_norm = numpy.linalg.norm(robot_vel, ord=2)
        time_const = self.relaxation_time / self.inertia.m
        goal_force = self.inertia.m / time_const * (goal_dist_unit * self.max_linear_vel - robot_vel)
        print("desired force:", goal_force)
        return goal_force
        
    # calculate potential force from social agents - inverse_exponential
    def calculate_social_force(self):
        social_force = numpy.array([0,0,0], numpy.dtype("float64"))
        robot_vel = numpy.array([self.current_velocity_twist.linear.x, self.current_velocity_twist.linear.y, 0], numpy.dtype("float64"))
        robot_vel_norm = numpy.linalg.norm(robot_vel, ord=2)
        for agent in self.current_surroundings_agents:
            actual_dist_x = self.current_pose_pose.position.x - agent.pose.position.x
            actual_dist_y = self.current_pose_pose.position.y - agent.pose.position.y
            actual_dist = numpy.array([actual_dist_x, actual_dist_y, 0], numpy.dtype("float64"))
            actual_dist_norm = numpy.linalg.norm(actual_dist, ord=2)
            agent_vel = numpy.array([agent.twist.linear.x, agent.twist.linear.y, 0], numpy.dtype("float64"))
            agent_vel_norm = numpy.linalg.norm(agent_vel, ord=2)
            robot_to_agent_vel_norm = numpy.linalg.norm(numpy.cross(robot_vel, agent_vel), ord=2) / agent_vel_norm
            agent_vel_unit = agent_vel / agent_vel_norm
            agent_vel_perp_unit = numpy.cross(agent_vel_unit, numpy.array([0,0,1], numpy.dtype("float64")))
            robot_to_agent_dir = (-1)**(numpy.dot(actual_dist, robot_vel) > 0)
            robot_to_agent_vel = agent_vel_perp_unit * robot_to_agent_vel_norm * robot_to_agent_dir
            agent_to_robot_vel_x = agent.twist.linear.x - robot_to_agent_vel[0]
            agent_to_robot_vel_y = agent.twist.linear.y - robot_to_agent_vel[1]
            agent_to_robot_vel = numpy.array([agent_to_robot_vel_x, agent_to_robot_vel_y, 0], numpy.dtype("float64"))
            agent_to_robot_vel_norm = numpy.linalg.norm(agent_to_robot_vel, ord=2)
            agent_to_robot_vel_angle_cos = numpy.dot(actual_dist, agent_to_robot_vel) / (actual_dist_norm * agent_to_robot_vel_norm)
            agent_to_robot_vel_unit = agent_to_robot_vel / agent_to_robot_vel_norm
            agent_to_robot_dist = actual_dist_norm * agent_to_robot_vel_angle_cos * agent_to_robot_vel_unit
            collision_time =  numpy.linalg.norm(agent_to_robot_dist / agent_to_robot_vel_norm, ord=2)
            collision_dist = actual_dist - agent_to_robot_dist
            collision_dist_norm = numpy.linalg.norm(collision_dist, ord=2)
            coef_a = 1
            coef_b = 1
            social_force += coef_a * (robot_to_agent_vel_norm * robot_to_agent_dir / collision_time) * math.exp( - collision_dist_norm / coef_b) * (collision_dist / collision_dist_norm)
            # print('_____________________________')
            # print(f'agent {agent.id}')
            # print('_____________________________')
            # print(f'dist: {actual_dist}')
        print("social force: ", social_force)
        return social_force  
        
    # calculate potential force from obstacles - repulse_from_closest_inverse_exponential
    def calculate_obstacle_force(self):

        obstacle_force = numpy.array([0,0,0], numpy.dtype("float64"))
        diff_robot_laser = []
        for i in range(0, 359):
            distance = math.sqrt(
                math.pow(self.current_laser_ranges[i] * math.cos(math.radians(i - 90)), 2) + 
                math.pow(self.current_laser_ranges[i] * math.sin(math.radians(i - 90)), 2)
            )
            diff_robot_laser.append(distance)
        diff_robot_laser = numpy.array(diff_robot_laser, numpy.dtype("float64"))
        for i in range(0, 359):
            if diff_robot_laser[i] == numpy.nan:
                diff_robot_laser[i] = numpy.inf
        min_index = 0
        tmp_val = 1000
        for i in range(0, 359):
            if diff_robot_laser[i] and (diff_robot_laser[i] < tmp_val):
                tmp_val = diff_robot_laser[i]
                min_index = i
        if diff_robot_laser[min_index] < 1:
            # print("wall distance: ", diff_robot_laser[min_index])
            laser_pos = -1 * numpy.array(
                [   self.current_laser_ranges[min_index]
                    * math.cos(math.radians(min_index - 180)),
                    self.current_laser_ranges[min_index]
                    * math.sin(math.radians(min_index - 180)),
                    0
                ], numpy.dtype("float64"))
            # print("laser_pos:", laser_pos)
            laser_vec_norm = numpy.linalg.norm(laser_pos)
            if laser_vec_norm != 0:
                norm_laser_direction = laser_pos / laser_vec_norm
            else:
                norm_laser_direction = numpy.array([0, 0, 0], numpy.dtype("float64"))
            distance = diff_robot_laser[min_index] - self.safe_distance
            force_amount = math.exp(-distance / self.obstacle_force_const)
            obstacle_force = force_amount * norm_laser_direction
            print("obstacle force: ", obstacle_force)
        return obstacle_force

    # calculate total potential force - plane_no_torque
    def calculate_force(self) -> Wrench:
        complete_force = (self.force_factor_desired * self.calculate_goal_force()
                        + self.force_factor_obstacle * self.calculate_obstacle_force()
                        + self.force_factor_social * self.calculate_social_force())
        # complete_force = self.force_factor_social * self.calculate_social_force()
        self.current_force_wrench.force.x = complete_force[0]
        self.current_force_wrench.force.y = complete_force[1]
        self.current_force_wrench.force.z = complete_force[2]
        print("complete force:", complete_force)
        return self.current_force_wrench

    # calculate total acceleration from potential forces - plane_no_angular
    def calculate_acceleration(self) -> Accel:
        self.calculate_force()
        self.current_acceleration_accel.linear.x = self.current_force_wrench.force.x / self.inertia.m
        self.current_acceleration_accel.linear.y = self.current_force_wrench.force.y / self.inertia.m
        self.current_acceleration_accel.angular.z = self.current_force_wrench.torque.z / self.inertia.izz
        # print(f'desired acc: \n{self.current_acceleration_accel.linear}')
        return self.current_acceleration_accel 

    # calculate target velocity in order to satisfy acceleration - curvature_only
    def calculate_velocity(self, *, dt=(RATE**-1)*10) -> Twist:

        # calculate acceleration first
        self.calculate_acceleration()

        # calculate some angles
        self.current_velocity_twist = self.desired_velocity_twist #eto nepravilno
        dvx = self.current_acceleration_accel.linear.x * dt
        dvy = self.current_acceleration_accel.linear.y * dt
        desired_velocity = numpy.array([self.current_velocity_twist.linear.x + dvx, 
                                        self.current_velocity_twist.linear.y + dvy, 
                                        0],numpy.dtype("float64"))
        desired_velocity_norm = numpy.linalg.norm(desired_velocity, ord=2)
        if desired_velocity_norm > self.max_linear_vel:
            desired_velocity = self.max_linear_vel * desired_velocity / desired_velocity_norm
        robot_orientation = self.current_pose_pose.orientation
        robot_angle_offset = quaternion_to_euler(robot_orientation)[2]
        angle_vel_to_base_x = ( (-1)**(self.current_pose_pose.position.y > self.current_goal_pose.position.y) #acos compensation
                                * calculate_v3_angle(desired_velocity, numpy.array([1, 0, 0], numpy.dtype("float64")))) 
        angle_vel_to_my_x = robot_angle_offset - angle_vel_to_base_x
        if abs(angle_vel_to_my_x) > math.pi: angle_vel_to_my_x -= 2*math.pi*numpy.sign(angle_vel_to_my_x)

        # calculate new linear velocity
        v = 0 
        if abs(angle_vel_to_my_x) < (math.pi/2): 
            v = numpy.linalg.norm(desired_velocity, ord=2) * math.cos(angle_vel_to_my_x)

        # calculate new angular velocity (either through acceleration or through curvature or ...)
                        # dw = self.current_acceleration_accel.angular.z * dt
                        # w = self.current_velocity_twist.angular.z + dw
                                        # curvature = ??????
                                        # w = curvature * v
        w = self.angular_velocity_controller(angle_vel_to_my_x)
        
        # form output message
        self.desired_velocity_twist.linear.x = v
        self.desired_velocity_twist.angular.z = w
        print(f'my pos is {self.current_pose_pose.position.x} {self.current_pose_pose.position.y}')
        print(f'desired vel: {desired_velocity}')
        # print(f'desired vel angle to base x: {angle_vel_to_base_x}')
        # print(f'my angle is {robot_angle_offset}')
        # print(f'desired vel angle to my x is {angle_vel_to_my_x}')
        print(f'cmd v: {self.desired_velocity_twist.linear.x}\ncmd w: {self.desired_velocity_twist.angular.z}')
        return self.desired_velocity_twist

    def run(self):
        print('run')
        while (not rospy.is_shutdown()) and self.is_ok:
            if (self.goal_set):
                while (not self.goal_reached()) and self.is_ok:
                    print('\n__________________\n__________________\nstate\n__________________')
                    self.publisher.publish(self.calculate_velocity())
                    print('__________________\n__________________')
                    rospy.sleep(RATE**(-1))
                self.goal_set = False
                print('goal reached')
                self.stop()
                return True
            rospy.sleep(1)
        return False

        print('not run')
        while True:
            self.calculate_social_force()
            rospy.sleep(1)
    
    def stop(self):
        print('\n__________________\n__________________\nstate\n__________________')
        self.calculate_velocity()
        print('__________________\n__________________')
        self.publisher.publish(Twist())


def calculate_v3_angle(v1, v2): 
    return math.acos(numpy.dot(v1, v2) / (numpy.linalg.norm(v1, ord=2) * numpy.linalg.norm(v2, ord=2)))

def quaternion_to_euler(q: Quaternion):
    angles = numpy.array([0,0,0],numpy.dtype("float64"))

    # // roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    angles[0] = math.atan2(sinr_cosp, cosr_cosp)

    # // pitch (y-axis rotation)
    sinp = math.sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
    cosp = math.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
    angles[1] = 2 * math.atan2(sinp, cosp) - math.pi / 2

    # // yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    angles[2] = math.atan2(siny_cosp, cosy_cosp)

    return angles

def main():
    # rospy.init_node(NODE_NAME, anonymous=False, disable_signals=False)
    server = sfm_controller()
    
if __name__ == '__main__':
    main()














