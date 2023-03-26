#!/usr/bin/env python

import math
import numpy
import rospy
import actionlib
from tf import TransformListener
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Inertia, Twist, Accel, Wrench, Pose, Point
from pedsim_msgs.msg import AgentStates, AgentState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from simple_pid import PID
from sfm_diff_drive.msg import (
    SFMDriveFeedback,
    SFMDriveResult,
    SFMDriveAction,
)

GOAL_REACH_TOLERANCE = .5
POSE_TOLERANCE_DEFAULT = 1
MAX_LINEAR_VEL = .5
MAX_ANGULAR_VEL = 1
RELAXATION_TIME = .5
SAFE_DISTANCE_DEFAULT = 1
OBSTACLE_FORCE_CONST = .8


NODE_NAME = 'sfm_controller_node'
ACTION_NAME = 'sfm_controller_action_node'
COMMAND_TOPIC = '/pedbot/control/cmd_vel'
AGENTS_TOPIC = '/pedsim_simulator/simulated_agents'
ODOMETRY_TOPIC = '/pedsim_simulator/robot_position'
LASER_SCAN_TOPIC = '/scan_filtered'
HUMAN_MASS = 70
ROBOT_MASS = 50
MAX_DIST = 5

hui_znaet = 1
dt = .01
eps = .001

class sfm_controller():
   

    def __init__(self) -> None:

        # goal tmp
        self.current_goal_pose.position.x = 0
        self.current_goal_pose.position.y = 0

        # switches
        self.goal_set = False
        self.loaded = False

        # names
        self.node_name = NODE_NAME
        self.action_name = ACTION_NAME
        self.command_topic = COMMAND_TOPIC
        self.odometry_topic = ODOMETRY_TOPIC
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
        self.pid_rotation = PID(0.25, 0.1, 0.0001, setpoint=0)
        self.pid_rotation.output_limits = (-0.75, 0.75)

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

        # ros things
        self.tf = TransformListener()
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction) #???????
        self.action_server = actionlib.SimpleActionServer(
            self.action_name,
            SFMDriveAction,
            execute_cb=self.callback_sas,
            auto_start=False)
        self.action_server.start()
        rospy.init_node(self.node_name, anonymous=False)
        self.publisher = rospy.Publisher(self.command_topic, Twist, queue_size=10)
        self.subscriber_odom = rospy.Subscriber(self.odometry_topic, Odometry, self.callback_sub_odom)
        self.subscriber_agents = rospy.Subscriber(self.agents_topic, AgentStates, self.callback_sub_agents)
        self.subscriber_scan = rospy.Subscriber(self.scan_topic, LaserScan, self.callback_sub_scan)

        # init
        self.load_change()

    # check if goal is reached
    def goal_reached(self) -> bool:
        pose_na = numpy.array([self.current_pose_pose.position.x, self.current_pose_pose.position.y], numpy.dtype("float64"))
        goal_na = numpy.array([self.current_goal_pose.position.x, self.current_goal_pose.position.y], numpy.dtype("float64"))
        return numpy.linalg.norm(pose_na - goal_na, ord=2) <= GOAL_REACH_TOLERANCE

    # simple action server execution callback
    def callback_sas(self, goal):
        rospy.loginfo("Starting social drive")
        r_sleep = rospy.Rate(30)
        cancel_move_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        cancel_msg = GoalID()
        cancel_move_pub.publish(cancel_msg)

        self.goal_set = True
        self.current_goal_pose.position = Point(goal.goal.x, goal.goal.y, goal.goal.z)
        # self.current_waypoint = np.array(
        #     [goal.goal.x, goal.goal.y, goal.goal.z], np.dtype("float64")
        # )

        while not self.goal_reached():
            complete_force = numpy.array([self.calculate_force().force.x, 
                                          self.calculate_force().force.y, 
                                          self.calculate_force().force.z], 
                                          numpy.dtype("float64"))
            print("complete force:", complete_force)

        #     self.robot_current_vel = self.robot_current_vel + (complete_force / 25)
        #     print("robot current vel:", self.robot_current_vel)

        #     speed = np.linalg.norm(self.robot_current_vel)

        #     if speed > self.robot_max_vel:
        #         self.robot_current_vel = (
        #             self.robot_current_vel
        #             / np.linalg.norm(self.robot_current_vel)
        #             * self.robot_max_vel
        #         )

        #     t = self.tf.getLatestCommonTime("/base_footprint", "/odom")
        #     position, quaternion = self.tf.lookupTransform(
        #         "/base_footprint", "/odom", t
        #     )
        #     robot_offset_angle = tf.transformations.euler_from_quaternion(quaternion)[2]

        #     print("offset_angle:", math.degrees(robot_offset_angle))

        #     angulo_velocidad = angle(
        #         self.robot_current_vel, np.array([1, 0, 0], np.dtype("float64"))
        #     )

        #     print("angulo_velocidad:", math.degrees(angulo_velocidad))

        #     if self.robot_position[1] > self.current_waypoint[1]:
        #         angulo_velocidad = angulo_velocidad - robot_offset_angle
        #     else:
        #         angulo_velocidad = angulo_velocidad + robot_offset_angle

        #     print("angulo final (degree):", math.degrees(angulo_velocidad))

        #     vx = np.linalg.norm(self.robot_current_vel) * math.cos(angulo_velocidad)

        #     # w = np.linalg.norm(self.robot_current_vel) * math.sin(angulo_velocidad)

        #     w = self.pid_rotation(angulo_velocidad)

        #     cmd_vel_msg = Twist()
        #     cmd_vel_msg.linear.x = vx
        #     cmd_vel_msg.angular.z = -w

        #     self.velocity_pub.publish(cmd_vel_msg)

        #     print("v lineal:", vx)
        #     print("w:", w)
        #     print("#####")
        #     self._feedback.feedback = "robot moving"
        #     rospy.loginfo("robot_ moving")
        #     self._as.publish_feedback(self._feedback)
        #     r_sleep.sleep()
        # cmd_vel_msg = Twist()
        # cmd_vel_msg.linear.x = 0
        # cmd_vel_msg.angular.z = 0

        # self.velocity_pub.publish(cmd_vel_msg)
        # self._result = "waypoint reached"
        # rospy.loginfo("waypoint reached")
        # self._as.set_succeeded(self._result)

    # odometry callback
    def callback_sub_odom(self, called_data):
        self.current_pose_pose = called_data.pose.pose
        self.current_pose_covariance = called_data.pose.covariance
        self.current_velocity_twist = called_data.twist.twist

    # agent states callback
    def callback_sub_agents(self, called_data):
        self.current_surroundings_agents = []
        for agent_state in called_data.agent_states:
            if self.is_close(agent_state.pose):                  
                self.current_surroundings_agents.append(agent_state)
    
    # lidar callback
    def callback_sub_scan(self, called_data):
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
        print("social force:", social_force)
        return social_force

        # force = np.array([0, 0, 0], np.dtype("float64"))
        # for i in self.agents_states_register:
        #     diff_position = (
        #         np.array(
        #             [
        #                 i.pose.position.x,
        #                 i.pose.position.y,
        #                 i.pose.position.z,
        #             ],
        #             np.dtype("float64"),
        #         )
        #         - self.robot_position
        #     )
        #     diff_direction = diff_position / np.linalg.norm(diff_position)
        #     agent_velocity = i.twist.linear
        #     diff_vel = self.robot_current_vel - np.array(
        #         [
        #             agent_velocity.x,
        #             agent_velocity.y,
        #             agent_velocity.z,
        #         ],
        #         np.dtype("float64"),
        #     )
        #     interaction_vector = self.lambda_importance * diff_vel + diff_direction
        #     interaction_length = np.linalg.norm(interaction_vector)
        #     interaction_direction = interaction_vector / interaction_length
        #     theta = angle(interaction_direction, diff_direction)
        #     B = self.gamma * interaction_length
        #     force_velocity_amount = -math.exp(
        #         -np.linalg.norm(diff_position) / B
        #         - (self.n_prime * B * theta) * (self.n_prime * B * theta)
        #     )
        #     force_angle_amount = -number_sign(theta) * math.exp(
        #         -np.linalg.norm(diff_position) / B
        #         - (self.n * B * theta) * (self.n * B * theta)
        #     )
        #     force_velocity = force_velocity_amount * interaction_direction
        #     force_angle = force_angle_amount * np.array(
        #         [
        #             -interaction_direction[1],
        #             interaction_direction[0],
        #             0,
        #         ],
        #         np.dtype("float64"),
        #     )
        #     force += force_velocity + force_angle
        #     print("Social force:", force)
        
    # calculate potential force from obstacles - repulse_from_closest
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
            print("minimo:", diff_robot_laser[min_index])
            laser_pos = -1 * numpy.array(
                [
                    self.current_laser_ranges[min_index]
                    * math.cos(math.radians(min_index - 180)),
                    self.current_laser_ranges[min_index]
                    * math.sin(math.radians(min_index - 180)),
                    0,
                ],
                numpy.dtype("float64"),
            )
            # print("laser_pos:", laser_pos)
            laser_vec_norm = numpy.linalg.norm(laser_pos)
            if laser_vec_norm != 0:
                norm_laser_direction = laser_pos / laser_vec_norm
            else:
                norm_laser_direction = numpy.array([0, 0, 0], numpy.dtype("float64"))
            distance = diff_robot_laser[min_index] - self.safe_distance
            force_amount = math.exp(-distance / self.obstacle_force_const)
            obstacle_force = force_amount * norm_laser_direction
            print("Obstacle force:", obstacle_force)
        return obstacle_force

    # calculate total potential force
    def calculate_force(self):

        complete_force = (self.force_factor_desired * self.calculate_goal_force()
                        + self.force_factor_obstacle * self.calculate_obstacle_force()
                        + self.force_factor_social * self.calculate_social_force())

        self.current_force_wrench.force.x = complete_force[0]
        self.current_force_wrench.force.y = complete_force[1]
        self.current_force_wrench.force.z = complete_force[2]

        return complete_force


        # robot_vel = numpy.array([self.current_velocity_twist.linear.x, self.current_velocity_twist.linear.y, 0], numpy.dtype("float64"))
        # robot_vel_norm = numpy.linalg.norm(robot_vel, ord=2)
        # if robot_vel_norm < eps: robot_vel_norm = eps
        # force_tan = numpy.dot(force, robot_vel / robot_vel_norm) * robot_vel / robot_vel_norm
        # force_tan_norm = numpy.linalg.norm(force_tan, ord=2)
        # force_nor = force - force_tan
        # force_nor_norm = numpy.linalg.norm(force_nor, ord=2)
        # self.current_force_wrench.force.x += force_tan_norm
        # self.current_force_wrench.torque.z += force_nor_norm * 1

        print(f'force, torque: {self.current_force_wrench.force.x, self.current_force_wrench.torque.z}')
        return self.current_force_wrench

    def calculate_acceleration(self) -> Accel:
        self.calculate_force()
        self.current_acceleration_accel.linear.x = self.current_force_wrench.force.x / self.inertia.m
        self.current_acceleration_accel.angular.z = self.current_force_wrench.torque.z / self.inertia.izz
        # print(f'acc:\n{self.current_acceleration_accel}')
        return self.current_acceleration_accel 

    def calculate_velocity(self) -> Twist:
        self.calculate_acceleration()
        dv = self.current_acceleration_accel.linear.x * dt
        dw = self.current_acceleration_accel.angular.z * dt
        self.desired_velocity_twist.linear.x = self.current_velocity_twist.linear.x + dv
        self.desired_velocity_twist.angular.z = self.current_velocity_twist.angular.z + dw
        # это тут неправильно сделано надо нормально но потом
        # if self.desired_velocity_twist.linear.x > self.max_linear_vel: self.desired_velocity_twist.linear.x = self.max_linear_vel
        # if self.desired_velocity_twist.angular.z > self.max_angular_vel: self.desired_velocity_twist.angular.z = self.max_angular_vel
        print(f'v: {self.desired_velocity_twist.linear.x}\nw: {self.desired_velocity_twist.angular.z}')
        return self.desired_velocity_twist

    def nasrat_v_chat(self):
        msg_to_publish = self.calculate_velocity()
        self.publisher.publish(msg_to_publish)
   
    def spam(self):
        while not rospy.is_shutdown():
            self.nasrat_v_chat()





def main():
    a = sfm_controller()
    a.spam()

if __name__ == '__main__':
    main()














