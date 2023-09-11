#!/usr/bin/env python3


from PIL import Image, ImageColor
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import PointCloud, LaserScan
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Pose
from math import *
import numpy
from control_server_node import quaternion_to_euler




class environment_manager():

    # init
    def __init__(self) -> None:
        rospy.init_node('environment_manager_node', anonymous=False)
        print('env manager node init')
        self.laserscan_pub = rospy.Publisher('laserscan_v1', LaserScan, queue_size=10)
        
        # lidar imitation parameters
        self.angle_max = 180
        self.angle_min = -180
        self.angle_increment = 1
        self.min_height = 0
        self.max_height = 2
        self.range_max = 5

        # structures
        self.robot_pose = Pose()
        self.obstacle_pointcloud_raw = PointCloud()
        self.obstacle_pointcloud = PointCloud()
        self.obstacle_laserscan = LaserScan(angle_max=self.angle_max,
                                            angle_min=self.angle_min,
                                            angle_increment=self.angle_increment,
                                            range_max=self.range_max)
  
    # robot pose from tf
    def get_robot_pose(self) -> Pose:
        tf_msg = rospy.wait_for_message('/tf', TFMessage)
        cur_transform = tf_msg.transforms[0].transform
        self.robot_pose = Pose()
        self.robot_pose.position.x = cur_transform.translation.x
        self.robot_pose.position.y = cur_transform.translation.y
        self.robot_pose.position.z = cur_transform.translation.z
        self.robot_pose.orientation = cur_transform.rotation
        return self.robot_pose

    # pointcloud from sim sensor
    def get_obstacle_pointcloud(self) -> PointCloud:
        self.obstacle_pointcloud_raw = rospy.wait_for_message('/pedsim_obstacle_sensor/point_cloud_local', PointCloud)
        self.obstacle_pointcloud.channels = self.obstacle_pointcloud_raw.channels
        self.obstacle_pointcloud.header = self.obstacle_pointcloud_raw.header
        self.obstacle_pointcloud.points.clear()
        reference = self.get_robot_pose()
        for point in self.obstacle_pointcloud_raw.points:
            if point.z!=0 or point.x!=0 or point.y!=0: 
                point.x -= reference.position.x
                point.y -= reference.position.y
                self.obstacle_pointcloud.points.append(point)
        return self.obstacle_pointcloud

    # pointcloud to laserscan (2d ignoring z)
    def pointcloud_to_laserscan(self) -> LaserScan:

        ranges_size = ceil((self.angle_max - self.angle_min) / self.angle_increment)
        self.obstacle_laserscan.ranges.clear()
        for i in range(ranges_size):
            self.obstacle_laserscan.ranges.append(numpy.inf)
        # print(len(self.obstacle_laserscan.ranges))

        min_range_tmp = 10
        min_angle_tmp = 0

        for point in self.obstacle_pointcloud.points:
            # print(point.z)
            if point.z > self.min_height and point.z < self.max_height:
                range_ = hypot(point.x, point.y)
                if range_ > self.obstacle_laserscan.range_max: 
                    continue
                angle_abs = atan2(point.y, point.x)
                angle_ = angle_abs - quaternion_to_euler(self.robot_pose.orientation)[2]
                if angle_ > 2*pi: angle_-=2*pi
                if angle_ < 0: angle_+=2*pi
                angle_index = int(angle_/pi*180)

                if range_ < min_range_tmp: 
                    min_range_tmp = range_
                    min_angle_tmp = angle_index
                
                if self.obstacle_laserscan.ranges[angle_index] > range_:
                    self.obstacle_laserscan.ranges[angle_index] = range_
                # print(angle_index, range_)
        
        # if (min_range_tmp < 10): print('range: ', min_range_tmp, ' angle: ', min_angle_tmp)

        return self.obstacle_laserscan






def main():
    em = environment_manager()
    while not rospy.is_shutdown():
        em.get_obstacle_pointcloud()
        em.pointcloud_to_laserscan()
        em.laserscan_pub.publish(em.obstacle_laserscan)
        rospy.sleep(.1)

if __name__ == '__main__':
    main()