#!/usr/bin/env python
import roslib; roslib.load_manifest('walk_msgs')
import rospy

from geometry_msgs.msg import Pose, Point
import walk_msgs.msg
import walk_msgs.srv

class Client(object):
    client = None
    serviceType = None
    serviceName = None

    initial_left_foot_position = None
    initial_right_foot_position = None
    initial_center_of_mass_position = None

    final_left_foot_position = None
    final_right_foot_position = None
    final_center_of_mass_position = None

    start_with_left_foot = None
    footprints = None

    @staticmethod
    def identityPose():
        pose = Pose()
        pose.position.x = 0.
        pose.position.y = 0.
        pose.position.z = 0.

        pose.orientation.x = 0.
        pose.orientation.y = 0.
        pose.orientation.z = 0.
        pose.orientation.w = 1.
        return pose


    def __init__(self,
                 serviceType = walk_msgs.srv.GetPath,
                 serviceName = 'getPath'):
        self.serviceType = serviceType
        self.serviceName = serviceName
        rospy.wait_for_service(serviceName)
        self.client = rospy.ServiceProxy(serviceName, serviceType)

        self.initial_left_foot_position = self.identityPose()
        self.initial_right_foot_position = self.identityPose()
        self.initial_center_of_mass_position = Point()

        self.final_left_foot_position = self.identityPose()
        self.final_right_foot_position = self.identityPose()
        self.final_center_of_mass_position = Point()


    def __call__(self):
        response = self.client(
            self.initial_left_foot_position,
            self.initial_right_foot_position,
            self.initial_center_of_mass_position,
            self.final_left_foot_position,
            self.final_right_foot_position,
            self.final_center_of_mass_position,
            self.start_with_left_foot,
            self.footprints)
        return response.path
