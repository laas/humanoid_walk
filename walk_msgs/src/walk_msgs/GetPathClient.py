#!/usr/bin/env python
import roslib; roslib.load_manifest('walk_msgs')
import rospy

from geometry_msgs.msg import Pose, Point
import walk_msgs.msg
import walk_msgs.srv

class Client(object):
    """
    Generic GetPath client.

    We make here the assumption that all GetPath services are
    compatible in the sense that they provide /at least/ the fields of
    the walk_msgs.GetPath service message.

    This class can then be inherited to add algorithm specific
    behaviors.
    """

    client = None
    """ROS GetPath service client."""

    serviceType = None
    """ROS service type."""

    serviceName = None
    """Service name."""


    initial_left_foot_position = None
    """Initial left foot position (Pose)."""

    initial_right_foot_position = None
    """Initial right foot position (Pose)."""

    initial_center_of_mass_position = None
    """Initial center of mass position (3d point)."""


    final_left_foot_position = None
    """Final left foot position (Pose)."""
    final_right_foot_position = None
    """Final right foot position (Pose)."""
    final_center_of_mass_position = None
    """Final center of mass position (3d point)."""

    start_with_left_foot = None
    """Which foot is the first flying foot?"""
    footprints = None
    """
    List of footprints.

    The footprint type can vary depending on the used pattern
    generator.  You have to make sure that the footprint type you use
    is the one that the used GetPath service expects.
    """

    @staticmethod
    def identityPose():
        """
        Generate an identity pose.
        """
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
        """
        Initialize the class and create the proxy to call the service.

        This will hang as long as the service is not available.
        """
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
        """
        Call the service and return the computed paths.

        This will trigger the trajectory generation in the generator
        node.
        """
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
