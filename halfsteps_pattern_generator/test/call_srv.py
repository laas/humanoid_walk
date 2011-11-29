#!/usr/bin/env python
import roslib; roslib.load_manifest('halfsteps_pattern_generator')

import sys

import rospy
from geometry_msgs.msg import *
from walk_msgs.msg import *
from walk_msgs.srv import *

def get_path_client():
    rospy.wait_for_service('halfsteps_pattern_generator/getPath')
    try:
        get_path = rospy.ServiceProxy(
            'halfsteps_pattern_generator/getPath', GetPath)

        initial_left_foot_position = Pose()
        initial_left_foot_position.position.x = 0.
        initial_left_foot_position.position.y = -0.19
        initial_left_foot_position.position.z = 0.

        initial_left_foot_position.orientation.x = 0.
        initial_left_foot_position.orientation.y = 0.
        initial_left_foot_position.orientation.z = 0.
        initial_left_foot_position.orientation.w = 1.

        initial_right_foot_position = Pose()
        initial_right_foot_position.position.x = 0.
        initial_right_foot_position.position.y = +0.19
        initial_right_foot_position.position.z = 0.

        initial_right_foot_position.orientation.x = 0.
        initial_right_foot_position.orientation.y = 0.
        initial_right_foot_position.orientation.z = 0.
        initial_right_foot_position.orientation.w = 1.


        initial_center_of_mass_position = Point()
        initial_center_of_mass_position.x = 0.
        initial_center_of_mass_position.y = 0.
        initial_center_of_mass_position.z = 0.8

        final_center_of_mass_position = Point()
        final_center_of_mass_position.x = 0.
        final_center_of_mass_position.y = 0.
        final_center_of_mass_position.z = 0.8


        final_left_foot_position = Pose()
        final_left_foot_position.position.x = 3*0.25
        final_left_foot_position.position.y = -0.19
        final_left_foot_position.position.z = 0.

        final_right_foot_position = Pose()
        final_right_foot_position.position.x = 3*0.25
        final_right_foot_position.position.y = +0.19
        final_right_foot_position.position.z = 0.


        start_with_left_foot = True
        footprints = []

        step = Footprint2d()
        step.duration.secs = 0.
        step.duration.nsecs = 5000.
        step.x = 0.*0.25
        step.y = -0.19
        step.theta = 0.
        footprints.append(step)

        step = Footprint2d()
        step.duration.secs = 0.
        step.duration.nsecs = 5000.
        step.x = 0.*0.25
        step.y = +0.19
        step.theta = 0.
        footprints.append(step)

        step = Footprint2d()
        step.duration.secs = 0.
        step.duration.nsecs = 5000.
        step.x = 1*0.25
        step.y = -0.19
        step.theta = 0.
        footprints.append(step)

        step = Footprint2d()
        step.duration.secs = 0.
        step.duration.nsecs = 5000.
        step.x = 1*0.25
        step.y = +0.19
        step.theta = 0.
        footprints.append(step)

        step = Footprint2d()
        step.duration.secs = 0.
        step.duration.nsecs = 5000.
        step.x = 2*0.25
        step.y = -0.19
        step.theta = 0.
        footprints.append(step)

        step = Footprint2d()
        step.duration.secs = 0.
        step.duration.nsecs = 5000.
        step.x = 2*0.25
        step.y = +0.19
        step.theta = 0.
        footprints.append(step)

        step = Footprint2d()
        step.duration.secs = 0.
        step.duration.nsecs = 5000.
        step.x = 3*0.25
        step.y = -0.19
        step.theta = 0.
        footprints.append(step)

        step = Footprint2d()
        step.duration.secs = 0.
        step.duration.nsecs = 5000.
        step.x = 3*0.25
        step.y = +0.19
        step.theta = 0.
        footprints.append(step)


        resp1 = get_path(initial_left_foot_position,
                         initial_right_foot_position,
                         initial_center_of_mass_position,
                         final_left_foot_position,
                         final_right_foot_position,
                         final_center_of_mass_position,
                         start_with_left_foot,
                         footprints)
        return not not resp1.path
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    print "Requesting"
    print get_path_client()
