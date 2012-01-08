#!/usr/bin/env python
import roslib; roslib.load_manifest('halfsteps_pattern_generator')

import rospy
from geometry_msgs.msg import Point, Pose
from walk_msgs.GetPathClient import Client
import halfsteps_pattern_generator.msg
import halfsteps_pattern_generator.srv

class HalfStepPatternGeneratorClient(Client):
    def __init__(self):
        Client.__init__(
            self,
            halfsteps_pattern_generator.srv.GetPath,
            'getPath')
