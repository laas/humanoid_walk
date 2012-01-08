#!/usr/bin/env python
import roslib; roslib.load_manifest('halfsteps_pattern_generator')
import rospy
from halfsteps_pattern_generator.GetPathClient \
    import HalfStepPatternGeneratorClient
import halfsteps_pattern_generator.msg

if __name__ == "__main__":
    print "Calling getPath service..."
    try:
        client = HalfStepPatternGeneratorClient()
        client.initial_left_foot_position.position.y = -0.19
        client.initial_right_foot_position.position.y = +0.19
        client.initial_center_of_mass_position.z = 0.8
        client.final_left_foot_position.position.y = -0.19
        client.final_right_foot_position.position.y = +0.19
        client.final_center_of_mass_position.z = 0.8
        client.start_with_left_foot = True
        client.footprints = []

        client.final_left_foot_position.position.x = 3 * 0.25
        client.final_right_foot_position.position.x = 3 * 0.25

        def makeFootprint(x, y):
            footprint = halfsteps_pattern_generator.msg.Footprint()
            footprint.footprint.beginTime.secs = 0
            footprint.footprint.beginTime.nsecs = 0
            footprint.footprint.duration.secs = 0.
            footprint.footprint.duration.nsecs = 1. * 1e9
            footprint.footprint.x = x
            footprint.footprint.y = y
            footprint.footprint.theta = 0.
            footprint.slideUp = -1.52
            footprint.slideDown = -0.76
            footprint.horizontalDistance = 0.38
            footprint.stepHeight = 0.15

            print("{0} {1}".format(footprint.footprint.x,
                                   footprint.footprint.y))
            return footprint


        client.footprints.append(makeFootprint(1 * 0.25, -0.19))
        client.footprints.append(makeFootprint(1 * 0.25, +0.19))

        client.footprints.append(makeFootprint(2 * 0.25, -0.19))
        client.footprints.append(makeFootprint(2 * 0.25, +0.19))

        client.footprints.append(makeFootprint(3 * 0.25, -0.19))
        client.footprints.append(makeFootprint(3 * 0.25, +0.19))

        if not not client():
            print("Service call succeed")
        else:
            print("Service call failed")

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
