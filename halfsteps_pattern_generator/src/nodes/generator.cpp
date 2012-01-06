#include <sstream>
#include <stdexcept>
#include <boost/bind.hpp>
#include <boost/date_time.hpp>

#include <Eigen/LU>

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>

#include <ros/ros.h>

#include "halfsteps_pattern_generator.hh"
#include "yaml.hh"

#include "geometry_msgs/Pose.h"
#include "visualization_msgs/MarkerArray.h"
#include "halfsteps_pattern_generator/GetPath.h"
#include "halfsteps_pattern_generator/Footprint.h"

#include <walk_msgs/conversion.hh>

#include <walk_msgs/abstract-node.hh>

using walk::HomogeneousMatrix3d;
using walk::Posture;

using walk_msgs::convertPoseToHomogeneousMatrix;
using walk_msgs::convertPointToVector3d;
using walk_msgs::convertTrajectoryToPath;
using walk_msgs::convertTrajectoryV3dToPath;
using walk_msgs::convertTrajectoryV2dToPath;

template <typename T>
T getParam (const std::string& param,
	    const T& defaultValue)
{
  T result;
  ros::param::param(param, result, defaultValue);
  return result;
}

class GeneratorNode :
  public walk_msgs::AbstractNode<HalfStepsPatternGenerator,
				 halfsteps_pattern_generator::Footprint,
				 halfsteps_pattern_generator::GetPath>
{
public:
  explicit GeneratorNode ();
  ~GeneratorNode ();

  virtual void convertFootprint
  (patternGenerator_t::footprints_t& dst,
   const std::vector<footprintRosType_t>& src);

  virtual void
  setupPatternGenerator (serviceRosType_t::Request& req);
};

GeneratorNode::GeneratorNode ()
  : walk_msgs::AbstractNode<HalfStepsPatternGenerator,
			    halfsteps_pattern_generator::Footprint,
			    halfsteps_pattern_generator::GetPath>
    ("", getParam<std::string> ("~world_frame_id", "/world"),
     HalfStepsPatternGenerator
     (getParam ("~time_before_zmp_shift", 0.95),
      getParam ("~time_after_zmp_shift", 1.05),
      getParam ("~step", 0.005)
      ))
{}

GeneratorNode::~GeneratorNode ()
{}

void
GeneratorNode::convertFootprint (patternGenerator_t::footprints_t& dst,
				 const std::vector<footprintRosType_t>& src)
{
  using boost::posix_time::seconds;
  using boost::posix_time::milliseconds;

  dst.clear();
  std::vector<footprintRosType_t>::const_iterator it = src.begin();
  for (; it != src.end(); ++it)
    {
      // Check that footprint is valid.
      if (it->slideUp < -1.52 || it->slideUp > 0.)
	throw std::runtime_error ("invalid up slide");
      if (it->slideDown < -0.76 || it->slideDown > 0.)
	throw std::runtime_error ("invalid down slide");

      // Add it to the pattern generator.
      HalfStepsPatternGenerator::footprint_t footprint;
      footprint.beginTime = (it->footprint.beginTime).toBoost();
      footprint.duration =
	seconds(it->footprint.duration.sec)
	+ milliseconds(it->footprint.duration.nsec * 1000);
      footprint.position(0) = it->footprint.x;
      footprint.position(1) = it->footprint.y;
      footprint.position(2) = it->footprint.theta;
      footprint.slideUp = it->slideUp;
      footprint.slideDown = it->slideDown;
      footprint.horizontalDistance = it->horizontalDistance;
      footprint.stepHeight = it->stepHeight;
      dst.push_back(footprint);
    }
}

void
GeneratorNode::setupPatternGenerator (serviceRosType_t::Request& req)
{
}


int main(int argc, char **argv)
{
  try
    {
      ros::init(argc, argv, "halfsteps_pattern_generator");
      GeneratorNode node;
      if (ros::ok())
	node.spin();
    }
  catch (std::exception& e)
    {
      std::cerr << "fatal error: " << e.what() << std::endl;
      ROS_ERROR_STREAM("fatal error: " << e.what());
      return 1;
    }
  catch (...)
    {
      ROS_ERROR_STREAM("unexpected error");
      return 2;
    }
  return 0;
}

