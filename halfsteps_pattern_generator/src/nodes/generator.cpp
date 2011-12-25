#include <sstream>
#include <stdexcept>
#include <boost/bind.hpp>
#include <boost/date_time.hpp>

#include <Eigen/LU>

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>

#include <ros/ros.h>

#include <walk_interfaces/yaml.hh>

#include "halfsteps_pattern_generator.hh"

#include "geometry_msgs/Pose.h"
#include "visualization_msgs/MarkerArray.h"
#include "walk_msgs/Footprint2d.h"
#include "walk_msgs/GetPath.h"

#include <walk_msgs/conversion.hh>

#include <walk_msgs/abstract-node.hh>

using walk::HomogeneousMatrix3d;
using walk::Posture;

using walk_msgs::convertPoseToHomogeneousMatrix;
using walk_msgs::convertPointToVector3d;
using walk_msgs::convertTrajectoryToPath;
using walk_msgs::convertTrajectoryV3dToPath;
using walk_msgs::convertTrajectoryV2dToPath;

class GeneratorNode :
  public walk_msgs::AbstractNode<HalfStepsPatternGenerator,
				 walk_msgs::Footprint2d>
{
public:
  explicit GeneratorNode ();
  ~GeneratorNode ();

  virtual void convertFootprint
  (patternGenerator_t::footprints_t& dst,
   const std::vector<footprintRosType_t>& src);

  virtual void
  setupPatternGenerator (walk_msgs::GetPath::Request& req);
};

GeneratorNode::GeneratorNode ()
  : walk_msgs::AbstractNode<HalfStepsPatternGenerator,
			    walk_msgs::Footprint2d> ("", "/world")
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
  std::vector<walk_msgs::Footprint2d>::const_iterator it = src.begin();
  for (; it != src.end(); ++it)
    {
      HalfStepsPatternGenerator::footprint_t footprint;
      footprint.beginTime = (it->beginTime).toBoost();
      footprint.duration =
	seconds(it->duration.sec) + milliseconds(it->duration.nsec * 1000);
      footprint.position(0) = it->x;
      footprint.position(1) = it->y;
      footprint.position(2) = it->theta;
      dst.push_back(footprint);
    }
}

void
GeneratorNode::setupPatternGenerator (walk_msgs::GetPath::Request& req)
{
  //FIXME: fixed sliding for now.
  HalfStepsPatternGenerator::slides_t slides;
  for (unsigned i = 0; i < patternGenerator ().footprints ().size (); ++i)
    slides.push_back (std::make_pair (-0.1, -0.1));
  patternGenerator ().setSlides (slides);
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

