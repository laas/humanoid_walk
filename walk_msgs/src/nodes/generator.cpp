#include <sstream>
#include <stdexcept>
#include <boost/bind.hpp>
#include <boost/date_time.hpp>

#include <Eigen/LU>

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>

#include <ros/ros.h>

#include <walk_interfaces/yaml.hh>

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

std::string getParam (const std::string& param,
		      const std::string& defaultValue)
{
  std::string result;
  ros::param::param(param, result, defaultValue);
  return result;
}

class GeneratorYamlNode :
  public walk_msgs::AbstractNode<
  walk::YamlReader<walk::DiscretizedPatternGenerator2d>,
  walk_msgs::Footprint2d,
  walk_msgs::GetPath>
{
public:
  explicit GeneratorYamlNode ();
  ~GeneratorYamlNode ();

  virtual void convertFootprint
  (patternGenerator_t::footprints_t& dst,
   const std::vector<footprintRosType_t>& src);

  virtual void
  setupPatternGenerator (walk_msgs::GetPath::Request& req);
};

GeneratorYamlNode::GeneratorYamlNode ()
  : walk_msgs::AbstractNode<
  walk::YamlReader<walk::DiscretizedPatternGenerator2d>,
  walk_msgs::Footprint2d,
  walk_msgs::GetPath>
    ("", getParam ("~world_frame_id", "/world"),
     walk::YamlReader<walk::DiscretizedPatternGenerator2d>
     (getParam ("~yaml", "")),
     false)
{
  walk_msgs::GetPath::Response res;
  prepareTopicsData (res, patternGenerator().startWithLeftFoot());
  writeMotionAsParameter ();
  ROS_INFO ("motion parsed succesfully, "
	    "starting to publish reference trajectories...");
}

GeneratorYamlNode::~GeneratorYamlNode ()
{}

void
GeneratorYamlNode::convertFootprint (patternGenerator_t::footprints_t& dst,
				 const std::vector<footprintRosType_t>& src)
{
  using boost::posix_time::seconds;
  using boost::posix_time::milliseconds;

  dst.clear();
  std::vector<walk_msgs::Footprint2d>::const_iterator it = src.begin();
  for (; it != src.end(); ++it)
    {
      patternGenerator_t::footprint_t footprint;
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
GeneratorYamlNode::setupPatternGenerator (walk_msgs::GetPath::Request& req)
{
}


int main(int argc, char **argv)
{
  try
    {
      ros::init(argc, argv, "walk_msgs");

      if (getParam ("~yaml", "").empty ())
	{
	  ROS_FATAL_STREAM
	    ("Failed to start node, missing YAML motion.\n"
	     << "Please provide a motion to load:\n"
	     << "rosrun walk_msgs " << argv[0] << " _yaml:=/path/to/motion.yaml");
	  return 1;
	}

      GeneratorYamlNode node;
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

