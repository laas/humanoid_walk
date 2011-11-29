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

void convertFootprint(HalfStepsPatternGenerator::footsteps_t& dst,
		      const std::vector<walk_msgs::Footprint2d>& src);

void convertFootprint(HalfStepsPatternGenerator::footsteps_t& dst,
		      const std::vector<walk_msgs::Footprint2d>& src)
{
  using boost::posix_time::seconds;
  using boost::posix_time::milliseconds;

  dst.clear();
  std::vector<walk_msgs::Footprint2d>::const_iterator it = src.begin();
  for (; it != src.end(); ++it)
    {
      HalfStepsPatternGenerator::footstep_t step;
      step.duration =
	seconds(it->duration.sec) + milliseconds(it->duration.nsec * 1000);
      step.position(0) = it->x;
      step.position(1) = it->y;
      step.position(2) = it->theta;
      dst.push_back(step);
    }
}


using walk::HomogeneousMatrix3d;
using walk::Posture;

using walk_msgs::convertPoseToHomogeneousMatrix;
using walk_msgs::convertPointToVector3d;
using walk_msgs::convertTrajectoryToPath;
using walk_msgs::convertTrajectoryV3dToPath;
using walk_msgs::convertTrajectoryV2dToPath;

class GeneratorNode
{
public:
  explicit GeneratorNode();
  ~GeneratorNode();

  void spin();

protected:
  bool getPath(walk_msgs::GetPath::Request&,
	       walk_msgs::GetPath::Response&);

private:
  /// \brief Main node handle.
  ros::NodeHandle nodeHandle_;

  ros::ServiceServer getPathSrv_;

  std::string frameName_;

  HalfStepsPatternGenerator patternGenerator_;


  visualization_msgs::MarkerArray steps_;
  nav_msgs::Path leftFootPath_;
  nav_msgs::Path rightFootPath_;
  nav_msgs::Path comPath_;
  nav_msgs::Path zmpPath_;

  ros::Publisher stepsPub_;
  ros::Publisher leftFootPub_;
  ros::Publisher rightFootPub_;
  ros::Publisher comPub_;
  ros::Publisher zmpPub_;
};

GeneratorNode::GeneratorNode()
  : nodeHandle_("halfsteps_pattern_generator"),
    getPathSrv_(),
    frameName_("/world"),
    patternGenerator_(),

    steps_ (),
    leftFootPath_ (),
    rightFootPath_ (),
    comPath_ (),
    zmpPath_ (),

    stepsPub_ (),
    leftFootPub_ (),
    rightFootPub_ (),
    comPub_ ()
{
  typedef boost::function<bool (walk_msgs::GetPath::Request&,
				walk_msgs::GetPath::Response&)> callback_t;
  callback_t callback = boost::bind(&GeneratorNode::getPath, this, _1, _2);
  getPathSrv_ = nodeHandle_.advertiseService("getPath", callback);

  stepsPub_ = nodeHandle_.advertise<visualization_msgs::MarkerArray> ("steps", 5);
  leftFootPub_ = nodeHandle_.advertise<nav_msgs::Path> ("left_foot", 5);
  rightFootPub_ = nodeHandle_.advertise<nav_msgs::Path> ("right_foot", 5);
  comPub_ = nodeHandle_.advertise<nav_msgs::Path> ("com", 5);
  zmpPub_ = nodeHandle_.advertise<nav_msgs::Path> ("zmp", 5);
}

GeneratorNode::~GeneratorNode()
{}

void
GeneratorNode::spin()
{
  ros::Rate rate(10);

  while (ros::ok ())
    {
      stepsPub_.publish (steps_);
      leftFootPub_.publish (leftFootPath_);
      rightFootPub_.publish (rightFootPath_);
      comPub_.publish (comPath_);
      zmpPub_.publish (zmpPath_);

      ros::spinOnce();
      rate.sleep ();
    }
}

bool
GeneratorNode::getPath(walk_msgs::GetPath::Request& req,
		       walk_msgs::GetPath::Response& res)
{
  Posture initialPosture;
  Posture finalPosture;

  HomogeneousMatrix3d initialLeftFootPosition;
  HomogeneousMatrix3d initialRightFootPosition;
  walk::Vector3d initialCenterOfMassPosition;

  convertPoseToHomogeneousMatrix(initialLeftFootPosition,
				 req.initial_left_foot_position);
  convertPoseToHomogeneousMatrix(initialRightFootPosition,
				 req.initial_right_foot_position);
  convertPointToVector3d(initialCenterOfMassPosition,
			 req.initial_center_of_mass_position);

  patternGenerator_.setInitialRobotPosition(initialLeftFootPosition,
					    initialRightFootPosition,
					    initialCenterOfMassPosition,
					    initialPosture);

  HomogeneousMatrix3d finalLeftFootPosition;
  HomogeneousMatrix3d finalRightFootPosition;
  walk::Vector3d finalCenterOfMassPosition;

  convertPoseToHomogeneousMatrix(finalLeftFootPosition,
				 req.final_left_foot_position);
  convertPoseToHomogeneousMatrix(finalRightFootPosition,
				 req.final_right_foot_position);
  convertPointToVector3d(finalCenterOfMassPosition,
			req.final_center_of_mass_position);

  patternGenerator_.setFinalRobotPosition(finalLeftFootPosition,
					  finalRightFootPosition,
					  finalCenterOfMassPosition,
					  finalPosture);

  bool startWithLeftFoot = req.start_with_left_foot;
  HalfStepsPatternGenerator::footsteps_t steps;
  convertFootprint(steps, req.footprints);

  patternGenerator_.setSteps(steps, startWithLeftFoot);

  //FIXME: no sliding for now.
  HalfStepsPatternGenerator::slides_t slides;
  for (unsigned i = 0; i < steps.size (); ++i)
    slides.push_back (std::make_pair (-0.1, -0.1));
  patternGenerator_.setSlides(slides);

  res.path.left_foot.header.seq = 0;
  res.path.left_foot.header.stamp.sec = 0;
  res.path.left_foot.header.stamp.nsec = 0;
  res.path.left_foot.header.frame_id = frameName_;

  res.path.right_foot.header = res.path.left_foot.header;
  res.path.center_of_mass.header = res.path.left_foot.header;
  res.path.zmp.header = res.path.left_foot.header;

  convertTrajectoryToPath(res.path.left_foot,
			  patternGenerator_.leftFootTrajectory(),
			  frameName_);
  convertTrajectoryToPath(res.path.right_foot,
			  patternGenerator_.rightFootTrajectory(),
			  frameName_);
  convertTrajectoryV3dToPath(res.path.center_of_mass,
			     patternGenerator_.centerOfMassTrajectory(),
			     frameName_);
  convertTrajectoryV2dToPath(res.path.zmp,
			     patternGenerator_.zmpTrajectory(),
			     frameName_);

  // Prepare topics data.
  leftFootPath_ = res.path.left_foot;
  leftFootPath_.header.frame_id = "/world";
  rightFootPath_ = res.path.right_foot;
  rightFootPath_.header.frame_id = "/world";
  convertTrajectoryV3dToPath(comPath_,
			     patternGenerator_.centerOfMassTrajectory(),
			     frameName_);
  comPath_.header.frame_id = "/world";
  convertTrajectoryV2dToPath(zmpPath_,
			     patternGenerator_.zmpTrajectory(),
			     frameName_);
  zmpPath_.header.frame_id = "/world";

  uint32_t shape = visualization_msgs::Marker::CUBE;
  uint32_t id = 0;
  bool isLeft = startWithLeftFoot;
  BOOST_FOREACH (const HalfStepsPatternGenerator::footstep_t& step,
		 patternGenerator_.steps ())
    {
      visualization_msgs::Marker marker;
      // Set the frame ID and timestamp.  See the TF tutorials for
      // information on these.
      marker.header.frame_id = frameName_;
      marker.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker.  This serves to
      // create a unique ID Any marker sent with the same namespace
      // and id will overwrite the old one
      marker.ns = "halfsteps_pattern_generator";
      marker.id = id++;

      // Set the marker type.
      marker.type = shape;

      // Set the marker action.
      marker.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose
      // relative to the frame/time specified in the header
      marker.pose.position.x = step.position[0];
      marker.pose.position.y = step.position[1];
      marker.pose.position.z = 0.;

      btQuaternion quaternion;
      quaternion.setEuler (step.position[2], 0., 0.);
      marker.pose.orientation.x = quaternion.x ();
      marker.pose.orientation.y = quaternion.y ();
      marker.pose.orientation.z = quaternion.z ();
      marker.pose.orientation.w = quaternion.w ();

      // Set the scale of the marker
      marker.scale.x = 0.2172;
      marker.scale.y = 0.138;
      marker.scale.z = 0.001;

      // Set the color
      marker.color.r = isLeft ? 1.0f : 0.0f;
      marker.color.g = isLeft ? 0.0f : 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 0.5f;

      marker.lifetime = ros::Duration();

      isLeft = !isLeft;

      steps_.markers.push_back (marker);
    }

  std::stringstream ss;
  walk::YamlWriter<HalfStepsPatternGenerator> writer (patternGenerator_);
  writer.write (ss);
  nodeHandle_.setParam ("walk_movement", ss.str ());
  return true;
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

