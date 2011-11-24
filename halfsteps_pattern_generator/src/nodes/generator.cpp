#include <stdexcept>
#include <boost/bind.hpp>
#include <boost/date_time.hpp>

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>

#include <ros/ros.h>
#include "halfsteps_pattern_generator.hh"

#include "geometry_msgs/Pose.h"
#include "walk_msgs/Footprint2d.h"
#include "walk_msgs/GetPath.h"


void convertPoseToHomogeneousMatrix(walk::HomogeneousMatrix& dst,
				    const geometry_msgs::Pose& src);
void convertHomogeneousMatrixToPose(geometry_msgs::Pose&,
				    const walk::HomogeneousMatrix&);

void convertFootprint(HalfStepsPatternGenerator::footsteps_t& dst,
		      const std::vector<walk_msgs::Footprint2d>& src);



void convertTrajectoryToPath(nav_msgs::Path&,
			     const walk::Trajectory3d&,
			     const std::string& frameName);

void convertPoseToHomogeneousMatrix(walk::HomogeneousMatrix& dst,
				    const geometry_msgs::Pose& src)
{
  btQuaternion quaternion
    (src.orientation.x, src.orientation.y, src.orientation.z,
     src.orientation.w);
  btMatrix3x3 rotation (quaternion);

  dst.setIdentity();

  // Copy the rotation component.
  for(unsigned i = 0; i < 3; ++i)
    for(unsigned j = 0; j < 3; ++j)
      dst (i, j) = rotation[i][j];

  // Copy the translation component.
  dst(0, 3) = src.position.x;
  dst(1, 3) = src.position.y;
  dst(2, 3) = src.position.z;
}

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

void convertHomogeneousMatrixToPose(geometry_msgs::Pose& dst,
				    const walk::HomogeneousMatrix& src)
{
  btMatrix3x3 rotation;
  btQuaternion quaternion;
  for(unsigned i = 0; i < 3; ++i)
    for(unsigned j = 0; j < 3; ++j)
      rotation[i][j] = src (i, j);
  rotation.getRotation (quaternion);

  dst.position.x = src (0, 3);
  dst.position.y = src (1, 3);
  dst.position.z = src (2, 3);

  dst.orientation.x = quaternion.x ();
  dst.orientation.y = quaternion.y ();
  dst.orientation.z = quaternion.z ();
  dst.orientation.w = quaternion.w ();

}

void convertTrajectoryToPath(nav_msgs::Path& dst,
			     const walk::Trajectory3d& src,
			     const std::string& frameName)
{
  std::size_t size = src.data().size();

  std_msgs::Header poseHeader;
  poseHeader.seq = 0;
  poseHeader.stamp.sec = 0.;
  poseHeader.stamp.nsec = 0.;
  poseHeader.frame_id = frameName;

  geometry_msgs::PoseStamped poseStamped;

  walk::TimeDuration duration;
  for (std::size_t i = 0; i < size; ++i)
    {
      // Update header.
      ++poseHeader.seq;
      poseHeader.stamp.sec =
	duration.ticks() / walk::TimeDuration::rep_type::res_adjust ();
      poseHeader.stamp.nsec = duration.fractional_seconds() * 1000000;

      // Fill header.
      poseStamped.header = poseHeader;

      // Fill pose.
      convertHomogeneousMatrixToPose
	(poseStamped.pose,
	 src.data()[i].position);

      // Add to path.
      dst.poses.push_back(poseStamped);

      duration += src.data()[i].duration;
    }
}

using walk::HomogeneousMatrix;
using walk::Posture;

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
};

GeneratorNode::GeneratorNode()
  : nodeHandle_("halfsteps_pattern_generator"),
    getPathSrv_(),
    frameName_("world"),
    patternGenerator_()
{
  typedef boost::function<bool (walk_msgs::GetPath::Request&,
				walk_msgs::GetPath::Response&)> callback_t;
  callback_t callback = boost::bind(&GeneratorNode::getPath, this, _1, _2);
  getPathSrv_ = nodeHandle_.advertiseService("getPath", callback);
}

GeneratorNode::~GeneratorNode()
{}

void
GeneratorNode::spin()
{
  ros::spin();
}

bool
GeneratorNode::getPath(walk_msgs::GetPath::Request& req,
		       walk_msgs::GetPath::Response& res)
{
  Posture initialPosture;
  Posture finalPosture;

  HomogeneousMatrix initialLeftFootPosition;
  HomogeneousMatrix initialRightFootPosition;
  HomogeneousMatrix initialCenterOfMassPosition;

  convertPoseToHomogeneousMatrix(initialLeftFootPosition,
				 req.initial_left_foot_position);
  convertPoseToHomogeneousMatrix(initialRightFootPosition,
				 req.initial_right_foot_position);
  convertPoseToHomogeneousMatrix(initialCenterOfMassPosition,
				 req.initial_center_of_mass_position);

  patternGenerator_.setInitialRobotPosition(initialLeftFootPosition,
					    initialRightFootPosition,
					    initialCenterOfMassPosition,
					    initialPosture);

  HomogeneousMatrix finalLeftFootPosition;
  HomogeneousMatrix finalRightFootPosition;
  HomogeneousMatrix finalCenterOfMassPosition;

  convertPoseToHomogeneousMatrix(finalLeftFootPosition,
				 req.final_left_foot_position);
  convertPoseToHomogeneousMatrix(finalRightFootPosition,
				 req.final_right_foot_position);
  convertPoseToHomogeneousMatrix(finalCenterOfMassPosition,
				 req.final_center_of_mass_position);

  patternGenerator_.setFinalRobotPosition(finalLeftFootPosition,
					  finalRightFootPosition,
					  finalCenterOfMassPosition,
					  finalPosture);

  HalfStepsPatternGenerator::footsteps_t steps;
  convertFootprint(steps, req.footprints);

  bool startWithLeftFoot = req.start_with_left_foot;
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
  convertTrajectoryToPath(res.path.center_of_mass,
			  patternGenerator_.centerOfMassTrajectory(),
			  frameName_);
  convertTrajectoryToPath(res.path.zmp,
			  patternGenerator_.zmpTrajectory(),
			  frameName_);
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

