#include <Eigen/LU>

#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>

#include "walk_msgs/conversion.hh"

namespace walk_msgs
{
  void convertPoseToHomogeneousMatrix(walk::HomogeneousMatrix3d& dst,
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


  void convertPointToVector3d(walk::Vector3d& dst,
			      const geometry_msgs::Point& src)
  {
    dst[0] = src.x;
    dst[1] = src.y;
    dst[2] = src.z;
  }

  void convertHomogeneousMatrixToPose(geometry_msgs::Pose& dst,
				      const walk::HomogeneousMatrix3d& src)
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
	poseHeader.stamp.nsec = duration.fractional_seconds() * 1e3;

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

  void convertTrajectoryV2dToPath(walk_msgs::PathPoint2d& dst,
				  const walk::TrajectoryV2d& src,
				  const std::string& frameName)
  {
    std::size_t size = src.data().size();

    std_msgs::Header pointHeader;
    pointHeader.seq = 0;
    pointHeader.stamp.sec = 0.;
    pointHeader.stamp.nsec = 0.;
    pointHeader.frame_id = frameName;

    walk_msgs::Point2dStamped pointStamped;

    walk::TimeDuration duration;
    for (std::size_t i = 0; i < size; ++i)
      {
	// Update header.
	++pointHeader.seq;
	pointHeader.stamp.sec =
	  duration.ticks() / walk::TimeDuration::rep_type::res_adjust ();
	pointHeader.stamp.nsec = duration.fractional_seconds() * 1000000;

	// Fill header.
	pointStamped.header = pointHeader;

	// Fill point.
	pointStamped.point.x = src.data()[i].position[0];
	pointStamped.point.y = src.data()[i].position[1];

	// Add to path.
	dst.points.push_back(pointStamped);

	duration += src.data()[i].duration;
      }
  }

  void convertTrajectoryV3dToPath(walk_msgs::PathPoint3d& dst,
				  const walk::TrajectoryV3d& src,
				  const std::string& frameName)
  {
    std::size_t size = src.data().size();

    std_msgs::Header pointHeader;
    pointHeader.seq = 0;
    pointHeader.stamp.sec = 0.;
    pointHeader.stamp.nsec = 0.;
    pointHeader.frame_id = frameName;

    geometry_msgs::PointStamped pointStamped;

    walk::TimeDuration duration;
    for (std::size_t i = 0; i < size; ++i)
      {
	// Update header.
	++pointHeader.seq;
	pointHeader.stamp.sec =
	  duration.ticks() / walk::TimeDuration::rep_type::res_adjust ();
	pointHeader.stamp.nsec = duration.fractional_seconds() * 1000000;

	// Fill header.
	pointStamped.header = pointHeader;

	// Fill point.
	pointStamped.point.x = src.data()[i].position[0];
	pointStamped.point.y = src.data()[i].position[1];
	pointStamped.point.z = src.data()[i].position[2];

	// Add to path.
	dst.points.push_back(pointStamped);

	duration += src.data()[i].duration;
      }
  }

  void convertTrajectoryV2dToPath(nav_msgs::Path& dst,
				  const walk::TrajectoryV2d& src,
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

	// Fill point.
	poseStamped.pose.position.x = src.data()[i].position[0];
	poseStamped.pose.position.y = src.data()[i].position[1];
	poseStamped.pose.position.z = 0.;

	poseStamped.pose.orientation.x = 0.;
	poseStamped.pose.orientation.y = 0.;
	poseStamped.pose.orientation.z = 0.;
	poseStamped.pose.orientation.z = 1.;

	// Add to path.
	dst.poses.push_back(poseStamped);

	duration += src.data()[i].duration;
      }
  }

  void convertTrajectoryV3dToPath(nav_msgs::Path& dst,
				  const walk::TrajectoryV3d& src,
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

	// Fill point.
	poseStamped.pose.position.x = src.data()[i].position[0];
	poseStamped.pose.position.y = src.data()[i].position[1];
	poseStamped.pose.position.z = src.data()[i].position[2];

	poseStamped.pose.orientation.x = 0.;
	poseStamped.pose.orientation.y = 0.;
	poseStamped.pose.orientation.z = 0.;
	poseStamped.pose.orientation.z = 1.;

	// Add to path.
	dst.poses.push_back(poseStamped);

	duration += src.data()[i].duration;
      }
  }

} // end of namespace walk_msgs.
