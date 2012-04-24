#ifndef WALK_MSGS_ABSTRACT_NODE_HXX
# define WALK_MSGS_ABSTRACT_NODE_HXX
# include <sstream>
# include <stdexcept>
# include <boost/bind.hpp>

# include <LinearMath/btQuaternion.h>

# include <ros/ros.h>

# include <walk_interfaces/binary.hh>
# include <walk_interfaces/yaml.hh>

# include "geometry_msgs/Pose.h"
# include "visualization_msgs/MarkerArray.h"
# include "walk_msgs/Footprint2d.h"
# include "walk_msgs/GetPath.h"

# include <walk_msgs/conversion.hh>

namespace walk_msgs
{
  using walk::HomogeneousMatrix3d;
  using walk::Posture;

  template <typename T, typename U, typename S>
  AbstractNode<T, U, S>::AbstractNode (const std::string& rosNamespace,
				       const std::string& frameWorldId,
				       bool enableService)
    : nodeHandle_ (rosNamespace),
      rate_ (1),
      getPathSrv_ (),
      frameName_ (frameWorldId),
      parameterName_ ("walk_movement"),
      patternGenerator_ (),

      footprints_ (),
      leftFootPath_ (),
      rightFootPath_ (),
      comPath_ (),
      zmpPath_ (),

      footprintsPub_ (),
      leftFootPub_ (),
      rightFootPub_ (),
      comPub_ (),

      footPrintWidth_ (0.),
      footPrintHeight_ (0.)
  {
    ros::param::param<double> ("~footprint_width", footPrintWidth_, 0.2172);
    ros::param::param<double> ("~footprint_height", footPrintHeight_, 0.138);

    typedef boost::function<
    bool (typename serviceRosType_t::Request&,
	  typename serviceRosType_t::Response&)>
      callback_t;
    if (enableService)
      {
	callback_t callback = boost::bind(&AbstractNode::getPath, this, _1, _2);
	getPathSrv_ = nodeHandle_.advertiseService("getPath", callback);
      }

    footprintsPub_ =
      nodeHandle_.advertise<visualization_msgs::MarkerArray>
      ("footprints", 1,
       ros::SubscriberStatusCallback (),
       ros::SubscriberStatusCallback (),
       ros::VoidConstPtr (),
       true);
    leftFootPub_ = nodeHandle_.advertise<nav_msgs::Path>
      ("left_foot", 1,
       ros::SubscriberStatusCallback (),
       ros::SubscriberStatusCallback (),
       ros::VoidConstPtr (),
       true);
    rightFootPub_ = nodeHandle_.advertise<nav_msgs::Path>
      ("right_foot", 1,
       ros::SubscriberStatusCallback (),
       ros::SubscriberStatusCallback (),
       ros::VoidConstPtr (),
       true);
    comPub_ = nodeHandle_.advertise<nav_msgs::Path>
      ("com", 1,
       ros::SubscriberStatusCallback (),
       ros::SubscriberStatusCallback (),
       ros::VoidConstPtr (),
       true);
    zmpPub_ = nodeHandle_.advertise<nav_msgs::Path>
      ("zmp", 1,
       ros::SubscriberStatusCallback (),
       ros::SubscriberStatusCallback (),
       ros::VoidConstPtr (),
       true);
  }

  template <typename T, typename U, typename S>
  AbstractNode<T, U, S>::AbstractNode (const std::string& rosNamespace,
				       const std::string& frameWorldId,
				       const patternGenerator_t& pg,
				       bool enableService)
    : nodeHandle_ (rosNamespace),
      rate_ (1),
      getPathSrv_ (),
      frameName_ (frameWorldId),
      parameterName_ ("walk_movement"),
      patternGenerator_ (pg),

      footprints_ (),
      leftFootPath_ (),
      rightFootPath_ (),
      comPath_ (),
      zmpPath_ (),

      footprintsPub_ (),
      leftFootPub_ (),
      rightFootPub_ (),
      comPub_ (),

      footPrintWidth_ (0.),
      footPrintHeight_ (0.)
  {
    ros::param::param<double> ("~footprint_width", footPrintWidth_, 0.2172);
    ros::param::param<double> ("~footprint_height", footPrintHeight_, 0.138);

    typedef boost::function<
    bool (typename serviceRosType_t::Request&,
	  typename serviceRosType_t::Response&)>
      callback_t;

    if (enableService)
      {
	callback_t callback = boost::bind(&AbstractNode::getPath, this, _1, _2);
	getPathSrv_ = nodeHandle_.advertiseService("getPath", callback);
      }

    footprintsPub_ =
      nodeHandle_.advertise<visualization_msgs::MarkerArray>
      ("footprints", 1,
       ros::SubscriberStatusCallback (),
       ros::SubscriberStatusCallback (),
       ros::VoidConstPtr (),
       true);
    leftFootPub_ = nodeHandle_.advertise<nav_msgs::Path>
      ("left_foot", 1,
       ros::SubscriberStatusCallback (),
       ros::SubscriberStatusCallback (),
       ros::VoidConstPtr (),
       true);
    rightFootPub_ = nodeHandle_.advertise<nav_msgs::Path>
      ("right_foot", 1,
       ros::SubscriberStatusCallback (),
       ros::SubscriberStatusCallback (),
       ros::VoidConstPtr (),
       true);
    comPub_ = nodeHandle_.advertise<nav_msgs::Path>
      ("com", 1,
       ros::SubscriberStatusCallback (),
       ros::SubscriberStatusCallback (),
       ros::VoidConstPtr (),
       true);
    zmpPub_ = nodeHandle_.advertise<nav_msgs::Path>
      ("zmp", 1,
       ros::SubscriberStatusCallback (),
       ros::SubscriberStatusCallback (),
       ros::VoidConstPtr (),
       true);
  }

  template <typename T, typename U, typename S>
  AbstractNode<T, U, S>::~AbstractNode()
  {}


  template <typename T, typename U, typename S>
  void
  AbstractNode<T, U, S>::spin ()
  {
    // We only need to publish once as this is latched topics.
    footprintsPub_.publish (footprints_);
    leftFootPub_.publish (leftFootPath_);
    rightFootPub_.publish (rightFootPath_);
    comPub_.publish (comPath_);
    zmpPub_.publish (zmpPath_);

    while (ros::ok ())
      {
	ros::spinOnce();
	rate_.sleep ();
      }
  }

  template <typename T, typename U, typename S>
  bool
  AbstractNode<T, U, S>::getPath (typename serviceRosType_t::Request& req,
				  typename serviceRosType_t::Response& res)
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
    typename patternGenerator_t::footprints_t footprints;
    convertFootprint(footprints, req.footprints);

    patternGenerator_.setFootprints(footprints, startWithLeftFoot);

    setupPatternGenerator (req);

    prepareTopicsData (res, startWithLeftFoot);
    writeMotionAsParameter ();

    // We only need to publish once as this is latched topics.
    footprintsPub_.publish (footprints_);
    leftFootPub_.publish (leftFootPath_);
    rightFootPub_.publish (rightFootPath_);
    comPub_.publish (comPath_);
    zmpPub_.publish (zmpPath_);

    return true;
  }

  template <typename T, typename U, typename S>
  void
  AbstractNode<T, U, S>::prepareTopicsData
  (typename serviceRosType_t::Response& res, bool startWithLeftFoot)
  {
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
    leftFootPath_.header.frame_id = frameName_;
    rightFootPath_ = res.path.right_foot;
    rightFootPath_.header.frame_id = frameName_;
    convertTrajectoryV3dToPath(comPath_,
			       patternGenerator_.centerOfMassTrajectory(),
			       frameName_);
    comPath_.header.frame_id = frameName_;
    convertTrajectoryV2dToPath(zmpPath_,
			       patternGenerator_.zmpTrajectory(),
			       frameName_);
    zmpPath_.header.frame_id = frameName_;

    uint32_t shape = visualization_msgs::Marker::CUBE;
    uint32_t id = 0;
    bool isLeft = startWithLeftFoot;
    BOOST_FOREACH (const typename patternGenerator_t::footprint_t& footprint,
		   patternGenerator_.footprints ())
      {
	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for
	// information on these.
	// The timestamp *must* be 0 so that rviz always render
	// the element independently from the current time associated
	// with the tf transform.
	marker.header.frame_id = frameName_;
	marker.header.stamp = ros::Time (0);

	// Set the namespace and id for this marker.  This serves to
	// create a unique ID Any marker sent with the same namespace
	// and id will overwrite the old one
	marker.ns = "walk_msgs";
	marker.id = id++;

	// Set the marker type.
	marker.type = shape;

	// Set the marker action.
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose
	// relative to the frame/time specified in the header
	marker.pose.position.x = footprint.position[0];
	marker.pose.position.y = footprint.position[1];
	marker.pose.position.z = 0.;

	btQuaternion quaternion;
	quaternion.setEuler (0., 0., footprint.position[2]);
	marker.pose.orientation.x = quaternion.x ();
	marker.pose.orientation.y = quaternion.y ();
	marker.pose.orientation.z = quaternion.z ();
	marker.pose.orientation.w = quaternion.w ();

	// Set the scale of the marker
	marker.scale.x = footPrintWidth_;
	marker.scale.y = footPrintHeight_;
	marker.scale.z = 0.001;

	// Set the color
	marker.color.r = isLeft ? 1.0f : 0.0f;
	marker.color.g = isLeft ? 0.0f : 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 0.5f;

	marker.lifetime = ros::Duration();

	isLeft = !isLeft;

	footprints_.markers.push_back (marker);
      }
  }

  template <typename T, typename U, typename S>
  void
  AbstractNode<T, U, S>::writeMotionAsParameter ()
  {
    walk::BinaryWriter<patternGenerator_t> writerBin (patternGenerator_);
    writerBin.write ("/tmp/trajectory.bin");

    std::stringstream ss;
    walk::YamlWriter<patternGenerator_t> writer (patternGenerator_);
    writer.write (ss);
    nodeHandle_.setParam (parameterName_, ss.str ());
  }
} // end of namespace walk_msgs.

#endif //! WALK_MSGS_ABSTRACT_NODE_HXX
