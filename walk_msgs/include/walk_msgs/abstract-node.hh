#ifndef WALK_MSGS_ABSTRACT_NODE_HH
# define WALK_MSGS_ABSTRACT_NODE_HH
# include <string>
# include <vector>

# include <ros/ros.h>
# include <walk_interfaces/pattern-generator.hh>

# include <nav_msgs/Path.h>
# include <visualization_msgs/MarkerArray.h>

# include "walk_msgs/GetPath.h"

namespace walk_msgs
{
  /// \brief Walking trajectory abstract node.
  ///
  /// This class provides the skeleton of a generator node. A
  /// generator node provides the GetPath service which allows to ask
  /// remotely for walking trajectory computation.
  ///
  /// This node also published the data as topics in order to allow
  /// their display in rviz for instance.
  ///
  /// walking_trajectories.vcg at this package root level is a rviz
  /// configuration file which can be used to display walking
  /// trajectories.
  ///
  /// \tparam T pattern generator type
  /// \tparam U ROS footprint message type
  template <typename T, typename U>
  class AbstractNode
  {
  public:
    /// \brief pattern generatoe type
    typedef T patternGenerator_t;
    /// \brief ROS footprint type
    typedef U footprintRosType_t;

    /// \brief Constructor.
    ///
    /// \param rosNamespace namespace used to initialize the ROS handle
    /// \param frameWorldId frame id in which the trajectories are expressed
    /// \param enableService enable getPath service registration
    explicit AbstractNode (const std::string& rosNamespace,
			   const std::string& frameWorldId,
			   bool enableService = true);

    /// \brief Constructor copying a given pattern generator into the node.
    ///
    /// This constructor is needed when a pattern generator requires arguments
    /// during instantiation. In this case, the default constructor will not work.
    ///
    /// \warning the given pattern generator must be copyable.
    ///
    /// \param rosNamespace namespace used to initialize the ROS handle
    /// \param frameWorldId frame id in which the trajectories are expressed
    /// \param pg pattern generator to be copied into this object
    /// \param enableService enable getPath service registration
    explicit AbstractNode (const std::string& rosNamespace,
			   const std::string& frameWorldId,
			   const patternGenerator_t& pg,
			   bool enableService = true);

    /// \brief Destructor.
    ~AbstractNode ();

    /// \brief Run the node. Returns when the node is exiting.
    void spin();

  protected:
    /// \brief getPath service callback
    ///
    /// This fill the pattern generator with data from the request,
    /// compute the trajectories by calling the computeTrajectories
    /// method and then both put the result in a ROS parameter and
    /// publish a displayable version into various separated topics.
    ///
    /// \param req service request (i.e. input)
    /// \param res service response (i.e. output)
    bool getPath(walk_msgs::GetPath::Request& req,
		 walk_msgs::GetPath::Response& res);

    /// \brief Convert ROS footprint representation into the
    /// corresponding C++ representation.
    ///
    /// Subclasses must implement this method.
    virtual void convertFootprint
    (typename patternGenerator_t::footprints_t& dst,
     const std::vector<footprintRosType_t>& src) = 0;

    /// \brief Pattern generator specific setup.
    ///
    /// Pattern generators often needs specific data to be passed or
    /// dedicated conversion to be made.
    ///
    /// The getPath callback first fill the pattern generator with
    /// data common to all the pattern generators algorithm then calls
    /// the setupPatternGenerator method so that specific treatments
    /// can be realized.
    ///
    /// \param req request passed to the service callback
    virtual void
    setupPatternGenerator (walk_msgs::GetPath::Request& req) = 0;

    /// \brief Pattern generator getter.
    patternGenerator_t& patternGenerator ()
    {
      return patternGenerator_;
    }

    /// \brief Fill attributes with data which will be published.
    void prepareTopicsData (walk_msgs::GetPath::Response& res,
			    bool startWithLeftFoot);

    /// \brief Write the motion as a parameter of the parameter
    /// server.
    void writeMotionAsParameter ();

  private:
    /// \brief Main node handle.
    ros::NodeHandle nodeHandle_;

    /// \brief Topic publishing rate.
    ros::Rate rate_;

    /// \brief GetPath service.
    ros::ServiceServer getPathSrv_;

    /// \brief World frame id.
    ///
    /// All walking trajectories are expressed w.r.t this frame.
    std::string frameName_;

    /// \brief Parameter in which the trajectory is stored.
    std::string parameterName_;

    /// \brief Pattern generator instance used to generate the
    /// trajectories.
    patternGenerator_t patternGenerator_;

    /// \brief Marker array containing footprints data.
    visualization_msgs::MarkerArray footprints_;

    /// \brief Left foot path.
    nav_msgs::Path leftFootPath_;
    /// \brief Right foot path.
    nav_msgs::Path rightFootPath_;

    /// \brief Center of mass path.
    nav_msgs::Path comPath_;
    /// \brief Zero Momentum point path.
    nav_msgs::Path zmpPath_;

    /// \brief Footprints publisher.
    ros::Publisher footprintsPub_;
    /// \brief Left foot path publisher.
    ros::Publisher leftFootPub_;
    /// \brief Right foot path publisher.
    ros::Publisher rightFootPub_;
    /// \brief Center of mass path publisher.
    ros::Publisher comPub_;
    /// \brief Zero Momentum Point path publisher.
    ros::Publisher zmpPub_;
  };

} // end of namespace walk_msgs

# include <walk_msgs/abstract-node.hxx>
#endif //! WALK_MSGS_ABSTRACT_NODE_HH
