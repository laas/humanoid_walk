#ifndef WALK_MSGS_ABSTRACT_NODE_HH
# define WALK_MSGS_ABSTRACT_NODE_HH
# include <sstream>
# include <stdexcept>
# include <boost/bind.hpp>
# include <boost/date_time.hpp>

# include <Eigen/LU>

# include <LinearMath/btMatrix3x3.h>
# include <LinearMath/btQuaternion.h>

# include <ros/ros.h>

# include <walk_interfaces/yaml.hh>

# include "halfsteps_pattern_generator.hh"

# include "geometry_msgs/Pose.h"
# include "visualization_msgs/MarkerArray.h"
# include "walk_msgs/Footprint2d.h"
# include "walk_msgs/GetPath.h"

# include <walk_msgs/conversion.hh>


namespace walk_msgs
{
  template <typename T, typename U>
  class AbstractNode
  {
  public:
    typedef T patternGenerator_t;
    typedef U footprintRosType_t;

    explicit AbstractNode (const std::string& rosNamespace,
			   const std::string& frameWorldId);
    ~AbstractNode ();
    void spin();

  protected:
    bool getPath(walk_msgs::GetPath::Request&,
		 walk_msgs::GetPath::Response&);

    virtual void convertFootprint
    (typename patternGenerator_t::footprints_t& dst,
     const std::vector<footprintRosType_t>& src) = 0;

    virtual void setupPatternGenerator (walk_msgs::GetPath::Request& req) = 0;

    patternGenerator_t& patternGenerator ()
    {
      return patternGenerator_;
    }

  private:
    /// \brief Main node handle.
    ros::NodeHandle nodeHandle_;

    ros::ServiceServer getPathSrv_;

    std::string frameName_;

    patternGenerator_t patternGenerator_;

    visualization_msgs::MarkerArray footprints_;
    nav_msgs::Path leftFootPath_;
    nav_msgs::Path rightFootPath_;
    nav_msgs::Path comPath_;
    nav_msgs::Path zmpPath_;

    ros::Publisher footprintsPub_;
    ros::Publisher leftFootPub_;
    ros::Publisher rightFootPub_;
    ros::Publisher comPub_;
    ros::Publisher zmpPub_;
  };

} // end of namespace walk_msgs

# include <walk_msgs/abstract-node.hxx>
#endif //! WALK_MSGS_ABSTRACT_NODE_HH
