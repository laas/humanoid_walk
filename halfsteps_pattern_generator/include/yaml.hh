#ifndef HALFSTEPS_PATTERN_YAML_HH
# define HALFSTEPS_PATTERN_YAML_HH
# include <utility>
# include <walk_interfaces/pattern-generator.hh>
# include <walk_interfaces/yaml.hh>

# include <halfsteps_pattern_generator.hh>

namespace walk
{
  inline YAML::Emitter&
  operator<< (YAML::Emitter& out,
	      const HalfStepsPatternGenerator::footprint_t& footprint)
  {
    out << YAML::BeginMap
	<< YAML::Key << "beginTime"
	<< YAML::Value
	<< to_iso_string (footprint.beginTime)
	<< YAML::Key << "duration"
	<< YAML::Value
	<< (0. + footprint.duration.total_nanoseconds () / 1e9)
	<< YAML::Key << "position"
	<< YAML::Value << footprint.position
	<< YAML::Key << "slide-up"
	<< YAML::Value
	<< footprint.slideUp
	<< YAML::Key << "slide-down"
	<< YAML::Value
	<< footprint.slideDown
	<< YAML::Key << "horizontal-distance"
	<< YAML::Value
	<< footprint.horizontalDistance
	<< YAML::Key << "step-height"
	<< YAML::Value
	<< footprint.stepHeight
	<< YAML::EndMap;
    return out;
  }

  inline YAML::Emitter&
  operator<< (YAML::Emitter& out,
	      const WALK_INTERFACES_EIGEN_STL_VECTOR
	      (HalfStepsPatternGenerator::footprint_t)& footprints)
  {
    out << YAML::BeginSeq;
    typedef HalfStepsPatternGenerator::footprint_t a_t;
    BOOST_FOREACH (const a_t& footprint, footprints)
      out << footprint;
    out << YAML::EndSeq;
    return out;
  }

  inline void
  operator>> (const YAML::Node& node,
	      HalfStepsPatternGenerator::footprint_t& footprint)
  {
    checkYamlType (node, YAML::NodeType::Map, "footprint");

    using namespace boost::posix_time;
    using namespace boost::gregorian;

    std::string beginTimeStr;
    node["beginTime"] >> beginTimeStr;
    footprint.beginTime = walk::Time (from_iso_string (beginTimeStr));
    double d = 0;
    node["duration"] >> d;
    footprint.duration = milliseconds (d * 1e3);
    node["position"] >> footprint.position;

    node["slide-up"] >> footprint.slideUp;
    node["slide-down"] >> footprint.slideDown;
    node["horizontal-distance"] >> footprint.horizontalDistance;
    node["step-height"] >> footprint.stepHeight;
  }

} // end of namespace walk.

#endif //! HALFSTEPS_PATTERN_YAML_HH
