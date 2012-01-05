#ifndef HALFSTEPS_PATTERN_YAML_HH
# define HALFSTEPS_PATTERN_YAML_HH
# include <utility>
# include <walk_interfaces/pattern-generator.hh>
# include <walk_interfaces/yaml.hh>

namespace walk
{
  inline YAML::Emitter&
  operator<< (YAML::Emitter& out, const footprint_t& footprint)
  {
    out << (const walk::Footprint2d&) (footprint)
	<< YAML::BeginMap
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
	      const WALK_INTERFACES_EIGEN_STL_VECTOR(footprint_t)& footprints)
  {
    out << YAML::BeginSeq;
    BOOST_FOREACH (const footprint_t& footprint, footprints)
      out << footprint;
    out << YAML::EndSeq;
    return out;
  }

  inline void
  operator>> (const YAML::Node& node, footprint_t& footprint)
  {
    checkYamlType (node, YAML::NodeType::Map, "footprint");

    node >> (walk::Footprint2d&) (footprint);
    node["slide-up"] >> footprint.slideUp;
    node["slide-down"] >> footprint.slideDown;
    node["horizontal-distance"] >> footprint.horizontalDistance;
    node["step-height"] >> footprint.stepHeight;
  }

} // end of namespace walk.

#endif //! HALFSTEPS_PATTERN_YAML_HH
