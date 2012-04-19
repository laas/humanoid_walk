#ifndef HALFSTEPS_PATTERN_BINARY_HH
# define HALFSTEPS_PATTERN_BINARY_HH
# include <utility>
# include <walk_interfaces/pattern-generator.hh>
# include <walk_interfaces/binary.hh>

# include <halfsteps_pattern_generator.hh>

namespace walk
{
  inline
  BinaryReaderHelper&
  operator>> (BinaryReaderHelper& helper,
	      HalfStepsPatternGenerator::footprint_t& footprint)
  {
    using namespace boost::posix_time;
    using namespace boost::gregorian;

    std::size_t beginTimeStrSize = 0;
    helper.stream.read(reinterpret_cast<char*>(&beginTimeStrSize), sizeof (std::size_t));

    char* str = new char[beginTimeStrSize];
    helper.stream.read(str, beginTimeStrSize * sizeof (char));

    std::string beginTimeStr = str;
    delete[] str;

    footprint.beginTime = walk::Time (from_iso_string (beginTimeStr));

    double d = 0;
    helper.stream.read(reinterpret_cast<char*>(&d), sizeof (double));
    footprint.duration = milliseconds (d * 1e3);
    helper >> footprint.position;
    return helper;
  }

  inline
  BinaryReaderHelper&
  operator>>
  (BinaryReaderHelper& helper,
   WALK_INTERFACES_EIGEN_STL_VECTOR(HalfStepsPatternGenerator::footprint_t)&
   footprints)
  {
    WALK_INTERFACES_EIGEN_STL_VECTOR
      (HalfStepsPatternGenerator::footprint_t)::size_type
      countFootprints = 0.;

    helper.stream.read
      (reinterpret_cast<char*>(&countFootprints),
       sizeof (WALK_INTERFACES_EIGEN_STL_VECTOR
    	       (HalfStepsPatternGenerator::footprint_t)::size_type));

    footprints.clear ();
    for (unsigned i = 0; i < countFootprints; ++i)
      {
	HalfStepsPatternGenerator::footprint_t footprint;
	helper >> footprint;
	footprints.push_back (footprint);
      }
    return helper;
  }

  inline
  BinaryWriterHelper&
  operator<< (BinaryWriterHelper& helper,
	      const HalfStepsPatternGenerator::footprint_t& footprint)
  {
    using namespace boost::gregorian;

    std::string beginTimeStr = to_iso_string (footprint.beginTime);
    std::size_t size = beginTimeStr.size () + 1;

    helper.stream.write(reinterpret_cast<const char*>(&size), sizeof (std::size_t));
    helper.stream.write(beginTimeStr.c_str(), size * sizeof (unsigned char));

    double t = 0. + footprint.duration.total_nanoseconds () / 1e9;
    helper.stream.write(reinterpret_cast<const char*>(&t), sizeof (double));
    helper << footprint.position;
    return helper;
  }

  inline
  BinaryWriterHelper&
  operator<<
  (BinaryWriterHelper& helper,
   const WALK_INTERFACES_EIGEN_STL_VECTOR
   (HalfStepsPatternGenerator::footprint_t)& footprints)
  {
    WALK_INTERFACES_EIGEN_STL_VECTOR
      (HalfStepsPatternGenerator::footprint_t)::size_type
      countFootprints = footprints.size ();

    helper.stream.write
      (reinterpret_cast<const char*>(&countFootprints),
       sizeof (WALK_INTERFACES_EIGEN_STL_VECTOR
	       (HalfStepsPatternGenerator::footprint_t)::size_type));

    BOOST_FOREACH (const HalfStepsPatternGenerator::footprint_t& footprint, footprints)
      helper << footprint;
    return helper;
  }



  // inline BINARY::Emitter&
  // operator<< (BINARY::Emitter& out,
  // 	      const HalfStepsPatternGenerator::footprint_t& footprint)
  // {
  //   out << BINARY::BeginMap
  // 	<< BINARY::Key << "beginTime"
  // 	<< BINARY::Value
  // 	<< to_iso_string (footprint.beginTime)
  // 	<< BINARY::Key << "duration"
  // 	<< BINARY::Value
  // 	<< (0. + footprint.duration.total_nanoseconds () / 1e9)
  // 	<< BINARY::Key << "position"
  // 	<< BINARY::Value << footprint.position
  // 	<< BINARY::Key << "slide-up"
  // 	<< BINARY::Value
  // 	<< footprint.slideUp
  // 	<< BINARY::Key << "slide-down"
  // 	<< BINARY::Value
  // 	<< footprint.slideDown
  // 	<< BINARY::Key << "horizontal-distance"
  // 	<< BINARY::Value
  // 	<< footprint.horizontalDistance
  // 	<< BINARY::Key << "step-height"
  // 	<< BINARY::Value
  // 	<< footprint.stepHeight
  // 	<< BINARY::EndMap;
  //   return out;
  // }

  // inline void
  // operator>> (const BINARY::Node& node,
  // 	      HalfStepsPatternGenerator::footprint_t& footprint)
  // {
  //   checkYamlType (node, BINARY::NodeType::Map, "footprint");

  //   using namespace boost::posix_time;
  //   using namespace boost::gregorian;

  //   std::string beginTimeStr;
  //   node["beginTime"] >> beginTimeStr;
  //   footprint.beginTime = walk::Time (from_iso_string (beginTimeStr));
  //   double d = 0;
  //   node["duration"] >> d;
  //   footprint.duration = milliseconds (d * 1e3);
  //   node["position"] >> footprint.position;

  //   node["slide-up"] >> footprint.slideUp;
  //   node["slide-down"] >> footprint.slideDown;
  //   node["horizontal-distance"] >> footprint.horizontalDistance;
  //   node["step-height"] >> footprint.stepHeight;
  // }

} // end of namespace walk.

#endif //! HALFSTEPS_PATTERN_BINARY_HH
