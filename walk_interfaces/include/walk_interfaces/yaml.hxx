#ifndef WALK_INTERFACE_YAML_HXX
# define WALK_INTERFACE_YAML_HXX
# include <fstream>
# include <stdexcept>

# include <boost/foreach.hpp>
# include <boost/filesystem/fstream.hpp>

namespace walk
{
  template <typename T>
  YamlWriter<T>::YamlWriter (const patternGenerator_t& pg)
    : patternGenerator_ (pg)
  {}

  template <typename T>
  YamlWriter<T>::~YamlWriter()
  {}

  template <typename T>
  void
  YamlWriter<T>::write (const std::string& filename) const
  {
    boost::filesystem::path path (filename);
    write (path);
  }

  template <typename T>
  void
  YamlWriter<T>::write (boost::filesystem::path& path) const
  {
    boost::filesystem::ofstream stream
      (path,
       boost::filesystem::ofstream::out | boost::filesystem::ofstream::trunc);
    write (stream);
  }

  template <typename T>
  void
  YamlWriter<T>::write (std::ostream& stream) const
  {
    stream << "# walk_interfaces - walk motion\n"
	   << "# generated automatically - DO NOT EDIT\n"
	   << "\n"
	   << "\n"
	   << "start-with-left-foot: "
	   << (patternGenerator_.startWithLeftFoot() ? "True" : "False")
	   << "\n"
	   << "initial-position:\n"
	   << "    left-foot: ";
    writeMatrix (stream, patternGenerator_.initialLeftFootPosition ());
    stream << "\n"
	   << "    right-foot: ";
    writeMatrix (stream, patternGenerator_.initialRightFootPosition ());
    stream << "\n"
	   << "    center-of-mass: ";
    writeMatrix (stream, patternGenerator_.initialCenterOfMassPosition ());
    stream << "\n"
	   << "    posture: ";
    writeMatrix (stream, patternGenerator_.initialPosture ());
    stream << "\n"
	   << "final-position:\n"
	   << "    left-foot: ";
    writeMatrix (stream, patternGenerator_.finalLeftFootPosition ());
    stream << "\n"
	   << "    right-foot: ";
    writeMatrix (stream, patternGenerator_.finalRightFootPosition ());
    stream << "\n"
	   << "    center-of-mass: ";
    writeMatrix (stream, patternGenerator_.finalCenterOfMassPosition ());
    stream << "\n"
	   << "    posture: ";
    writeMatrix (stream, patternGenerator_.finalPosture ());
    stream << "\n";

    writeFootprints (stream);

    stream << "trajectories:\n"
	   << "    left-foot:\n";
    writeTrajectory(stream, patternGenerator_.leftFootTrajectory ());
    stream << "\n"
	   << "    right-foot:\n";
    writeTrajectory(stream, patternGenerator_.rightFootTrajectory ());
    stream << "\n"
	   << "    center-of-mass:\n";
    writeTrajectory(stream, patternGenerator_.centerOfMassTrajectory ());
    stream << "\n"
	   << "    zmp:\n";
    writeTrajectory(stream, patternGenerator_.zmpTrajectory ());
    stream << "\n"
	   << "    posture:\n";
    writeTrajectory(stream, patternGenerator_.postureTrajectory ());
    stream << "\n";
  }

  template <typename T>
  template <typename U>
  void
  YamlWriter<T>::writeTrajectory (std::ostream& stream,
				  const U& gamma) const
  {
    BOOST_FOREACH(const typename U::element_t& d, gamma.data ())
      {
	stream << "      - duration: "
	       << (0. + d.duration.total_nanoseconds () / 1e9) << "\n";
	stream << "        position: ";
	writeMatrix(stream, d.position);
	stream << "\n";
      }
  }

  template <typename T>
  template <typename S>
  void
  YamlWriter<T>::writeFootprint (std::ostream& stream,
			    const StampedFootprint<S>& footprint) const
  {
    stream
      << "     - duration: "
      << (0. + footprint.duration.total_nanoseconds () / 1e9) << "\n"
      << "       position: ";
    writeMatrix (stream, footprint.position);
    stream << "\n";
  }

  template <typename T>
  void
  YamlWriter<T>::writeFootprints (std::ostream& stream) const
  {
    const typename patternGenerator_t::footprints_t& footprints =
      patternGenerator_.footprints();

    stream << "footprints: \n";
    BOOST_FOREACH(const typename patternGenerator_t::footprint_t& footprint, footprints)
      {
	stream << "    - ";
	writeFootprint(stream, footprint);
	stream << "\n";
      }
  }

  template <typename T>
  template <typename M>
  void
  YamlWriter<T>::writeMatrix (std::ostream& stream, const M& matrix) const
  {
    if (matrix.cols () > 1)
      stream << "[";
    for (int i = 0; i < matrix.cols (); ++i)
      {
	stream << "[";
	for (int j = 0; j < matrix.rows (); ++j)
	  {	    
	    stream << matrix (j, i);
	    if (j + 1 < matrix.rows ())
	      stream << ", ";
	  }
	stream << "]";
	if (i + 1 < matrix.cols ())
	  stream << ", ";
      }
    if (matrix.cols () > 1)
      stream << "]";
  }

} // end of namespace walk.

# include <walk_interfaces/yaml.hxx>
#endif //! WALK_INTERFACE_YAML_HH
