#ifndef WALK_INTERFACE_YAML_HXX
# define WALK_INTERFACE_YAML_HXX
# include <fstream>
# include <sstream>
# include <stdexcept>

# include <boost/foreach.hpp>
# include <boost/filesystem/fstream.hpp>

# include "yaml-cpp/yaml.h"

namespace walk
{
  template <typename T>
  YamlReader<T>::YamlReader (const boost::filesystem::path& path)
    : T ()
  {
    boost::filesystem::ifstream stream (path);
    load (stream);
  }

  template <typename T>
  YamlReader<T>::YamlReader (std::istream& stream)
    : T ()
  {
    load (stream);
  }

  template <typename T, int I, int J>
  void
  operator>> (const YAML::Node& node, Eigen::Matrix<T, I, J>& matrix)
  {
    if (node.Type() != YAML::NodeType::Sequence)
      throw std::runtime_error("bad type");
    unsigned cols = node.size ();
    if (!cols)
      return;
    unsigned rows = node[0].size ();

    matrix.resize (rows, cols);

    if (rows)
      for (unsigned i = 0; i < cols; ++i)
	for (unsigned j = 0; j < rows; ++j)
	  node[i][j] >> matrix (j, i);
    else
      for (unsigned i = 0; i < cols; ++i)
	node[i] >> matrix (i, 0);
  }

  template <typename T>
  void
  operator>> (const YAML::Node& node, StampedFootprint<T>& footprint)
  {
    if (node.Type() != YAML::NodeType::Map)
      throw std::runtime_error("bad type");

    using namespace boost::posix_time;
    long t;
    node["beginTime"] >> t;
    //FIXME:
    long d;
    node["duration"] >> d;
    footprint.duration = milliseconds (d);
    node["position"] >> footprint.position;
  }

  template <typename T>
  void
  operator>> (const YAML::Node& node, StampedPosition<T>& stampedPosition)
  {
    if (node.Type() != YAML::NodeType::Map)
      throw std::runtime_error("bad type");

    using namespace boost::posix_time;
    long t;
    node["beginTime"] >> t;
    //FIXME:
    long d;
    node["duration"] >> d;
    stampedPosition.duration = milliseconds (d);

    node["position"] >> stampedPosition.position;
  }

  template <typename T>
  void
  operator>> (const YAML::Node& node,
	      WALK_INTERFACES_EIGEN_STL_VECTOR(StampedFootprint<T>)& footprints)
  {
    if (node.Type() != YAML::NodeType::Sequence)
      throw std::runtime_error("bad type");

    footprints.clear ();
    for (unsigned i = 0; i < node.size (); ++i)
      {
	if (node[i].Type() != YAML::NodeType::Map)
	  throw std::runtime_error("bad type");

	StampedFootprint<T> footprint;
	node[i] >> footprint;
	footprints.push_back (footprint);
      }
  }

  template <typename T>
  void
  operator>> (const YAML::Node& node,
	      Trajectory<T>& trajectory)
  {
    if (node.Type() != YAML::NodeType::Sequence)
      throw std::runtime_error("bad type");

    trajectory.data ().clear ();
    for (unsigned i = 0; i < node.size (); ++i)
      {
	if (node[i].Type() != YAML::NodeType::Map)
	  throw std::runtime_error("bad type");

	typename Trajectory<T>::element_t stampedPosition;
	node[i] >> stampedPosition;
	trajectory.data ().push_back (stampedPosition);
      }
  }

  template <typename T>
  void
  operator>> (const YAML::Node& node, YamlReader<T>& pg)
  {
    if (node.Type() != YAML::NodeType::Map)
      throw std::runtime_error("bad type");

    bool startWithLeftFoot = false;
    typename YamlReader<T>::footprints_t footprints;

    node["footprints"] >> footprints;
    node["start-with-left-foot"] >> startWithLeftFoot;

    pg.setFootprints(footprints, startWithLeftFoot);

    node["initial-position"]["left-foot"] >> pg.initialLeftFootPosition ();
    node["initial-position"]["right-foot"] >> pg.initialRightFootPosition ();
    node["initial-position"]["center-of-mass"] >> pg.initialCenterOfMassPosition ();
    node["initial-position"]["posture"] >> pg.initialPosture ();

    node["final-position"]["left-foot"] >> pg.finalLeftFootPosition ();
    node["final-position"]["right-foot"] >> pg.finalRightFootPosition ();
    node["final-position"]["center-of-mass"] >> pg.finalCenterOfMassPosition ();
    node["final-position"]["posture"] >> pg.finalPosture ();

    node["trajectories"]["left-foot"] >> pg.leftFootTrajectory ();
    node["trajectories"]["right-foot"] >> pg.rightFootTrajectory ();
    node["trajectories"]["center-of-mass"] >> pg.centerOfMassTrajectory ();
    node["trajectories"]["zmp"] >> pg.zmpTrajectory ();
    node["trajectories"]["posture"] >> pg.postureTrajectory ();
  }



  template <typename T>
  void
  YamlReader<T>::load (const boost::filesystem::path& path)
  {
    load (boost::filesystem::ifstream (path));
  }


  // May throw YAML::ParserException if an error happens
  template <typename T>
  void
  YamlReader<T>::load (std::istream& stream)
  {
    YAML::Parser parser (stream);
    YAML::Node doc;

    parser.GetNextDocument (doc);
    doc >> *this;
  }

  template <typename T>
  YamlReader<T>::~YamlReader()
  {}




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

  namespace
  {
    template <typename T>
    YAML::Emitter&
    operator<< (YAML::Emitter& out, const Eigen::MatrixBase<T>& matrix)
    {
      if (matrix.cols () > 1)
	out << YAML::BeginSeq;
      for (int i = 0; i < matrix.cols (); ++i)
	{
	  out << YAML::Flow;
	  out << YAML::BeginSeq;
	  for (int j = 0; j < matrix.rows (); ++j)
	    out << matrix (j, i);
	  out << YAML::EndSeq;
	}
      if (matrix.cols () > 1)
	out << YAML::EndSeq;

      return out;
    }

    template <typename T>
    YAML::Emitter&
    operator<< (YAML::Emitter& out,
		const StampedFootprint<T>& footprint)
    {
      out << YAML::BeginMap
	  << YAML::Key << "beginTime"
	  << YAML::Value
	  << 0. //FIXME:
	  << YAML::Key << "duration"
	  << YAML::Value
	  << (0. + footprint.duration.total_nanoseconds () / 1e9)
	  << YAML::Key << "position"
	  << YAML::Value << footprint.position
	  << YAML::EndMap;
      return out;
    }

    template <typename T>
    YAML::Emitter&
    operator<< (YAML::Emitter& out,
		const WALK_INTERFACES_EIGEN_STL_VECTOR(StampedFootprint<T>)& footprints)
    {
      out << YAML::BeginSeq;
      BOOST_FOREACH (const StampedFootprint<T>& footprint, footprints)
	out << footprint;
      out << YAML::EndSeq;
      return out;
    }

    template <typename T>
    YAML::Emitter&
    operator<< (YAML::Emitter& out, const StampedPosition<T>& stampedPosition)
    {
      out << YAML::BeginMap
	  << YAML::Key << "beginTime"
	  << YAML::Value
	  << 0. //FIXME:
	  << YAML::Key << "duration"
	  << YAML::Value
	  << (0. + stampedPosition.duration.total_nanoseconds () / 1e9)
	  << YAML::Key << "position"
	  << YAML::Value << stampedPosition.position
	  << YAML::EndMap;
      return out;
    }

    template <typename T>
    YAML::Emitter&
    operator<< (YAML::Emitter& out, const Trajectory<T>& trajectory)
    {
      out << YAML::BeginSeq;
      BOOST_FOREACH (const typename Trajectory<T>::element_t& data,
		     trajectory.data ())
	out << data;
      out << YAML::EndSeq;
      return out;
    }

    template <typename T>
    YAML::Emitter&
    operator<< (YAML::Emitter& out, const PatternGenerator<T>& pg)
    {
      out << YAML::Comment ("walk_interfaces - walk motion")
	  << YAML::Newline
	  << YAML::Comment ("generated automatically - DO NOT EDIT")
	  << YAML::Newline
	  << YAML::Newline
	  << YAML::Newline

	  << YAML::BeginMap
	  << YAML::Key << "start-with-left-foot"
	  << YAML::Value << pg.startWithLeftFoot()

	  << YAML::Key << "initial-position"
	  << YAML::Value
	  << YAML::BeginMap
	  << YAML::Key << "left-foot"
	  << YAML::Value << pg.initialLeftFootPosition ()
	  << YAML::Key << "right-foot"
	  << YAML::Value << pg.initialRightFootPosition ()
	  << YAML::Key << "center-of-mass"
	  << YAML::Value << pg.initialCenterOfMassPosition ()
	  << YAML::Key << "posture"
	  << YAML::Value << pg.initialPosture ()
	  << YAML::EndMap

	  << YAML::Key << "final-position"
	  << YAML::Value
	  << YAML::BeginMap
	  << YAML::Key << "left-foot"
	  << YAML::Value << pg.finalLeftFootPosition ()
	  << YAML::Key << "right-foot"
	  << YAML::Value << pg.finalRightFootPosition ()
	  << YAML::Key << "center-of-mass"
	  << YAML::Value << pg.finalCenterOfMassPosition ()
	  << YAML::Key << "posture"
	  << YAML::Value << pg.finalPosture ()
	  << YAML::EndMap

	  << YAML::Key << "footprints"
	  << YAML::Value << pg.footprints ()

	  << YAML::Key << "trajectories"
	  << YAML::Value
	  << YAML::BeginMap
	  << YAML::Key << "left-foot"
	  << YAML::Value << pg.leftFootTrajectory ()
	  << YAML::Key << "right-foot"
	  << YAML::Value << pg.rightFootTrajectory ()
	  << YAML::Key << "center-of-mass"
	  << YAML::Value << pg.centerOfMassTrajectory ()
	  << YAML::Key << "zmp"
	  << YAML::Value << pg.zmpTrajectory ()
	  << YAML::Key << "posture"
	  << YAML::Value << pg.postureTrajectory ()
	  << YAML::EndMap

	  << YAML::EndMap
	  << YAML::Newline
	;
      return out;
    }
  } // end of anonymous namespace.

  template <typename T>
  void
  YamlWriter<T>::write (std::ostream& stream) const
  {
    YAML::Emitter out;
    out << patternGenerator_;
    if (!out.good ())
      {
	std::stringstream ss;
	ss << "failed to emit document: ";
	ss << out.GetLastError ();
	throw std::runtime_error (ss.str ());
      }
    stream << out.c_str ();
  }
} // end of namespace walk.

# include <walk_interfaces/yaml.hxx>
#endif //! WALK_INTERFACE_YAML_HH
