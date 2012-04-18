#ifndef WALK_INTERFACE_BINARY_HXX
# define WALK_INTERFACE_BINARY_HXX
# include <fstream>
# include <sstream>
# include <stdexcept>

# include <boost/foreach.hpp>
# include <boost/format.hpp>
# include <boost/filesystem/fstream.hpp>


namespace walk
{
  static const char binaryFormatMagicNumber[] = "HWBF";
  static const char binaryFormatVersion[] = "1.0.0";

  template <typename T>
  BinaryReaderHelper&
  operator>> (BinaryReaderHelper& helper, Eigen::PlainObjectBase<T>& matrix)
  {
    double rows = 0;
    double cols = 0;
    helper.stream.read(reinterpret_cast<char*>(&cols), sizeof (double));
    helper.stream.read(reinterpret_cast<char*>(&rows), sizeof (double));

    for (unsigned i = 0; i < rows; ++i)
      for (unsigned j = 0; j < cols; ++j)
	{
	  double tmp = 0;
	  helper.stream.read(reinterpret_cast<char*>(&tmp), sizeof (double));
	  matrix (i, j) = tmp;
	}
    return helper;
  }


  template <typename T>
  BinaryReaderHelper&
  operator>> (BinaryReaderHelper& helper, StampedFootprint<T>& footprint)
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

  template <typename T>
  BinaryReaderHelper&
  operator>> (BinaryReaderHelper& helper,
	      WALK_INTERFACES_EIGEN_STL_VECTOR(StampedFootprint<T>)& footprints)
  {
    typename WALK_INTERFACES_EIGEN_STL_VECTOR(StampedFootprint<T>)::size_type
      countFootprints = 0.;

    helper.stream.read
      (reinterpret_cast<char*>(&countFootprints),
       sizeof (typename WALK_INTERFACES_EIGEN_STL_VECTOR
    	       (StampedFootprint<T>)::size_type));

    footprints.clear ();
    for (unsigned i = 0; i < countFootprints; ++i)
      {
	StampedFootprint<T> footprint;
	helper >> footprint;
	footprints.push_back (footprint);
      }
    return helper;
  }

  template <typename T>
  BinaryReaderHelper&
  operator>> (BinaryReaderHelper& helper, StampedPosition<T>& stampedPosition)
  {
    using namespace boost::posix_time;
    using namespace boost::gregorian;

    double d = 0;
    helper.stream.read(reinterpret_cast<char*>(&d), sizeof (double));
    stampedPosition.duration = milliseconds (d * 1e3);
    helper >> stampedPosition.position;
    return helper;
  }

  template <typename T>
  BinaryReaderHelper&
  operator>> (BinaryReaderHelper& helper, DiscretizedTrajectory<T>& trajectory)
  {
    typename DiscretizedTrajectory<T>::data_t::size_type trajectorySize = 0;
    helper.stream.read
      (reinterpret_cast<char*>(&trajectorySize),
       sizeof (typename DiscretizedTrajectory<T>::data_t::size_type));

    trajectory.data ().clear ();

    for (unsigned i = 0; i < trajectorySize; ++i)
      {
	typename DiscretizedTrajectory<T>::element_t stampedPosition;
	helper >> stampedPosition;
	trajectory.data ().push_back (stampedPosition);
      }
    return helper;
  }

  template <typename T>
  BinaryReaderHelper&
  operator>> (BinaryReaderHelper& helper, BinaryReader<T>& pg)
  {
    char magicNumber[sizeof(binaryFormatMagicNumber)];
    char version[sizeof(binaryFormatVersion)];

    helper.stream.read(magicNumber, sizeof(binaryFormatMagicNumber));
    helper.stream.read(version, sizeof(binaryFormatVersion));

    if (strncmp(magicNumber, binaryFormatMagicNumber,
		sizeof(binaryFormatMagicNumber)))
      throw std::runtime_error ("invalid magic number");
    if (strncmp(version, binaryFormatVersion,
		sizeof(binaryFormatVersion)))
      throw std::runtime_error ("invalid format version");

    bool startWithLeftFoot = true;
    typename T::footprints_t footprints;
    helper.stream.read(reinterpret_cast<char*>(&startWithLeftFoot),
		       sizeof(bool));

    helper >> pg.initialLeftFootPosition ()
	   >> pg.initialRightFootPosition ()
	   >> pg.initialCenterOfMassPosition ()
	   >> pg.initialPosture ()
	   >> pg.finalLeftFootPosition ()
	   >> pg.finalRightFootPosition ()
	   >> pg.finalCenterOfMassPosition ()
	   >> pg.finalPosture ()
	   >> footprints
	   >> pg.leftFootTrajectory ()
	   >> pg.rightFootTrajectory ()
	   >> pg.centerOfMassTrajectory ()
	   >> pg.zmpTrajectory ()
	   >> pg.postureTrajectory ();

    pg.setFootprints(footprints, startWithLeftFoot);
    return helper;
  }

  template <typename T>
  BinaryReader<T>::BinaryReader (const boost::filesystem::path& filename)
    : T ()
  {
    boost::filesystem::ifstream stream
      (filename,
       boost::filesystem::ifstream::in
       | boost::filesystem::ifstream::binary);
    load (stream);
  }

  template <typename T>
  BinaryReader<T>::BinaryReader (std::istream& stream)
    : T ()
  {
    load (stream);
  }

  template <typename T>
  void
  BinaryReader<T>::load (const boost::filesystem::path& path)
  {
    load (boost::filesystem::ifstream (path));
  }

  template <typename T>
  void
  BinaryReader<T>::load (std::istream& stream)
  {
    BinaryReaderHelper helper (stream);
    helper >> *this;
  }

  template <typename T>
  BinaryReader<T>::~BinaryReader()
  {}

  template <typename T>
  BinaryWriter<T>::BinaryWriter (const patternGenerator_t& pg)
    : patternGenerator_ (pg)
  {}

  template <typename T>
  BinaryWriter<T>::~BinaryWriter ()
  {}

  template <typename T>
  void
  BinaryWriter<T>::write (const std::string& filename) const
  {
    boost::filesystem::path path (filename);
    write (path);
  }

  template <typename T>
  void
  BinaryWriter<T>::write (boost::filesystem::path& path) const
  {
    boost::filesystem::ofstream stream
      (path,
       boost::filesystem::ofstream::out
       | boost::filesystem::ofstream::trunc
       | boost::filesystem::ofstream::binary);
    write (stream);
  }

  template <typename T>
  void
  BinaryWriter<T>::write (std::ostream& stream) const
  {
    if (!stream.good ())
      throw std::runtime_error ("bad stream");

    BinaryWriterHelper helper (stream);
    helper << patternGenerator_;
  }

  template <typename T>
  BinaryWriterHelper&
  operator<< (BinaryWriterHelper& helper, const Eigen::PlainObjectBase<T>& matrix)
  {
    double cols = matrix.cols ();
    double rows = matrix.rows ();
    helper.stream.write(reinterpret_cast<const char*>(&cols), sizeof (double));
    helper.stream.write(reinterpret_cast<const char*>(&rows), sizeof (double));
    helper.stream.write(reinterpret_cast<const char*>(matrix.data()),
			cols * rows * sizeof (double));
    return helper;
  }

  template <typename T>
  BinaryWriterHelper&
    operator<< (BinaryWriterHelper& helper,
		const StampedFootprint<T>& footprint)
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

  template <typename T>
  BinaryWriterHelper&
  operator<< (BinaryWriterHelper& helper,
	      const WALK_INTERFACES_EIGEN_STL_VECTOR(StampedFootprint<T>)& footprints)
  {
    typename WALK_INTERFACES_EIGEN_STL_VECTOR(StampedFootprint<T>)::size_type
      countFootprints = footprints.size ();

    helper.stream.write
      (reinterpret_cast<const char*>(&countFootprints),
       sizeof (typename WALK_INTERFACES_EIGEN_STL_VECTOR
	       (StampedFootprint<T>)::size_type));

    BOOST_FOREACH (const StampedFootprint<T>& footprint, footprints)
      helper << footprint;
    return helper;
  }

  template <typename T>
  BinaryWriterHelper&
  operator<< (BinaryWriterHelper& helper,
	      const StampedPosition<T>& stampedPosition)
  {
    double t = 0. + stampedPosition.duration.total_nanoseconds () / 1e9;
    helper.stream.write(reinterpret_cast<const char*>(&t), sizeof (double));
    helper << stampedPosition.position;
    return helper;
  }

  template <typename T>
  BinaryWriterHelper&
  operator<< (BinaryWriterHelper& helper,
	      const DiscretizedTrajectory<T>& trajectory)
  {
    typename DiscretizedTrajectory<T>::data_t::size_type trajectorySize =
      trajectory.data ().size ();
    helper.stream.write
      (reinterpret_cast<const char*>(&trajectorySize),
       sizeof (typename DiscretizedTrajectory<T>::data_t::size_type));

    BOOST_FOREACH (const typename DiscretizedTrajectory<T>::element_t& data,
		   trajectory.data ())
      helper << data;
    return helper;
  }

  template <typename T>
  BinaryWriterHelper&
  operator<< (BinaryWriterHelper& helper, const PatternGenerator<T>& pg)
  {
    helper.stream.write(binaryFormatMagicNumber, sizeof (binaryFormatMagicNumber));
    helper.stream.write(binaryFormatVersion, sizeof (binaryFormatVersion));

    bool startWithLeftFoot = pg.startWithLeftFoot();
    helper.stream.write(reinterpret_cast<const char*>(&startWithLeftFoot),
			sizeof(bool));

    helper << pg.initialLeftFootPosition ()
	   << pg.initialRightFootPosition ()
	   << pg.initialCenterOfMassPosition ()
	   << pg.initialPosture ()
	   << pg.finalLeftFootPosition ()
	   << pg.finalRightFootPosition ()
	   << pg.finalCenterOfMassPosition ()
	   << pg.finalPosture ()
	   << pg.footprints ()
	   << pg.leftFootTrajectory ()
	   << pg.rightFootTrajectory ()
	   << pg.centerOfMassTrajectory ()
	   << pg.zmpTrajectory ()
	   << pg.postureTrajectory ();
    return helper;
  }

} // end of namespace walk.

#endif //! WALK_INTERFACE_BINARY_HXX
