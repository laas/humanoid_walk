#ifndef WALK_INTERFACE_BINARY_HH
# define WALK_INTERFACE_BINARY_HH
# include <boost/filesystem/path.hpp>
# include <walk_interfaces/pattern-generator.hh>

namespace walk
{
  template <typename T>
  class BinaryReader : public T
  {
  public:
    typedef T patternGenerator_t;

    explicit BinaryReader (const boost::filesystem::path& filename);
    explicit BinaryReader (std::istream& filename);
    ~BinaryReader ();

    typename patternGenerator_t::Trajectory3d& leftFootTrajectory ()
    {
      return this->getLeftFootTrajectory ();
    }

    typename patternGenerator_t::Trajectory3d& rightFootTrajectory ()
    {
      return this->getRightFootTrajectory ();
    }

    typename patternGenerator_t::TrajectoryV2d& zmpTrajectory ()
    {
      return this->getZmpTrajectory ();
    }

    typename patternGenerator_t::TrajectoryV3d& centerOfMassTrajectory ()
    {
      return this->getCenterOfMassTrajectory ();
    }

    typename patternGenerator_t::TrajectoryNd& postureTrajectory ()
    {
      return this->getPostureTrajectory ();
    }

    HomogeneousMatrix3d& initialLeftFootPosition ()
    {
      return this->getInitialLeftFootPosition ();
    }

    HomogeneousMatrix3d& initialRightFootPosition ()
    {
      return this->getInitialRightFootPosition ();
    }

    Vector3d& initialCenterOfMassPosition ()
    {
      return this->getInitialCenterOfMassPosition ();
    }

    Posture& initialPosture ()
    {
      return this->getInitialPosture ();
    }

    HomogeneousMatrix3d& finalLeftFootPosition ()
    {
      return this->getFinalLeftFootPosition ();
    }

    HomogeneousMatrix3d& finalRightFootPosition ()
    {
      return this->getFinalRightFootPosition ();
    }

    Vector3d& finalCenterOfMassPosition ()
    {
      return this->getFinalCenterOfMassPosition ();
    }

    Posture& finalPosture ()
    {
      return this->getFinalPosture ();
    }

    virtual void computeTrajectories ()
    {}

  protected:
    void load (const boost::filesystem::path& filename);
    void load (std::istream& stream);
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  struct BinaryReaderHelper
  {
    BinaryReaderHelper (std::istream& stream)
      : stream (stream)
    {}
    std::istream& stream;
  };

  template <typename T>
  BinaryReaderHelper&
  operator>> (BinaryReaderHelper& helper, BinaryReader<T>& pg);



  template <typename T>
  class BinaryWriter
  {
  public:
    typedef T patternGenerator_t;

    explicit BinaryWriter (const patternGenerator_t& pg);
    ~BinaryWriter ();

    void write (const std::string& filename) const;
    void write (boost::filesystem::path& filename) const;
    void write (std::ostream& stream) const;

  private:
    /// \brief Reference to the underlying pattern generator.
    const patternGenerator_t& patternGenerator_;
  };

  struct BinaryWriterHelper
  {
    BinaryWriterHelper (std::ostream& s)
      : stream (s)
    {}
    std::ostream& stream;
  };

  template <typename T>
  BinaryWriterHelper&
  operator<< (BinaryWriterHelper& helper, const PatternGenerator<T>& pg);

} // end of namespace walk.

# include <walk_interfaces/binary.hxx>
#endif //! WALK_INTERFACE_BINARY_HH
