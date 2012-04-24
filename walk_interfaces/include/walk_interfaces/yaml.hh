#ifndef WALK_INTERFACE_YAML_HH
# define WALK_INTERFACE_YAML_HH
# include <boost/filesystem/path.hpp>
# include <walk_interfaces/pattern-generator.hh>

# include "yaml-cpp/yaml.h"

namespace walk
{
  /// \brief Unserialize pattern generator data.
  template <typename T>
  void
  operator>> (const YAML::Node& node, PatternGenerator<T>& pg);

  /// \brief Unserialize data from a file into a specific pattern
  ///        generator.
  ///
  /// This class is a decorator over a pattern generator. It will fill
  /// the parent class information at construct time by reading data
  /// from a stream.
  ///
  /// If one wants to get the serialized data into a pattern generator
  /// whose type is Foo, it will instantiate a YamlReader<Foo>.
  ///
  /// \tparam T Pattern generator type (parent type)
  template <typename T>
  class YamlReader : public T
  {
  public:
    /// \brief Pattern generator / parent type.
    typedef T patternGenerator_t;

    /// \name Constructors and destructor.
    /// \{

    /// \brief Read the serialized data from a particular file on the
    /// filesystem.
    ///
    /// \param filename File path on the filesystem
    explicit YamlReader (const boost::filesystem::path& filename);

    /// \brief Read the serialized data from an input stream.
    ///
    /// \param stream Input stream.
    explicit YamlReader (std::istream& stream);

    template <typename A>
    explicit YamlReader (const boost::filesystem::path& filename, A a);
    template <typename A, typename B>
    explicit YamlReader (const boost::filesystem::path& filename, A a, B b);
    template <typename A, typename B, typename C>
    explicit YamlReader (const boost::filesystem::path& filename, A a, B b, C c);

    template <typename A>
    explicit YamlReader (std::istream& stream, A a);
    template <typename A, typename B>
    explicit YamlReader (std::istream& stream, A a, B b);
    template <typename A, typename B, typename C>
    explicit YamlReader (std::istream& stream, A a, B b, C c);



    /// \brief Destructor.
    ~YamlReader ();

    /// \}


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
    /// \brief Load the file from the filesystem.
    void load (const boost::filesystem::path& filename);
    /// \brief Load the file from an input stream.
    void load (std::istream& stream);
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  /// \brief Write a YAML file to serialize a pattern generator
  /// computed trajectories.
  ///
  /// This class takes a pattern generator as its input. The pattern
  /// generator is passed when instantiating the class. Then, using
  /// the write method, one can write a file on the filesystem of fill
  /// an output stream.
  ///
  /// The underlying pattern generator is kept as a constant
  /// reference. Therefore the pattern generator object life-time must
  /// exceed the YamlWriter instance life-time.
  ///
  /// \tparam T Pattern generator type.
  template <typename T>
  class YamlWriter
  {
  public:
    /// \brief Pattern generator type.
    typedef T patternGenerator_t;

    /// \name Constructor and destructor.
    /// \{

    /// \brief Default constructor.
    ///
    /// \param pg Pattern generator which trajectories will be serialized.
    explicit YamlWriter (const patternGenerator_t& pg);

    /// \brief Destructor.
    ~YamlWriter ();

    /// \}

    /// \brief Write the YAML data on the disk.
    ///
    /// \param filename Filename on the filesystem.
    void write (const std::string& filename) const;

    /// \brief Write the YAML data on the disk.
    ///
    /// \param filename Filename on the filesystem.
    void write (boost::filesystem::path& filename) const;

    /// \brief Write the YAML data into an output stream.
    ///
    /// \param stream Output stream into which the YAML data will be
    /// written.
    void write (std::ostream& stream) const;

  private:
    /// \brief Reference to the underlying pattern generator.
    const patternGenerator_t& patternGenerator_;
  };

} // end of namespace walk.

# include <walk_interfaces/yaml.hxx>
#endif //! WALK_INTERFACE_YAML_HH
