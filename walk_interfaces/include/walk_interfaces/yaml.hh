#ifndef WALK_INTERFACE_YAML_HH
# define WALK_INTERFACE_YAML_HH
# include <boost/filesystem/path.hpp>
# include <walk_interfaces/pattern-generator.hh>

# include "yaml-cpp/yaml.h"

namespace walk
{
  template <typename T>
  void
  operator>> (const YAML::Node& node, PatternGenerator<T>& pg);

  template <typename T>
  class YamlReader : public T
  {
  public:
    typedef T patternGenerator_t;

    explicit YamlReader (const boost::filesystem::path& filename);
    explicit YamlReader (std::istream& stream);

    ~YamlReader ();


    Trajectory3d& leftFootTrajectory ()
    {
      return this->getLeftFootTrajectory ();
    }

    Trajectory3d& rightFootTrajectory ()
    {
      return this->getRightFootTrajectory ();
    }

    TrajectoryV2d& zmpTrajectory ()
    {
      return this->getZmpTrajectory ();
    }

    TrajectoryV3d& centerOfMassTrajectory ()
    {
      return this->getCenterOfMassTrajectory ();
    }

    TrajectoryNd& postureTrajectory ()
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

  protected:
    void load (const boost::filesystem::path& filename);
    void load (std::istream& stream);
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  template <typename T>
  class YamlWriter
  {
  public:
    typedef T patternGenerator_t;

    explicit YamlWriter (const patternGenerator_t& pg);
    ~YamlWriter ();

    void write (const std::string& filename) const;
    void write (boost::filesystem::path& filename) const;
    void write (std::ostream& stream) const;

  private:
    const patternGenerator_t& patternGenerator_;
  };

} // end of namespace walk.

# include <walk_interfaces/yaml.hxx>
#endif //! WALK_INTERFACE_YAML_HH
