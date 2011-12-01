#ifndef WALK_INTERFACE_WALK_HH
# define WALK_INTERFACE_WALK_HH
# include <vector>

# include <walk_interfaces/types.hh>
# include <walk_interfaces/trajectory.hh>
# include <walk_interfaces/stamped-footprint.hh>

namespace walk
{
  template <typename T>
  class PatternGenerator;

  typedef PatternGenerator<StampedFootprint2d> PatternGenerator2d;

  template <typename T>
  class PatternGenerator
  {
  public:
    typedef T footprint_t;
    typedef WALK_INTERFACES_EIGEN_STL_VECTOR(footprint_t) footprints_t;

    explicit PatternGenerator();
    explicit PatternGenerator(const PatternGenerator<T>&);
    virtual ~PatternGenerator();

    PatternGenerator<T>& operator=(const PatternGenerator<T>&);

    void setInitialRobotPosition(const HomogeneousMatrix3d& leftFoot,
				 const HomogeneousMatrix3d& rightFoot,
				 const Vector3d& centerOfMass,
				 const Posture& posture);

    void setFinalRobotPosition(const HomogeneousMatrix3d& leftFoot,
			       const HomogeneousMatrix3d& rightFoot,
			       const Vector3d& centerOfMass,
			       const Posture& posture);

    void setFootprints(const footprints_t&, bool startWithLeftFoot);
    const footprints_t& footprints() const;
    bool startWithLeftFoot() const;

    const Trajectory3d& leftFootTrajectory() const;
    const Trajectory3d& rightFootTrajectory() const;
    const TrajectoryV2d& zmpTrajectory() const;
    const TrajectoryV3d& centerOfMassTrajectory() const;
    const TrajectoryNd& postureTrajectory() const;

    const HomogeneousMatrix3d& initialLeftFootPosition() const;
    const HomogeneousMatrix3d& initialRightFootPosition() const;
    const Vector3d& initialCenterOfMassPosition() const;
    const Posture& initialPosture() const;

    const HomogeneousMatrix3d& finalLeftFootPosition() const;
    const HomogeneousMatrix3d& finalRightFootPosition() const;
    const Vector3d& finalCenterOfMassPosition() const;
    const Posture& finalPosture() const;

  protected:
    Trajectory3d& getLeftFootTrajectory();
    Trajectory3d& getRightFootTrajectory();
    TrajectoryV2d& getZmpTrajectory();
    TrajectoryV3d& getCenterOfMassTrajectory();
    TrajectoryNd& getPostureTrajectory();

    HomogeneousMatrix3d& getInitialLeftFootPosition();
    HomogeneousMatrix3d& getInitialRightFootPosition();
    Vector3d& getInitialCenterOfMassPosition();
    Posture& getInitialPosture();

    HomogeneousMatrix3d& getFinalLeftFootPosition();
    HomogeneousMatrix3d& getFinalRightFootPosition();
    Vector3d& getFinalCenterOfMassPosition();
    Posture& getFinalPosture();


    virtual void computeTrajectories() = 0;

  private:
    footprints_t footprints_;
    bool startWithLeftFoot_;
    Trajectory3d leftFootTrajectory_;
    Trajectory3d rightFootTrajectory_;
    TrajectoryV3d centerOfMassTrajectory_;
    TrajectoryV2d zmpTrajectory_;
    TrajectoryNd postureTrajectory_;

    HomogeneousMatrix3d initialLeftFootPosition_;
    HomogeneousMatrix3d initialRightFootPosition_;
    Vector3d initialCenterOfMassPosition_;
    Posture initialPosture_;

    HomogeneousMatrix3d finalLeftFootPosition_;
    HomogeneousMatrix3d finalRightFootPosition_;
    Vector3d finalCenterOfMassPosition_;
    Posture finalPosture_;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  template <typename T>
  std::ostream& operator<<(std::ostream&, const PatternGenerator<T>&);
} // end of namespace walk.

# include <walk_interfaces/pattern-generator.hxx>
#endif //! WALK_INTERFACE_WALK_HH
