#ifndef WALK_INTERFACE_WALK_HH
# define WALK_INTERFACE_WALK_HH
# include <vector>

# include <walk_interfaces/types.hh>
# include <walk_interfaces/trajectory.hh>
# include <walk_interfaces/stamped-footstep.hh>

namespace walk
{
  template <typename T>
  class PatternGenerator;

  typedef PatternGenerator<StampedFootstep2d> PatternGenerator2d;

  template <typename T>
  class PatternGenerator
  {
  public:
    typedef T footstep_t;
    typedef WALK_INTERFACES_EIGEN_STL_VECTOR(footstep_t) footsteps_t;

    explicit PatternGenerator();
    explicit PatternGenerator(const PatternGenerator<T>&);
    virtual ~PatternGenerator();

    PatternGenerator<T>& operator=(const PatternGenerator<T>&);

    void setInitialRobotPosition(const HomogeneousMatrix& leftFoot,
				 const HomogeneousMatrix& rightFoot,
				 const HomogeneousMatrix& centerOfMass,
				 const Posture& posture);

    void setFinalRobotPosition(const HomogeneousMatrix& leftFoot,
			       const HomogeneousMatrix& rightFoot,
			       const HomogeneousMatrix& centerOfMass,
			       const Posture& posture);

    void setSteps(const footsteps_t&, bool startWithLeftFoot);
    const footsteps_t& steps() const;
    bool startWithLeftFoot() const;

    const Trajectory3d& leftFootTrajectory() const;
    const Trajectory3d& rightFootTrajectory() const;
    const Trajectory3d& zmpTrajectory() const;
    const Trajectory3d& centerOfMassTrajectory() const;
    const Trajectory3d& postureTrajectory() const;

    const HomogeneousMatrix& initialLeftFootPosition() const;
    const HomogeneousMatrix& initialRightFootPosition() const;
    const HomogeneousMatrix& initialCenterOfMassPosition() const;
    const Posture& initialPosture() const;

    const HomogeneousMatrix& finalLeftFootPosition() const;
    const HomogeneousMatrix& finalRightFootPosition() const;
    const HomogeneousMatrix& finalCenterOfMassPosition() const;
    const Posture& finalPosture() const;

  protected:
    Trajectory3d& getLeftFootTrajectory();
    Trajectory3d& getRightFootTrajectory();
    Trajectory3d& getZmpTrajectory();
    Trajectory3d& getCenterOfMassTrajectory();
    Trajectory3d& getPostureTrajectory();

    virtual void computeTrajectories() = 0;

  private:
    footsteps_t steps_;
    bool startWithLeftFoot_;
    Trajectory3d leftFootTrajectory_;
    Trajectory3d rightFootTrajectory_;
    Trajectory3d centerOfMassTrajectory_;
    Trajectory3d zmpTrajectory_;
    Trajectory3d postureTrajectory_;

    HomogeneousMatrix initialLeftFootPosition_;
    HomogeneousMatrix initialRightFootPosition_;
    HomogeneousMatrix initialCenterOfMassPosition_;
    Posture initialPosture_;

    HomogeneousMatrix finalLeftFootPosition_;
    HomogeneousMatrix finalRightFootPosition_;
    HomogeneousMatrix finalCenterOfMassPosition_;
    Posture finalPosture_;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  template <typename T>
  std::ostream& operator<<(std::ostream&, const PatternGenerator<T>&);
} // end of namespace walk.

# include <walk_interfaces/pattern-generator.hxx>
#endif //! WALK_INTERFACE_WALK_HH
