#ifndef WALK_INTERFACE_PATTERN_GENERATOR_HXX
# define WALK_INTERFACE_PATTERN_GENERATOR_HXX
# include <iostream>

namespace walk
{
  template <typename T>
  PatternGenerator<T>::PatternGenerator()
    : steps_ (),
      startWithLeftFoot_ (true),
      leftFootTrajectory_(),
      rightFootTrajectory_(),
      centerOfMassTrajectory_(),
      zmpTrajectory_(),
      postureTrajectory_(),

      initialLeftFootPosition_(),
      initialRightFootPosition_(),
      initialCenterOfMassPosition_(),
      initialPosture_(),

      finalLeftFootPosition_(),
      finalRightFootPosition_(),
      finalCenterOfMassPosition_(),
      finalPosture_()
  {
    initialLeftFootPosition_.setIdentity();
    initialRightFootPosition_.setIdentity();
    initialCenterOfMassPosition_.setIdentity();
    initialPosture_.setZero();

    finalLeftFootPosition_.setIdentity();
    finalRightFootPosition_.setIdentity();
    finalCenterOfMassPosition_.setIdentity();
    finalPosture_.setZero();
  }

  template <typename T>
  PatternGenerator<T>::PatternGenerator(const PatternGenerator<T>& pg)
    : steps_ (pg.steps_),
      startWithLeftFoot_ (pg.startWithLeftFoot_),
      leftFootTrajectory_(pg.leftFootTrajectory_),
      rightFootTrajectory_(pg.rightFootTrajectory_),
      centerOfMassTrajectory_(pg.centerOfMassTrajectory_),
      zmpTrajectory_(pg.zmpTrajectory_),
      postureTrajectory_(pg.postureTrajectory_)
  {}

  template <typename T>
  PatternGenerator<T>&
  PatternGenerator<T>::operator=(const PatternGenerator<T>& pg)
  {
    if (&pg == this)
      return *this;
    this->steps_ = pg.steps_;
    this->startWithLeftFoot_ = pg.startWithLeftFoot_;
    this->leftFootTrajectory_ = pg.leftFootTrajectory_;
    this->rightFootTrajectory_ = pg.rightFootTrajectory_;
    this->centerOfMassTrajectory_ = pg.centerOfMassTrajectory_;
    this->zmpTrajectory_ = pg.zmpTrajectory_;
    this->postureTrajectory_ = pg.postureTrajectory_;
    return *this;
  }

  template <typename T>
  void
  PatternGenerator<T>::setInitialRobotPosition
  (const HomogeneousMatrix3d& leftFoot,
   const HomogeneousMatrix3d& rightFoot,
   const HomogeneousMatrix3d& centerOfMass,
   const Posture& posture)
  {
    initialLeftFootPosition_ = leftFoot;
    initialRightFootPosition_ = rightFoot;
    initialCenterOfMassPosition_ = centerOfMass;
    initialPosture_ = posture;
  }

  template <typename T>
  void
  PatternGenerator<T>::setFinalRobotPosition
  (const HomogeneousMatrix3d& leftFoot,
   const HomogeneousMatrix3d& rightFoot,
   const HomogeneousMatrix3d& centerOfMass,
   const Posture& posture)
  {
    finalLeftFootPosition_ = leftFoot;
    finalRightFootPosition_ = rightFoot;
    finalCenterOfMassPosition_ = centerOfMass;
    finalPosture_ = posture;
  }


  template <typename T>
  const typename PatternGenerator<T>::footsteps_t&
  PatternGenerator<T>::steps() const
  {
    return steps_;
  }

  template <typename T>
  void
  PatternGenerator<T>::setSteps(const footsteps_t& steps, bool startWithLeftFoot)
  {
    steps_ = steps;
    startWithLeftFoot_ = startWithLeftFoot;
    this->computeTrajectories();
  }

  template <typename T>
  bool
  PatternGenerator<T>::startWithLeftFoot() const
  {
    return startWithLeftFoot_;
  }

  template <typename T>
  PatternGenerator<T>::~PatternGenerator()
  {}

  template <typename T>
  const Trajectory3d&
  PatternGenerator<T>::leftFootTrajectory() const
  {
    return leftFootTrajectory_;
  }

  template <typename T>
  const Trajectory3d&
  PatternGenerator<T>::rightFootTrajectory() const
  {
    return rightFootTrajectory_;
  }

  template <typename T>
  const TrajectoryV2d&
  PatternGenerator<T>::zmpTrajectory() const
  {
    return zmpTrajectory_;
  }

  template <typename T>
  const Trajectory3d&
  PatternGenerator<T>::centerOfMassTrajectory() const
  {
    return centerOfMassTrajectory_;
  }

  template <typename T>
  const TrajectoryNd&
  PatternGenerator<T>::postureTrajectory() const
  {
    return postureTrajectory_;
  }

  template <typename T>
  Trajectory3d&
  PatternGenerator<T>::getLeftFootTrajectory()
  {
    return leftFootTrajectory_;
  }

  template <typename T>
  Trajectory3d&
  PatternGenerator<T>::getRightFootTrajectory()
  {
    return rightFootTrajectory_;
  }

  template <typename T>
  TrajectoryV2d&
  PatternGenerator<T>::getZmpTrajectory()
  {
    return zmpTrajectory_;
  }

  template <typename T>
  Trajectory3d&
  PatternGenerator<T>::getCenterOfMassTrajectory()
  {
    return centerOfMassTrajectory_;
  }

  template <typename T>
  TrajectoryNd&
  PatternGenerator<T>::getPostureTrajectory()
  {
    return postureTrajectory_;
  }

  template <typename T>
  const HomogeneousMatrix3d&
  PatternGenerator<T>::initialLeftFootPosition() const
  {
    return initialLeftFootPosition_;
  }

  template <typename T>
  const HomogeneousMatrix3d&
  PatternGenerator<T>::initialRightFootPosition() const
  {
    return initialRightFootPosition_;
  }

  template <typename T>
  const HomogeneousMatrix3d&
  PatternGenerator<T>::initialCenterOfMassPosition() const
  {
    return initialCenterOfMassPosition_;
  }

  template <typename T>
  const Posture&
  PatternGenerator<T>::initialPosture() const
  {
    return initialPosture_;
  }

  template <typename T>
  const HomogeneousMatrix3d&
  PatternGenerator<T>::finalLeftFootPosition() const
  {
    return finalLeftFootPosition_;
  }

  template <typename T>
  const HomogeneousMatrix3d&
  PatternGenerator<T>::finalRightFootPosition() const
  {
    return finalRightFootPosition_;
  }

  template <typename T>
  const HomogeneousMatrix3d&
  PatternGenerator<T>::finalCenterOfMassPosition() const
  {
    return finalCenterOfMassPosition_;
  }

  template <typename T>
  const Posture&
  PatternGenerator<T>::finalPosture() const
  {
    return finalPosture_;
  }

  template <typename T>
  std::ostream&
  operator<<(std::ostream& os, const PatternGenerator<T>& pg)
  {
    os
      << "initial left foot position:\n"
      << pg.initialLeftFootPosition()
      << "initial right foot position:\n"
      << pg.initialRightFootPosition()
      << "initial center of mass position:\n"
      << pg.initialCenterOfMassPosition()
      << "initial posture:\n"
      << pg.initialPosture()

      << "final left foot position:\n"
      << pg.finalLeftFootPosition()
      << "final right foot position:\n"
      << pg.finalRightFootPosition()
      << "final center of mass position:\n"
      << pg.finalCenterOfMassPosition()
      << "final posture:\n"
      << pg.finalPosture()

      << "left foot trajectory:\n"
      << pg.leftFootTrajectory()
      << "right foot trajectory:\n"
      << pg.rightFootTrajectory()
      << "zmp trajectory:\n"
      << pg.zmpTrajectory()
      << "center of mass trajectory:\n"
      << pg.centerOfMassTrajectory();
    return os;
  }
} // end of namespace walk.

#endif //! WALK_INTERFACE_PATTERN_GENERATOR_HXX
