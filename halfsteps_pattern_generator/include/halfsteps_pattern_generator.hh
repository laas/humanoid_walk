#ifndef HALFSTEPS_PATTERN_GENERATOR_HH
# define HALFSTEPS_PATTERN_GENERATOR_HH
# include <utility>
# include <walk_interfaces/pattern-generator.hh>

template <typename T>
struct WithHalfStepsAdditionalData : public T
{
  explicit WithHalfStepsAdditionalData()
    : T(),
      slideUp(0.),
      slideDown(0.),
      horizontalDistance(0.),
      stepHeight(0.)
  {}

  double slideUp;
  double slideDown;
  double horizontalDistance;
  double stepHeight;
};

class HalfStepsPatternGenerator;

namespace walk
{
  template <>
  struct PatternGeneratorTraits<HalfStepsPatternGenerator>
  {
    /// \brief Footprint definition.
    typedef WithHalfStepsAdditionalData<walk::StampedFootprint2d> Footprint;
    
    /// \brief Trajectory in \f$SO(3)\f$.
    typedef DiscretizedTrajectory<StampedPosition3d> Trajectory3d;
    /// \brief Trajectory in \f$SO(2)\f$.
    typedef DiscretizedTrajectory<StampedPosition2d> Trajectory2d;
    /// \brief Trajectory in \f$R^2\f$.
    typedef DiscretizedTrajectory<StampedVector2d> TrajectoryV2d;
    /// \brief Trajectory in \f$R^3\f$.
    typedef DiscretizedTrajectory<StampedVector3d> TrajectoryV3d;
    /// \brief Trajectory in \f$R^n\f$.
    typedef DiscretizedTrajectory<StampedVectorNd> TrajectoryNd;
  };
} // end of namespace walk.

class HalfStepsPatternGenerator
  : public walk::PatternGenerator<HalfStepsPatternGenerator>
{
public:
  explicit HalfStepsPatternGenerator
  (const double& timeBeforeZmpShift,
   const double& timeAfterZmpShift,
   const double& step);
  explicit HalfStepsPatternGenerator(const HalfStepsPatternGenerator&);
  ~HalfStepsPatternGenerator();

  HalfStepsPatternGenerator& operator= (const HalfStepsPatternGenerator&);

protected:
  virtual void computeTrajectories();

private:
  double timeBeforeZmpShift_;
  double timeAfterZmpShift_;
  double step_;
};

#endif //! HALFSTEPS_PATTERN_GENERATOR_HH
