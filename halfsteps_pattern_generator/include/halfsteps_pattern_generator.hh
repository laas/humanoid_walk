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

typedef WithHalfStepsAdditionalData<walk::StampedFootprint2d> footprint_t;
typedef walk::PatternGenerator<footprint_t> halfStepsPgParent_t;

class HalfStepsPatternGenerator : public halfStepsPgParent_t
{
public:
  explicit HalfStepsPatternGenerator();
  explicit HalfStepsPatternGenerator(const HalfStepsPatternGenerator&);
  ~HalfStepsPatternGenerator();

  HalfStepsPatternGenerator& operator= (const HalfStepsPatternGenerator&);

protected:
  virtual void computeTrajectories();
};

#endif //! HALFSTEPS_PATTERN_GENERATOR_HH
