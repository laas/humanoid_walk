#ifndef HALFSTEPS_PATTERN_GENERATOR_HH
# define HALFSTEPS_PATTERN_GENERATOR_HH
# include <utility>
# include <walk_interfaces/pattern-generator.hh>

class HalfStepsPatternGenerator : public walk::PatternGenerator2d
{
public:
  typedef std::vector<std::pair<double, double> > slides_t;

  explicit HalfStepsPatternGenerator();
  explicit HalfStepsPatternGenerator(const HalfStepsPatternGenerator&);
  ~HalfStepsPatternGenerator();

  HalfStepsPatternGenerator& operator= (const HalfStepsPatternGenerator&);

  void setSlides(const slides_t& slides);

protected:
  virtual void computeTrajectories();

private:
  slides_t slides_;
};

#endif //! HALFSTEPS_PATTERN_GENERATOR_HH
