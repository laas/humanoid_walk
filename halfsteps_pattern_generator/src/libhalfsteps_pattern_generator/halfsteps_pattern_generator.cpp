#include <stdexcept>
#include <boost/date_time.hpp>
#include <boost/math/constants/constants.hpp>

#include "halfsteps_pattern_generator.hh"
#include "newPGstepStudy.h"

static const double g = 9.81;
static const double timeBeforeZmpShift = 0.95;
static const double timeAfterZmpShift = 1.05;
static const double halfStepLength = 2.;
static const double STEP = 0.005;


HalfStepsPatternGenerator::HalfStepsPatternGenerator()
  : walk::PatternGenerator2d(),
    slides_()
{}

HalfStepsPatternGenerator::HalfStepsPatternGenerator
(const HalfStepsPatternGenerator& pg)
  : walk::PatternGenerator2d(),
    slides_(pg.slides_)
{}

HalfStepsPatternGenerator::~HalfStepsPatternGenerator()
{}

HalfStepsPatternGenerator&
HalfStepsPatternGenerator::operator= (const HalfStepsPatternGenerator& pg)
{
  if (&pg == this)
    return *this;

  slides_ = pg.slides_;

  return *this;
}

void
HalfStepsPatternGenerator::setSlides(const slides_t& slides)
{
  // Make sure that slides coefficients are valid.
  slides_t::const_iterator iter = slides.begin();
  for (; iter != slides.end(); ++iter)
    {
      if (iter->first < -1.52 || iter->first > 0.)
	throw std::runtime_error ("invalid first slide");
      if (iter->second < -0.76 || iter->second > 0.)
	throw std::runtime_error ("invalid second slide");
    }

  slides_ = slides;
  this->computeTrajectories();
}

void
HalfStepsPatternGenerator::computeTrajectories()
{
  if (slides_.empty())
    return;

  if (slides_.size () != steps().size())
    throw std::runtime_error("slides and steps size does not match");

  CnewPGstepStudy pg;
  StepFeatures stepFeatures;

  double comZ = initialCenterOfMassPosition() (2, 3);

  Eigen::Matrix<double, 6, 1> initialStep;
  initialStep(0) =
    std::fabs(initialLeftFootPosition() (0, 3)
	      - initialRightFootPosition() (0, 3)) / 2.;
  initialStep(1) =
    std::fabs(initialLeftFootPosition() (1, 3)
	      - initialRightFootPosition() (1, 3)) / 2.;
  initialStep(2) = 0.;

  initialStep(3) = -initialStep (0);
  initialStep(4) = -initialStep (1);
  initialStep(5) = std::atan2(initialRightFootPosition() (1,0),
			      initialRightFootPosition() (0,0));

  std::vector<double> stepData;
  for (unsigned i = 0; i < 6; ++i)
    stepData.push_back(initialStep (i));

  for (unsigned i = 0; i < this->steps().size (); ++i)
    {
      stepData.push_back(this->slides_[i].first);
      stepData.push_back(0.31); // hor_distance
      stepData.push_back(0.15); // max height
      stepData.push_back(this->slides_[i].second);
      stepData.push_back(this->steps()[i].position(0));
      stepData.push_back(this->steps()[i].position(1));
      stepData.push_back(this->steps()[i].position(2) * 180. / M_PI);
    }

  pg.produceSeqSlidedHalfStepFeatures
    (stepFeatures, STEP, comZ, g,
     timeBeforeZmpShift, timeAfterZmpShift, halfStepLength, stepData,
     startWithLeftFoot() ? 'L' : 'R');

  getLeftFootTrajectory().data().resize(stepFeatures.size);
  getRightFootTrajectory().data().resize(stepFeatures.size);
  getCenterOfMassTrajectory().data().resize(stepFeatures.size);
  getZmpTrajectory().data().resize(stepFeatures.size);
  getPostureTrajectory().data().resize(stepFeatures.size);

  using boost::posix_time::seconds;

  for (unsigned i = 0; i < stepFeatures.size; ++i)
    {
      // Left foot.
      getLeftFootTrajectory().data()[i].duration = seconds(STEP);
      getLeftFootTrajectory().data()[i].position.setIdentity();
      getLeftFootTrajectory().data()[i].position (0,3) =
	stepFeatures.leftfootXtraj[i];
      getLeftFootTrajectory().data()[i].position (1,3) =
	stepFeatures.leftfootYtraj[i];
      getLeftFootTrajectory().data()[i].position (2,3) =
	stepFeatures.leftfootHeight[i];

      double theta = stepFeatures.leftfootOrient[i] * M_PI / 180.;
      getLeftFootTrajectory().data()[i].position (0,0) = std::cos(theta);
      getLeftFootTrajectory().data()[i].position (0,1) = -std::sin(theta);
      getLeftFootTrajectory().data()[i].position (1,0) = std::sin(theta);
      getLeftFootTrajectory().data()[i].position (1,1) = std::cos(theta);

      // Right foot.
      getRightFootTrajectory().data()[i].duration = seconds(STEP);
      getRightFootTrajectory().data()[i].position.setIdentity();
      getRightFootTrajectory().data()[i].position (0,3) =
	stepFeatures.rightfootXtraj[i];
      getRightFootTrajectory().data()[i].position (1,3) =
	stepFeatures.rightfootYtraj[i];
      getRightFootTrajectory().data()[i].position (2,3) =
	stepFeatures.rightfootHeight[i];

      theta = stepFeatures.rightfootOrient[i] * M_PI / 180.;
      getRightFootTrajectory().data()[i].position (0,0) = std::cos(theta);
      getRightFootTrajectory().data()[i].position (0,1) = -std::sin(theta);
      getRightFootTrajectory().data()[i].position (1,0) = std::sin(theta);
      getRightFootTrajectory().data()[i].position (1,1) = std::cos(theta);

      // Center of mass
      getCenterOfMassTrajectory().data()[i].duration = seconds(STEP);
      getCenterOfMassTrajectory().data()[i].position.setIdentity();
      getCenterOfMassTrajectory().data()[i].position (0,3) =
	stepFeatures.comTrajX[i];
      getCenterOfMassTrajectory().data()[i].position (1,3) =
	stepFeatures.comTrajY[i];
      getCenterOfMassTrajectory().data()[i].position (2,3) = comZ;

      // ZMP
      getZmpTrajectory().data()[i].duration = seconds(STEP);
      getZmpTrajectory().data()[i].position.setIdentity();
      getZmpTrajectory().data()[i].position[0] = stepFeatures.zmpTrajX[i];
      getZmpTrajectory().data()[i].position[1] = stepFeatures.zmpTrajY[i];

      //Posture
      //FIXME:
    }
}
