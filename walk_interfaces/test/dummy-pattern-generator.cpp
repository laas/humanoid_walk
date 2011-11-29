#include <iostream>
#include <sstream>
#include <gtest/gtest.h>

#include "walk_interfaces/pattern-generator.hh"
#include "walk_interfaces/yaml.hh"

class MyPatternGenerator : public walk::PatternGenerator2d
{
public:
  MyPatternGenerator()
    : walk::PatternGenerator2d()
  {}

  void computeTrajectories()
  {
    walk::TimeDuration l =
      walk::computeFootstepSequenceLength(steps());

    walk::Trajectory3d& lf = getLeftFootTrajectory();
    walk::Trajectory3d& rf = getRightFootTrajectory();
    walk::TrajectoryV3d& com = getCenterOfMassTrajectory();
    walk::TrajectoryV2d& zmp = getZmpTrajectory();

    lf.data().resize(1);
    rf.data().resize(1);
    com.data().resize(1);
    zmp.data().resize(1);

    lf.data()[0].duration = l;
    rf.data()[0].duration = l;
    com.data()[0].duration = l;
    zmp.data()[0].duration = l;

    lf.data()[0].position = initialLeftFootPosition();
    rf.data()[0].position = initialRightFootPosition();
    com.data()[0].position = initialCenterOfMassPosition();

    walk::Vector2d initialZmp;
    initialZmp[0] = initialZmp[1] = 0.;
    zmp.data()[0].position = initialZmp;
  }
};

TEST(TestStampedPosition, empty)
{
  MyPatternGenerator pg;
  walk::StampedFootstep2dSequence steps;
  pg.setSteps(steps, true);

  const walk::Trajectory3d& lf = pg.leftFootTrajectory();

  walk::TimeDuration zero;
  walk::TimeDuration lengthLf = lf.computeLength();

  EXPECT_EQ(zero, lengthLf);

  walk::YamlWriter<MyPatternGenerator> writer (pg);
  writer.write("/tmp/test.yaml");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
