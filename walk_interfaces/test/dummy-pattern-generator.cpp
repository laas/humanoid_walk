#include <iostream>
#include <sstream>
#include <gtest/gtest.h>

#include "walk_interfaces/pattern-generator.hh"
#include "walk_interfaces/binary.hh"
#include "walk_interfaces/yaml.hh"

class MyPatternGenerator : public walk::DiscretizedPatternGenerator2d
{
public:
  MyPatternGenerator()
    : walk::DiscretizedPatternGenerator2d()
  {}

  void computeTrajectories()
  {
    walk::TimeDuration l =
      walk::computeFootprintSequenceLength(footprints());

    Trajectory3d& lf = getLeftFootTrajectory();
    Trajectory3d& rf = getRightFootTrajectory();
    TrajectoryV3d& com = getCenterOfMassTrajectory();
    TrajectoryV2d& zmp = getZmpTrajectory();

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
  walk::StampedFootprint2dSequence footprints;

  walk::StampedFootprint2d footprint;

  using namespace boost::posix_time;
  using namespace boost::gregorian;
  walk::TimeDuration d (milliseconds (5));
  footprint.duration = d;
  footprint.beginTime = walk::Time (date(1970,1,1), seconds(1));
  footprints.push_back (footprint);

  pg.setFootprints(footprints, true);

  const MyPatternGenerator::Trajectory3d& lf = pg.leftFootTrajectory();

  walk::TimeDuration lengthLf = lf.computeLength();

  EXPECT_EQ(d, lengthLf);

  walk::YamlWriter<MyPatternGenerator> writer (pg);
  writer.write("/tmp/test.yaml");

  walk::BinaryWriter<MyPatternGenerator> binaryWriter (pg);
  binaryWriter.write("/tmp/test.bin");

  walk::YamlReader<MyPatternGenerator> reader ("/tmp/test.yaml");

  walk::BinaryReader<MyPatternGenerator> binaryReader ("/tmp/test.bin");

  walk::YamlWriter<walk::YamlReader<MyPatternGenerator> > writer2 (reader);
  writer2.write("/tmp/test2.yaml");

  walk::YamlReader<MyPatternGenerator> reader2 ("/tmp/test.yaml");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
