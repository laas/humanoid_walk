#include <iostream>
#include <sstream>
#include <gtest/gtest.h>

#include "halfsteps_pattern_generator.hh"
#include "halfsteps_pattern_generator/Footprint.h"
#include "yaml.hh"
#include "binary.hh"


TEST(TestStampedPosition, empty)
{
  HalfStepsPatternGenerator pg(0.95, 1.05, 0.005);
  HalfStepsPatternGenerator::footprints_t footprints;

  HalfStepsPatternGenerator::footprint_t footprint;

  using namespace boost::posix_time;
  using namespace boost::gregorian;
  walk::TimeDuration d (milliseconds (5));
  footprint.duration = d;
  footprint.beginTime = walk::Time (date(1970,1,1), seconds(1));
  footprints.push_back (footprint);

  pg.setFootprints(footprints, true);

  const HalfStepsPatternGenerator::Trajectory3d& lf = pg.leftFootTrajectory();

  walk::TimeDuration lengthLf = lf.computeLength();

  walk::YamlWriter<HalfStepsPatternGenerator> writer (pg);
  writer.write("/tmp/test.yaml");

  walk::BinaryWriter<HalfStepsPatternGenerator> binaryWriter (pg);
  binaryWriter.write("/tmp/test.bin");

  //walk::YamlReader<halfStepsPgParent_t> reader ("/tmp/test.yaml");

  // walk::YamlWriter<walk::YamlReader<halfStepsPgParent_t> > writer2 (reader);
  // writer2.write("/tmp/test2.yaml");

  // walk::YamlReader<halfStepsPgParent_t> reader2 ("/tmp/test.yaml");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
