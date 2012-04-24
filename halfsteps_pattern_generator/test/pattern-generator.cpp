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

  std::cout << "--- original pattern generator" << std::endl
	    << pg << std::endl;

  walk::YamlWriter<HalfStepsPatternGenerator> writer (pg);
  writer.write("/tmp/test.yaml");

  walk::BinaryWriter<HalfStepsPatternGenerator> binaryWriter (pg);
  binaryWriter.write("/tmp/test.bin");

  walk::BinaryReader<HalfStepsPatternGenerator> binaryReader ("/tmp/test.bin",
							      0.95, 1.05, 0.005);

  std::cout << "--- pattern generator after binary reading" << std::endl
	    << pg << std::endl;

  walk::BinaryWriter<HalfStepsPatternGenerator> binaryWriter2 (binaryReader);
  binaryWriter2.write("/tmp/test2.bin");

  // walk::YamlReader<HalfStepsPatternGenerator> reader ("/tmp/test.yaml",
  // 						      0.95, 1.05, 0.005);

  // walk::YamlWriter<walk::YamlReader<HalfStepsPatternGenerator> > writer2 (reader);
  // writer2.write("/tmp/test2.yaml");

  // walk::YamlReader<HalfStepsPatternGenerator> reader2 ("/tmp/test.yaml",
  // 						       0.95, 1.05, 0.005);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
