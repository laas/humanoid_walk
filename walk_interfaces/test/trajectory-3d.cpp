#include <iostream>
#include <sstream>
#include <gtest/gtest.h>

#include "walk_interfaces/trajectory.hh"

TEST(TestTrajectory, empty)
{
  walk::Trajectory3d gamma;
  walk::Trajectory3d::data_t data; // empty data.
  gamma.data() = data;

  walk::TimeDuration zero;
  EXPECT_EQ(zero, gamma.computeLength());
}

TEST(TestTrajectory, simple)
{
  using namespace boost::posix_time;

  walk::Trajectory3d gamma;
  walk::Trajectory3d::data_t data; // empty data.

  data.resize(2);

  data[0].duration = seconds(1);
  data[0].position <<
    1., 0., 0., 1.,
    0., 1., 0., 2.,
    0., 0., 1., 3.,
    0., 0., 0., 1.;

  data[1].duration = seconds(2);
  data[1].position <<
    1., 0., 0., 5.,
    0., 1., 0., 10.,
    0., 0., 1., 15.,
    0., 0., 0., 1.;

  gamma.data() = data;

  walk::TimeDuration zero = seconds(1 + 2);
  EXPECT_EQ(zero, gamma.computeLength());

  std::stringstream ss;
  ss << gamma;
  EXPECT_TRUE(!ss.str().empty());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
