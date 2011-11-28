#include <iostream>
#include <sstream>
#include <gtest/gtest.h>

#include "walk_interfaces/stamped-position.hh"

TEST(TestStampedPosition, empty)
{
  walk::StampedPosition3d stampedPosition;
  stampedPosition.position.setIdentity();

  walk::HomogeneousMatrix3d I;
  I.setIdentity();

  walk::TimeDuration zero;
  EXPECT_EQ(zero, stampedPosition.duration);
  EXPECT_EQ(I, stampedPosition.position);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
