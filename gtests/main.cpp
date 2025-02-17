#include <gtest/gtest.h>
#include <urg.hpp>

// Demonstrate some basic assertions.
TEST(urg, dummy) {
  // Expect two strings not to be equal.
  EXPECT_STRNE("hello", "world");
  // Expect equality.
  EXPECT_EQ(7 * 6, 42);
}