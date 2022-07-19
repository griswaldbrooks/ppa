#include "say-hello/speaker.hpp"  // For speaker

#include "gtest/gtest.h"                   // For TEST_*

namespace sh::test_hello {

TEST(Speaker, Decorate) {
  auto const sayer = speaker{"tacos"};
  EXPECT_EQ(sayer(), "I want tacos");
}

}  // namespace sh::test_hello

// int main(int argc, char **argv) {
//     testing::InitGoogleTest(&argc, argv);
//     return RUN_ALL_TESTS();
// }
