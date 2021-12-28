/**
 * @file test.cpp
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief for test
 * @version 0.1
 * @date 2021-12-26
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "gtest/gtest.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include <eigen3/Eigen/Dense>
#include "utility.h"

TEST(Utility, IsIntersect)
{
    Eigen::Vector2d p1(0, 0), p2(1, 1), p3(0, 1), p4(1, 0);
    int expect = 1;
    int result = Utility::IsIntersect(p1, p2, p3, p4);
    EXPECT_EQ(expect, result);
}