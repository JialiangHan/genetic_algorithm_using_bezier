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
#include "genetic_algorithm.h"

TEST(Utility, DegNormalization)
{
    double t1 = 361, t2 = -359, t3 = 721, t4 = -719;
    std::vector<double> one{t1, t2, t3, t4};
    int expect, result;
    for (const auto &number : one)
    {
        expect = 1.0;
        result = Utility::DegNormalization(number);
        EXPECT_FLOAT_EQ(expect, result);
    }
}
TEST(Utility, RadNormalization)
{
    double t1 = 361, t2 = -359, t3 = 721, t4 = -719;
    std::vector<double> one{t1, t2, t3, t4};
    int expect, result;
    for (const auto &number : one)
    {
        expect = Utility::ConvertDegToRad(1.0);
        result = Utility::RadNormalization(Utility::ConvertDegToRad(number));
        EXPECT_FLOAT_EQ(expect, result);
    }
}
TEST(Utility, IsIntersect)
{
    Eigen::Vector2d p1(0, 0), p2(60, 0), p3(60, 30), p4(0, 30), p5(23.686, 10.8453);
    std::vector<Eigen::Vector2d> polygon{p1, p2, p3, p4, p1};
    std::vector<Eigen::Vector2d> intersect{p5};
    // std::vector<Eigen::Vector2d> outside{p5, p7, p8};
    int expect, result;
    for (const auto &point : intersect)
    {
        expect = 1;
        Eigen::Vector2d p9(10000, point.y());
        result = Utility::IsIntersect(point, p9, polygon[1], polygon[2]);
        EXPECT_EQ(expect, result);
    }
}

TEST(Utility, FindIntersectionPoint)
{
    Eigen::Vector2d p1(0, 0), p2(1, 1), p3(0, 1), p4(1, 0);
    Eigen::Vector2d expect(1, 1);
    Eigen::Vector2d result = Utility::FindIntersectionPoint(p1, p2, p3, p2);
    EXPECT_EQ(expect, result);
}

TEST(Utility, OnSegment)
{
    Eigen::Vector2d p1(0, 0), p2(1, 1), p3(0, 1), p4(1, 0), p5(0, 5), p6(0.5, 0), p7(-10, 1), p8(10, 1);
    std::vector<Eigen::Vector2d> polygon{p1, p2, p3, p4, p1};
    std::vector<Eigen::Vector2d> outside{p5, p7, p8};
    bool expect, result;
    for (const auto &point : outside)
    {
        expect = false;
        for (uint i = 0; i < polygon.size() - 1; ++i)
        {
            result = Utility::OnSegment(polygon[i], polygon[i + 1], point);
            EXPECT_EQ(expect, result);
        }
    }
}

TEST(Utility, IsAboveSegment)
{
    Eigen::Vector2d p1(0, 0), p2(1, 1), p3(0, 1), p4(1, 0);
    int expect = 1;
    int result = Utility::IsAboveSegment(p1, p2, p3);
    EXPECT_EQ(expect, result);
    expect = 0;
    result = Utility::IsAboveSegment(p1, p2, p4);
    EXPECT_EQ(expect, result);
}

TEST(Utility, IsInsidePolygon)
{
    Eigen::Vector2d p1(0, 0), p2(60, 0), p3(60, 30), p4(0, 30), p5(23.686, 10.8453);
    std::vector<Eigen::Vector2d> polygon{p1, p2, p3, p4, p1};
    std::vector<Eigen::Vector2d> inside{p5};
    // std::vector<Eigen::Vector2d> outside{p5, p7, p8};
    int expect, result;
    for (const auto &point : inside)
    {
        expect = 1;
        result = Utility::IsInsidePolygon(polygon, point);
        EXPECT_EQ(expect, result);
    }
}

