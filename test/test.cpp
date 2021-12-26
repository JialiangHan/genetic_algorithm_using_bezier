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

#include "bezier.h"
#include "matplotlibcpp.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include <eigen3/Eigen/Dense>

std::vector<float> GetCurvature(const std::vector<Eigen::Vector3d> &path)
{
    std::vector<float> curvature_vec;
    float curvature;
    // DLOG(INFO) << "In CalculateCurvature: " << topic_name << " path size is :" << path.size();

    // use three points to calculate curvature;
    for (uint i = 0; i < path.size() - 2; ++i)
    {
        //get three points from path
        Eigen::Vector2d xp(path[i].x(), path[i].y());
        // DLOG(INFO) << "xp x is :" << xp(0,0) << "y is: " << xp.y();
        Eigen::Vector2d xi(path[i + 1].x(), path[i + 1].y());
        // DLOG(INFO) << "xi x is :" << xi(0,0) << "y is: " << xi.y();
        Eigen::Vector2d xs(path[i + 2].x(), path[i + 2].y());
        if (xp == xi || xi == xs)
        {
            // DLOG(WARNING) << "In CalculateCurvature: some points are equal, skip these points for curvature calculation!!";
            continue;
        }
        // DLOG(INFO) << "xs x is :" << xs(0,0) << "y is: " << xs.y();
        //get two vector between these three nodes
        Eigen::Vector2d pre_vector = xi - xp;
        // DLOG(INFO) << "pre_vector x is :" << pre_vector(0,0) << "y is: " << pre_vector.y();
        Eigen::Vector2d succ_vector = xs - xi;
        // DLOG(INFO) << "succ_vector x is :" << succ_vector(0,0) << "y is: " << succ_vector.y();
        //calculate delta distance and delta angle
        double delta_distance = succ_vector.norm();
        double pre_vector_length = pre_vector.norm();
        // DLOG(INFO) << "delta_distance is:" << delta_distance;
        // DLOG(INFO) << "pre_vector_length is: " << pre_vector_length;
        // there would some calculation error here causing number inside acos greater than 1 or smaller than -1.
        double temp = pre_vector.dot(succ_vector) / (delta_distance * pre_vector_length);
        if (temp > 1 || temp < -1)
        {
            temp = round(temp);
        }
        double delta_angle = std::acos(temp);
        // DLOG(INFO) << "delta_angle is: " << delta_angle;
        //curvature = abs(delta_angle)/abs(delta_distance)
        curvature = delta_angle / delta_distance;
        curvature_vec.emplace_back(curvature);
        // DLOG(INFO) << "In CalculateCurvature:" << i << "th curvature is:" << curvature;
        if (std::isnan(curvature))
        {
            DLOG(WARNING) << " curvature is NAN!!!";
            if (std::isnan(delta_distance) || delta_distance == 0)
            {
                DLOG(INFO) << "delta_distance is:" << delta_distance;
                DLOG(INFO) << "succ_vector x is :" << succ_vector(0, 0) << "y is: " << succ_vector(1, 0);
                DLOG(INFO) << "xp x is :" << xp(0, 0) << "y is: " << xp(1, 0);
                DLOG(INFO) << "xi x is :" << xi(0, 0) << "y is: " << xi(1, 0);
            }
            else if (std::isnan(delta_angle))
            {
                DLOG(INFO) << "inside std::Cos is: " << temp;
                DLOG(INFO) << "pre_vector_length is: " << pre_vector_length;
                DLOG(INFO) << "xs x is :" << xs(0, 0) << "y is: " << xs(1, 0);
                DLOG(INFO) << "pre_vector x is :" << pre_vector(0, 0) << "y is: " << pre_vector(1, 0);
            }
        }
        // DLOG(INFO) << " in curvature_vec is:" << curvature_vec.back();
    }
    return curvature_vec;
}

std::vector<Eigen::Vector3d> GetPathFromBezier(const Bezier::Bezier<3> bezier)
{
    std::vector<Eigen::Vector3d> path;
    float i = 0.0;
    while (i <= 1)
    {
        Eigen::Vector3d point;
        point.x() = bezier.valueAt(i, 0);
        point.y() = bezier.valueAt(i, 1);
        path.emplace_back(point);
    }
    return path;
}

void Plot(const std::vector<Eigen::Vector3d> &path)
{
    matplotlibcpp::clf();
    matplotlibcpp::subplot(1, 2, 1);
    std::vector<float> x;
    std::vector<float> y;
    for (auto point : path)
    {
        x.emplace_back(point.x());
        y.emplace_back(point.y());
    }
    matplotlibcpp::plot(x, y, {{"label", "path"}});
    matplotlibcpp::legend({{"loc", "upper right"}});
    matplotlibcpp::title("path");
    matplotlibcpp::ylabel("path");
    // matplotlibcpp::ylim(0, 1);
    matplotlibcpp::grid(true);

    matplotlibcpp::subplot(1, 2, 2);
    std::vector<float> curvature_vec = GetCurvature(path);
    matplotlibcpp::plot(curvature_vec, {{"label", "curvature"}});
    matplotlibcpp::legend({{"loc", "upper right"}});
    matplotlibcpp::title("curvature");
    matplotlibcpp::ylabel("curvature");
    // matplotlibcpp::ylim(0, 1);
    matplotlibcpp::grid(true);
    matplotlibcpp::show();
}

int main(int argc, char **argv)
{
    Bezier::Bezier<3> cubicBezier({{120, 160}, {35, 200}, {220, 260}, {220, 40}});

    google::InitGoogleLogging(argv[0]);

    google::ParseCommandLineFlags(&argc, &argv, true);

    google::InstallFailureSignalHandler();

    std::vector<Eigen::Vector3d> path = GetPathFromBezier(cubicBezier);

    Plot(path);

    //    ros::init(argc, argv, "genetic_algorithm_using_beizer");

    //    GeneticAlgorithm::Planner planner;
    //    planner.MakePlan();

    //    ros::spin();
    return 0;
}