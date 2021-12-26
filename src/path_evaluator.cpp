/**
 * @file path_evaluator.cpp
 * @author Jialiang Han
 * @brief this file is to evaluate path from planner, using metrics: curvature, smoothness,....maybe more.
 * @version 0.2
 * @date 2021-12-17
 * 
 * @copyright Copyright (c) 2021
 * 
 **/
#include "path_evaluator.h"
#include <cmath>
#include "matplotlibcpp.h"
#include "utility.h"
#include <algorithm>

namespace PathEvaluator
{
    int PathEvaluator::CalculateCurvature(const std::vector<Eigen::Vector3d> &path, const std::string &topic_name)
    {
        if (path.size() < 3)
        {
            DLOG(WARNING) << "In CalculateCurvature: path does not have enough points!!!";
            return 0;
        }

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
                DLOG(WARNING) << "In CalculateCurvature: some points are equal, skip these points for curvature calculation!!";
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
        if (curvature_map_.count(topic_name) > 0)
        {
            curvature_map_.at(topic_name).clear();
            curvature_map_.at(topic_name) = curvature_vec;
            // DLOG(INFO) << "In CalculateCurvature: " << topic_name << " is already in curvature map, clear vector and put new curvature into vector.";
        }
        else
        {
            curvature_map_.insert({topic_name, curvature_vec});
            // DLOG(INFO) << "In CalculateCurvature: " << topic_name << " is not in the curvature map, insert into the map.";
        }
        return 1;
    }

    int PathEvaluator::CalculateSmoothness(const std::vector<Eigen::Vector3d> &path, const std::string &topic_name)
    {
        if (path.size() < 3)
        {
            DLOG(WARNING) << "In CalculateSmoothness: path does not have enough points!!!";
            return 0;
        }
        // smoothness = (deltax(i+1)-delta(xi))^2
        // deltax(i+1)= x(i+1)-x(i), the same for deltaxi
        std::vector<float> smoothness_vec;
        float smoothness;
        for (uint i = 0; i < path.size() - 2; ++i)
        {
            //get three points from path
            Eigen::Vector2d xp(path[i].x(), path[i].y());
            Eigen::Vector2d xi(path[i + 1].x(), path[i + 1].y());
            Eigen::Vector2d xs(path[i + 2].x(), path[i + 2].y());
            if (xp == xi || xi == xs)
            {
                DLOG(WARNING) << "In CalculateSmoothness: some points are equal, skip these points for curvature calculation!!";
                continue;
            }
            //get two vector between these three nodes
            Eigen::Vector2d pre_vector = xi - xp;
            Eigen::Vector2d succ_vector = xs - xi;

            smoothness = std::pow((succ_vector - pre_vector).norm(), 2);
            smoothness_vec.emplace_back(smoothness);
        }
        if (smoothness_map_.count(topic_name) > 0)
        {
            smoothness_map_.at(topic_name).clear();
            smoothness_map_.at(topic_name) = smoothness_vec;
            // DLOG(INFO) << "In CalculateSmoothness:" << topic_name << " is already in smoothness map, clear vector and put new curvature into vector.";
        }
        else
        {
            smoothness_map_.insert({topic_name, smoothness_vec});
            // DLOG(INFO) << "In CalculateSmoothness:" << topic_name << " is not in the smoothness map, insert into the map.";
        }
        return 1;
    }
    int PathEvaluator::CalculateClearance(const std::vector<Eigen::Vector3d> &path, const std::string &topic_name)
    {
        if (path.size() < 1)
        {
            DLOG(WARNING) << "In CalculateClearance: path does not have enough points!!!";
            return 0;
        }
        // for clearance, node2d is enough, here clearance is the distance for current point to nearest obstacle.
        // TODO: maybe in future, we can change that to distance from vehicle to nearest obstacle.
        std::vector<float> clearance_vec;
        float clearance = INFINITY;
        int map_width = map_->info.width;
        int map_height = map_->info.height;
        for (const auto &vector_3d : path)
        {
            //naive algorithm, time complexity is n^2.
            for (int index = 0; index < map_height * map_width; ++index)
            {
                if (map_->data[index])
                {
                    Eigen::Vector2d obstacle_2d = Utility::ConvertIndexToEigenVector2d(index, map_width);
                    float distance = Utility::GetDistanceFromVector2dToVector3d(vector_3d, obstacle_2d);
                    if (distance < clearance)
                    {
                        clearance = distance;
                    }
                    // DLOG(INFO) << "In CalculateClearance: current index: " << index << " converted x: " << obstacle_2d(0,0) << " converted y: " << obstacle_2d.y() << " current path location x is: " << vector_3d(0,0) << " y:" << vector_3d.y() << " distance is: " << distance << " clearance is: " << clearance;
                }
            }
            // //find its nearest obstacle
            //Eigen::Vector2d node_2d = HybridAStar::Helper::ConvertNode3DToNode2D(node_3d);
            // node_2d.setIdx(map_width);
            // //find its neighbor in a defined range;

            clearance_vec.emplace_back(clearance);
            clearance = INFINITY;
        }

        if (clearance_map_.count(topic_name) > 0)
        {
            clearance_map_.at(topic_name).clear();
            clearance_map_.at(topic_name) = clearance_vec;
            // DLOG(INFO) << "In CalculateClearance:" << topic_name << " is already in clearance map, clear vector and put new curvature into vector.";
        }
        else
        {
            clearance_map_.insert({topic_name, clearance_vec});
            // DLOG(INFO) << "In CalculateClearance:" << topic_name << " is not in the clearance map, insert into the map.";
        }
        return 1;
    }
    void PathEvaluator::CallbackSetMap(const nav_msgs::OccupancyGrid::ConstPtr &map)
    {
        map_ = map;
    }

    void PathEvaluator::CallbackPath(const nav_msgs::Path::ConstPtr &path, const std::string &topic_name)
    {
        std::vector<Eigen::Vector3d> vector_3d_vec;
        Utility::ConvertRosPathToVectorVector3D(path, vector_3d_vec);
        //reverse path since path is from goal to start.
        std::reverse(vector_3d_vec.begin(), vector_3d_vec.end());
        CalculateCurvature(vector_3d_vec, topic_name);
        CalculateSmoothness(vector_3d_vec, topic_name);
        CalculateClearance(vector_3d_vec, topic_name);
    }

    void PathEvaluator::Plot()
    {
        matplotlibcpp::ion();
        matplotlibcpp::clf();
        matplotlibcpp::subplot(2, 2, 1);
        for (const auto &curvature_vec : curvature_map_)
        {
            if (curvature_vec.first == "/path")
            {
                matplotlibcpp::plot(curvature_vec.second, {{"label", "raw path"}});
            }
            else
            {
                matplotlibcpp::plot(curvature_vec.second, {{"label", "smoothed path"}});
            }

            matplotlibcpp::legend({{"loc", "upper right"}});
            // DLOG(INFO) << "Plot curvature for topic: " << curvature_vec.first;
        }
        matplotlibcpp::title("curvature");
        matplotlibcpp::ylabel("curvature");
        matplotlibcpp::ylim(0, 1);
        matplotlibcpp::grid(true);

        matplotlibcpp::subplot(2, 2, 2);
        for (const auto &smoothness_vec : smoothness_map_)
        {
            if (smoothness_vec.first == "/path")
            {
                matplotlibcpp::plot(smoothness_vec.second, {{"label", "raw path"}});
            }
            else
            {
                matplotlibcpp::plot(smoothness_vec.second, {{"label", "smoothed path"}});
            }
            // DLOG(INFO) << "Plot smoothness for topic: " << smoothness_vec.first;
            matplotlibcpp::legend({{"loc", "upper right"}});
        }
        matplotlibcpp::title("smoothness");
        matplotlibcpp::ylabel("smoothness");
        matplotlibcpp::ylim(0, 1);
        matplotlibcpp::grid(true);

        matplotlibcpp::subplot(2, 2, 3);
        for (const auto &clearance_vec : clearance_map_)
        {
            if (clearance_vec.first == "/path")
            {
                matplotlibcpp::plot(clearance_vec.second, {{"label", "raw path"}});
            }
            else
            {
                matplotlibcpp::plot(clearance_vec.second, {{"label", "smoothed path"}});
            }
            matplotlibcpp::legend({{"loc", "upper right"}});
            // DLOG(INFO) << "Plot clearance for topic: " << clearance_vec.first;
        }
        matplotlibcpp::title("clearance");
        matplotlibcpp::ylabel("clearance");
        matplotlibcpp::ylim(0, 10);
        matplotlibcpp::grid(true);

        matplotlibcpp::pause(0.1);
    }
}
