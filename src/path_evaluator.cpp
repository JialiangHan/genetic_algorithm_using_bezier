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

        std::vector<double> curvature_vec;
        double curvature;
        // DLOG(INFO) << "In CalculateCurvature: " << topic_name << " path size is :" << path.size();
        // for (const auto &point : path)
        // {
        //     DLOG(INFO) << "point is " << point.x() << " " << point.y() << " " << point.z();
        // }
        // use three points to calculate curvature;
        for (uint i = 0; i < path.size() - 2; ++i)
        {
            //get three points from path
            Eigen::Vector3d xp = path[i];
            Eigen::Vector3d xi = path[i + 1];
            Eigen::Vector3d xs = path[i + 2];

            //get two vector between these three nodes
            Eigen::Vector2d pre_vector = Utility::ConvertVector3dToVector2d(xi - xp);
            Eigen::Vector2d succ_vector = Utility::ConvertVector3dToVector2d(xs - xi);

            //calculate delta distance and delta angle
            double delta_distance = succ_vector.norm();
            double pre_vector_length = pre_vector.norm();
            if (delta_distance < 1e-3 || pre_vector_length < 1e-3)
            {
                DLOG(WARNING) << "In CalculateCurvature: some points are too close, skip these points for curvature calculation!!";
                continue;
            }
            // there would some calculation error here causing number inside acos greater than 1 or smaller than -1.
            double temp = pre_vector.dot(succ_vector) / (delta_distance * pre_vector_length);
            if (temp > 1 || temp < -1)
            {
                temp = round(temp);
            }
            double delta_angle = std::acos(temp);

            curvature = delta_angle / delta_distance;
            curvature_vec.emplace_back(curvature);

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

            // LOG(INFO) << "xp x is :" << xp.x() << " y is: " << xp.y();
            // LOG(INFO) << "xi x is :" << xi.x() << " y is: " << xi.y();
            // LOG(INFO) << "xs x is :" << xs.x() << " y is: " << xs.y();
            // LOG(INFO) << "pre_vector x is :" << pre_vector(0, 0) << "y is: " << pre_vector.y();
            // LOG(INFO) << "succ_vector x is :" << succ_vector(0, 0) << "y is: " << succ_vector.y();
            // LOG(INFO) << "delta_distance is:" << delta_distance;
            // LOG(INFO) << "pre_vector_length is: " << pre_vector_length;
            // LOG(INFO) << "delta_angle is: " << delta_angle;
            // LOG(INFO) << "In CalculateCurvature:" << i << "th curvature is:" << curvature;
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
            // DLOG(WARNING) << "In CalculateSmoothness: path does not have enough points!!!";
            return 0;
        }
        // smoothness = (deltax(i+1)-delta(xi))^2
        // deltax(i+1)= x(i+1)-x(i), the same for deltaxi
        std::vector<double> smoothness_vec;
        double smoothness;
        for (uint i = 0; i < path.size() - 2; ++i)
        {
            //get three points from path
            Eigen::Vector2d xp(path[i].x(), path[i].y());
            Eigen::Vector2d xi(path[i + 1].x(), path[i + 1].y());
            Eigen::Vector2d xs(path[i + 2].x(), path[i + 2].y());

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
            // DLOG(WARNING) << "In CalculateClearance: path does not have enough points!!!";
            return 0;
        }
        // for clearance, node2d is enough, here clearance is the distance for current point to nearest obstacle.
        // TODO: maybe in future, we can change that to distance from vehicle to nearest obstacle.
        std::vector<double> clearance_vec;
        double clearance = INFINITY;
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
                    double distance = Utility::GetDistanceFromVector2dToVector3d(vector_3d, obstacle_2d);
                    if (distance < clearance)
                    {
                        clearance = distance;
                    }
                    // DLOG(INFO) << "In CalculateClearance: current index: " << index << " converted x: " << obstacle_2d(0, 0) << " converted y: " << obstacle_2d.y() << " current path location x is: " << vector_3d(0, 0) << " y:" << vector_3d.y() << " distance is: " << distance << " clearance is: " << clearance;
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

    int PathEvaluator::CalculateSteeringAngle(const std::vector<Eigen::Vector3d> &path, const std::string &topic_name)
    {
        if (path.size() < 1)
        {
            // DLOG(WARNING) << "In CalculateSteeringAngle: path does not have enough points!!!";
            return 0;
        }
        // for clearance, node2d is enough, here clearance is the distance for current point to nearest obstacle.
        std::vector<double> steering_angle_vec;
        for (uint i = 0; i < path.size() - 1; ++i)
        {
            double steering_angle = path[i + 1].z() - path[i].z();
            steering_angle = Utility::DegNormalization(Utility::ConvertRadToDeg(steering_angle));

            steering_angle_vec.emplace_back(steering_angle);
        }
        if (steering_angle_map_.count(topic_name) > 0)
        {
            steering_angle_map_.at(topic_name).clear();
            steering_angle_map_.at(topic_name) = steering_angle_vec;
            // DLOG(INFO) << "In CalculateClearance:" << topic_name << " is already in clearance map, clear vector and put new curvature into vector.";
        }
        else
        {
            steering_angle_map_.insert({topic_name, steering_angle_vec});
            // DLOG(INFO) << "In CalculateClearance:" << topic_name << " is not in the clearance map, insert into the map.";
        }
        return 1;
    }
    void PathEvaluator::CallbackSetMap(const nav_msgs::OccupancyGrid::ConstPtr &map)
    {
        map_ = map;
    }

    void PathEvaluator::CallbackFitness(const genetic_algorithm_using_bezier::FitnessMsgVecConstPtr &fitness_vec)
    {
        std::string topic_name = "fitness";
        if (fitness_map_.count(topic_name) > 0)
        {
            if (fitness_vec->fitness_vec.size() > 0)
            {
                fitness_map_.at(topic_name).resize(fitness_vec->fitness_vec.size());
                fitness_map_.at(topic_name) = fitness_vec->fitness_vec;
                // LOG(INFO) << "fitness map size is " << fitness_map_.at(topic_name).size();
            }
            // DLOG(INFO) << "In CalculateClearance:" << topic_name << " is already in clearance map, clear vector and put new curvature into vector.";
        }
        else
        {
            fitness_map_.insert({topic_name, fitness_vec->fitness_vec});
            // DLOG(INFO) << "In CalculateClearance:" << topic_name << " is not in the clearance map, insert into the map.";
        }
    }

    void PathEvaluator::CallbackPath(const nav_msgs::Path::ConstPtr &path, const std::string &topic_name)
    {
        std::vector<Eigen::Vector3d> vector_3d_vec;
        Utility::ConvertRosPathToVectorVector3D(path, vector_3d_vec);
        // DLOG(INFO) << "path size is " << vector_3d_vec.size();
        //reverse path since path is from goal to start.
        // std::reverse(vector_3d_vec.begin(), vector_3d_vec.end());
        CalculateCurvature(vector_3d_vec, topic_name);
        CalculateSmoothness(vector_3d_vec, topic_name);
        CalculateClearance(vector_3d_vec, topic_name);
        CalculateSteeringAngle(vector_3d_vec, topic_name);
    }

    int PathEvaluator::CalculateMetricMap()
    {
        metric_map_.clear();
        std::string metric_name = "clearance";
        if (metric_map_.count(metric_name) > 0)
        {
            metric_map_.at(metric_name).clear();
            metric_map_.at(metric_name) = clearance_map_;
            // DLOG(INFO) << "In CalculateClearance:" << metric_name << " is already in clearance map, clear vector and put new curvature into vector.";
        }
        else
        {
            metric_map_.insert({metric_name, clearance_map_});
            // DLOG(INFO) << "In CalculateClearance:" << metric_name << " is not in the clearance map, insert into the map.";
        }
        metric_name = "curvature";
        if (metric_map_.count(metric_name) > 0)
        {
            metric_map_.at(metric_name).clear();
            metric_map_.at(metric_name) = curvature_map_;
            // DLOG(INFO) << "In CalculateClearance:" << metric_name << " is already in clearance map, clear vector and put new curvature into vector.";
        }
        else
        {
            metric_map_.insert({metric_name, curvature_map_});
            // DLOG(INFO) << "In CalculateClearance:" << metric_name << " is not in the clearance map, insert into the map.";
        }
        metric_name = "smoothness";
        if (metric_map_.count(metric_name) > 0)
        {
            metric_map_.at(metric_name).clear();
            metric_map_.at(metric_name) = smoothness_map_;
            // DLOG(INFO) << "In CalculateClearance:" << metric_name << " is already in clearance map, clear vector and put new curvature into vector.";
        }
        else
        {
            metric_map_.insert({metric_name, smoothness_map_});
            // DLOG(INFO) << "In CalculateClearance:" << metric_name << " is not in the clearance map, insert into the map.";
        }
        metric_name = "steering_angle";
        if (metric_map_.count(metric_name) > 0)
        {
            metric_map_.at(metric_name).clear();
            metric_map_.at(metric_name) = steering_angle_map_;
            // DLOG(INFO) << "In CalculateClearance:" << metric_name << " is already in clearance map, clear vector and put new curvature into vector.";
        }
        else
        {
            metric_map_.insert({metric_name, steering_angle_map_});
            // DLOG(INFO) << "In CalculateClearance:" << metric_name << " is not in the clearance map, insert into the map.";
        }

        metric_name = "fitness";
        if (metric_map_.count(metric_name) > 0)
        {
            // fitness_map_.at(topic_name).resize(fitness_vec->fitness_vec.size());
            // fitness_map_.at(topic_name) = fitness_vec->fitness_vec;
            metric_map_.at(metric_name).clear();
            metric_map_.at(metric_name) = fitness_map_;
            // for (const auto &item : metric_map_.at(metric_name))
            // {
            //     LOG(INFO) << "item size " << item.second.size();
            // }
            // DLOG(INFO) << "In CalculateClearance:" << metric_name << " is already in clearance map, clear vector and put new curvature into vector.";
        }
        else
        {
            metric_map_.insert({metric_name, fitness_map_});
            // for (const auto &item : metric_map_.at(metric_name))
            // {
            //     LOG(INFO) << "item size " << item.second.size();
            // }
            // DLOG(INFO) << "In CalculateClearance:" << metric_name << " is not in the clearance map, insert into the map.";
        }

        return 1;
    }

    void PathEvaluator::Plot()
    {
        CalculateMetricMap();
        matplotlibcpp::ion();

        int metric_size = metric_map_.size();
        int index = 1, row = 2, column = 2;
        while (1)
        {
            if (row * column >= metric_size)
            {
                for (const auto &map : metric_map_)
                {
                    matplotlibcpp::subplot(row, column, index);
                    for (const auto &pair : map.second)
                    {
                        if (pair.first == "/path")
                        {
                            if (pair.second.size() != 0)
                            {
                                matplotlibcpp::cla();
                                matplotlibcpp::plot(pair.second, {{"label", "raw path"}});
                            }
                        }
                        else
                        {
                            if (pair.second.size() != 0)
                            {
                                matplotlibcpp::cla();
                                matplotlibcpp::plot(pair.second, {{"label", "smoothed path"}});
                            }
                        }

                        // matplotlibcpp::legend({{"loc", "upper right"}});
                    }
                    matplotlibcpp::title(map.first);
                    matplotlibcpp::ylabel(map.first);
                    // matplotlibcpp::ylim(0, 1);
                    matplotlibcpp::grid(true);
                    index++;
                }
                matplotlibcpp::pause(0.1);
                return;
            }
            else
            {
                column++;
            }
        }
    }
}
