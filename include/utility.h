/**
 * @file utility.h
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief include some type conversion function and some computational geometry
 * @version 0.1
 * @date 2021-12-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef UTILITY
#define UTILITY

#include <cmath>
#include <nav_msgs/Path.h>
#include "glog/logging.h"
#include "gflags/gflags.h"
#include <Eigen/Dense>

namespace Utility
{

    void ConvertRosPathToVectorVector3D(const nav_msgs::Path::ConstPtr &path, std::vector<Eigen::Vector3d> &vector_3d_vec);

    nav_msgs::Path ConvertVectorVector3DToRosPath(const std::vector<Eigen::Vector3d> &vector_3d_vec);

    Eigen::Vector2d ConvertIndexToEigenVector2d(const int &index, const int &map_width);

    float GetDistanceFromVector2dToVector3d(const Eigen::Vector3d &vector_3d, const Eigen::Vector2d &vector_2d);

    /**
     * @brief convert angle in deg into [-PI,Pi)
     * 
     * @param deg 
     * @return float 
     */
    float DegNormalization(const float &deg);

    float RadNormalization(const float &rad);

    Eigen::Vector2d ConvertVector3dToVector2d(const Eigen::Vector3d &vector_3d);

    Eigen::Vector3d ConvertVector2dToVector3d(const Eigen::Vector2d &vector_2d);

    float ConvertDegToRad(const float &deg);

    float ConvertRadToDeg(const float &rad);
    /**
     * @brief check if p3 lines on p1-p2
     * 
     * @param p1 
     * @param p2 
     * @param p3 
     * @return true 
     * @return false 
     */
    bool OnSegment(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &p3);
    /**
     * @brief determine p3 is above or below segment p1-p2
     * 
     * @param p1 
     * @param p2 
     * @param p3 
     * @return int 1 above and on segment, 0 below, 
     */
    int IsAboveSegment(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &p3);
    /**
     * @brief determine if two segment(p1-p2, p3-p4) are intersected? 
     * 
     * @param p1 
     * @param p2 
     * @param p3 
     * @param p4 
     * @return int 
     */
    int IsIntersect(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &p3, const Eigen::Vector2d &p4);
    /**
     * @brief find intersection point between segment p1-p2 and segment p3-p4
     * 
     * @param p1 
     * @param p2 
     * @param p3 
     * @param p4 
     * @return Eigen::Vector2d 
     */
    Eigen::Vector2d FindIntersectionPoint(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &p3, const Eigen::Vector2d &p4);
    /**
     * @brief this is only work for vector2d, for 3d please cross in eigen
     * 
     * @param p1 
     * @param p2 
     * @return float 
     */
    float CrossProduct(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2);
    /**
     * @brief determine if a point is inside a polygon
     * 
     * @param polygon p0->p1->p2->p3->p0
     * @param point 
     * @return int 1 inside, 0 outside
     */
    int IsInsidePolygon(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &point);
    int IsInsidePolygon(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector3d &point);
    /**
     * @brief Create a Polygon object, current is only for rectangle
     * 
     * @param width 
     * @param height 
     * @return std::vector<Eigen::Vector2d> 
     */
    std::vector<Eigen::Vector2d> CreatePolygon(const float &width, const float &height);
}

#endif // UTILITY
