#include "utility.h"

namespace Utility
{
    nav_msgs::Path ConvertVectorVector3DToRosPath(const std::vector<Eigen::Vector3d> &vector_3d_vec)
    {
        nav_msgs::Path out;
        out.header.stamp = ros::Time::now();
        for (const auto &vector3d : vector_3d_vec)
        {
            geometry_msgs::PoseStamped vertex;
            vertex.pose.position.x = vector3d.x();
            vertex.pose.position.y = vector3d.y();
            vertex.pose.position.z = vector3d.z();
            vertex.pose.orientation.x = 0;
            vertex.pose.orientation.y = 0;
            vertex.pose.orientation.z = 0;
            vertex.pose.orientation.w = 0;
            out.poses.push_back(vertex);
        }
        return out;
    }
    Eigen::Vector3d ConvertVector2dToVector3d(const Eigen::Vector2d &vector_2d)
    {
        Eigen::Vector3d out;
        out.x() = vector_2d.x();
        out.y() = vector_2d.y();
        out.z() = 0;
        return out;
    }
    double ConvertDegToRad(const double &deg)
    {
        double rad;
        rad = deg / 360 * 2 * M_PI;
        return rad;
    }
    double ConvertRadToDeg(const double &rad)
    {
        double deg;
        deg = rad / 2 / M_PI * 360;
        return deg;
    }
    Eigen::Vector2d ConvertVector3dToVector2d(const Eigen::Vector3d &vector_3d)
    {
        Eigen::Vector2d out;
        out.x() = vector_3d.x();
        out.y() = vector_3d.y();
        return out;
    }

    void ConvertRosPathToVectorVector3D(const nav_msgs::Path::ConstPtr &path, std::vector<Eigen::Vector3d> &vector_3d_vec)
    {
        vector_3d_vec.clear();
        for (uint i = 0; i < path->poses.size(); ++i)
        {
            Eigen::Vector3d point;
            point.x() = (path->poses[i].pose.position.x);
            point.y() = (path->poses[i].pose.position.y);
            //not sure this is correct;
            point.z() = (path->poses[i].pose.position.z);
            vector_3d_vec.emplace_back(point);
        }
    }

    Eigen::Vector2d ConvertIndexToEigenVector2d(const int &index, const int &map_width)
    {
        Eigen::Vector2d out;
        out.x() = (index % map_width);
        out.y() = (index / map_width);
        return out;
    }

    double GetDistanceFromVector2dToVector3d(const Eigen::Vector3d &vector_3d, const Eigen::Vector2d &vector_2d)
    {
        double distance;
        double delta_x = vector_2d.x() - vector_3d.x();
        double delta_y = vector_2d.y() - vector_3d.y();
        distance = sqrt(delta_x * delta_x + delta_y * delta_y);
        return distance;
    }

    double DegNormalization(const double &t)
    {
        double out;
        out = fmod(t, 360);
        if (out < 0)
        {
            out += 360;
        }
        if (out > 180)
        {
            out = out - 360;
        }
        return out;
    }

    double RadNormalization(const double &rad)
    {
        double out;
        out = fmod(rad, 2.f * M_PI);
        if (out < 0)
        {
            out += 2.f * M_PI;
        }
        if (out > M_PI)
        {
            out = out - 2.0f * M_PI;
        }
        return out;
    }
    bool OnSegment(const Eigen::Vector2d &p1, const Eigen::Vector2d &p3, const Eigen::Vector2d &p2)
    {
        if (p2.x() <= std::max(p1.x(), p3.x()) && p2.x() >= std::min(p1.x(), p3.x()) && p2.y() <= std::max(p1.y(), p3.y()) && p2.y() >= std::min(p1.y(), p3.y()))
        {
            Eigen::Vector2d p1p2 = p2 - p1;
            Eigen::Vector2d p1p3 = p3 - p1;
            if (abs(p1p2.dot(p1p3) / p1p2.norm() / p1p3.norm() - 1) < 1e-6)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    double CrossProduct(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2)
    {
        double out;
        out = p1.x() * p2.y() - p2.x() * p1.y();
        // DLOG(INFO) << "result is " << out;
        return out;
    }

    int IsIntersect(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &p3, const Eigen::Vector2d &p4)
    {
        if (std::max(p3.x(), p4.x()) < std::min(p1.x(), p2.x()) ||
            std::max(p3.y(), p4.y()) < std::min(p1.y(), p2.y()) ||
            std::max(p1.x(), p2.x()) < std::min(p3.x(), p4.x()) ||
            std::max(p1.y(), p2.y()) < std::min(p3.y(), p4.y()))
        {
            DLOG(INFO) << "these two segments are far away!";
            return 0;
        }
        else
        {
            if (CrossProduct(p1 - p3, p4 - p3) * CrossProduct(p2 - p3, p4 - p3) <= 0 &&
                CrossProduct(p3 - p2, p1 - p2) * CrossProduct(p4 - p2, p1 - p2) <= 0)
            {
                DLOG(INFO) << "Intersection!!";
                return 1;
            }
            return 0;
        }
    }

    Eigen::Vector2d FindIntersectionPoint(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &p3, const Eigen::Vector2d &p4)
    {
        Eigen::Vector2d out;
        if (0 == IsIntersect(p1, p2, p3, p4))
        {
            out.x() = 10000;
            out.y() = 10000;
        }
        else
        {
            Eigen::Vector2d dir_1 = p2 - p1, dir_2 = p4 - p3;
            double t;
            t = CrossProduct(p3 - p1, dir_2) / CrossProduct(dir_1, dir_2);
            out = p1 + t * dir_1;
        }
        return out;
    }
    int IsAboveSegment(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &p3)
    {
        int above = 1, on_segment = 1, below = 0;
        if (OnSegment(p1, p2, p3))
        {
            return on_segment;
        }
        else
        {
            if ((p2 - p1).x() == 0)
            {
                if (p3.x() > p1.x())
                {
                    return below;
                }
                else
                {
                    return above;
                }
            }
            else if ((p2 - p1).y() == 0)
            {
                if (p3.y() > p1.y())
                {
                    return above;
                }
                else
                {
                    return below;
                }
            }
            else
            {
                double k = (p2 - p1).y() / (p2 - p1).x();
                double b = p1.y() - k * p1.x();
                if (k * p3.x() + b - p3.y() < 0)
                {
                    return above;
                }
                else
                {
                    return below;
                }
            }
        }
    }
    int IsInsidePolygon(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &point)
    {
        Eigen::Vector2d far_away_point(10000, point.y());
        int number_intersection = 0;
        for (uint i = 0; i < polygon.size() - 1; ++i)
        {
            //check if point is on polygon edge
            if (OnSegment(polygon[i], polygon[i + 1], point))
            {
                DLOG(INFO) << "Point is on polygon edge.";
                return 1;
            }
            //if edge intersect with vector from point to far away point
            if (IsIntersect(point, far_away_point, polygon[i], polygon[i + 1]) == 1)
            {
                Eigen::Vector2d intersection_point = FindIntersectionPoint(point, far_away_point, polygon[i], polygon[i + 1]);
                DLOG(INFO) << "point horizontal vector is intersecting polygon!";
                if ((polygon[i] - intersection_point).norm() < 1e-3)
                {
                    if (IsAboveSegment(point, far_away_point, polygon[i + 1]) == 0)
                    {
                        number_intersection += 1;
                        DLOG(INFO) << "true intersection!! +1";
                    }
                }
                else if ((polygon[i + 1] - intersection_point).norm() < 1e-3)
                {
                    if (IsAboveSegment(point, far_away_point, polygon[i]) == 0)
                    {
                        number_intersection += 1;
                        DLOG(INFO) << "true intersection!! +1";
                    }
                }
                else
                {
                    number_intersection += 1;
                    DLOG(INFO) << "true intersection!! +1";
                }
            }
        }
        DLOG(INFO) << "number of intersection is " << number_intersection;
        if ((number_intersection % 2) == 0)
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }
    int IsInsidePolygon(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector3d &point)
    {
        Eigen::Vector2d point_2d = Utility::ConvertVector3dToVector2d(point);
        return IsInsidePolygon(polygon, point_2d);
    }
    std::vector<Eigen::Vector2d> CreatePolygon(const double &width, const double &height)
    {
        std::vector<Eigen::Vector2d> polygon;
        polygon.emplace_back(Eigen::Vector2d(0, 0));
        polygon.emplace_back(Eigen::Vector2d(width, 0));
        polygon.emplace_back(Eigen::Vector2d(width, height));
        polygon.emplace_back(Eigen::Vector2d(0, height));
        polygon.emplace_back(Eigen::Vector2d(0, 0));
        return polygon;
    }
}
