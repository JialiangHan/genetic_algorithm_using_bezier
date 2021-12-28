#include "utility.h"

namespace Utility
{
    std::vector<Eigen::Vector3d> ConvertCubicBezierToVector3d(CubicBezier::CubicBezier &cubic_bezier)
    {
        std::vector<Eigen::Vector3d> out;
        int i = 0;
        for (i = 0; i < 100; ++i)
        {
            Eigen::Vector3d point3d;
            Eigen::Vector2d point;
            // DLOG(INFO) << " i/100 = " << i / 100.0;
            point = cubic_bezier.GetValueAt(i / 100.0);
            point3d = ConvertVector2dToVector3d(point);
            point3d.z() = cubic_bezier.GetAngleAt(i / 100.0);
            out.emplace_back(point3d);
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

    float ConvertRadToDeg(const float &rad)
    {
        float deg;
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

    float GetDistanceFromVector2dToVector3d(const Eigen::Vector3d &vector_3d, const Eigen::Vector2d &vector_2d)
    {
        float distance;
        float delta_x = vector_2d.x() - vector_3d.x();
        float delta_y = vector_2d.y() - vector_3d.y();
        distance = sqrt(delta_x * delta_x + delta_y * delta_y);
        return distance;
    }

    float DegNormalization(const float &t)
    {
        float out;
        out = fmod(t, 360);
        if (out < 0)
        {
            out += 360;
        }
        return out;
    }

    float RadNormalization(const float &rad)
    {
        float out;
        out = fmod(rad, 2.f * M_PI);
        if (out < 0)
        {
            out += 2.f * M_PI;
        }
        return out;
    }
    bool OnSegment(Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d p3)
    {
        if (p2.x() <= std::max(p1.x(), p3.x()) && p2.x() >= std::min(p1.x(), p3.x()) && p2.y() <= std::max(p1.y(), p3.y()) && p2.y() >= std::min(p1.y(), p3.y()))
        {
            Eigen::Vector2d p1p2 = p2 - p1;
            Eigen::Vector2d p1p3 = p3 - p1;
            if (p1p2.dot(p1p3) / p1p2.norm() / p1p3.norm() == 1)
            {
                return true;
            }
            return false;
        }
        return false;
    }

    float CrossProduct(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2)
    {
        return p1.x() * p2.y() - p2.x() * p1.y();
    }

    int IsIntersect(Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d p3, Eigen::Vector2d p4)
    {
        if (std::max(p3.x(), p4.x()) < std::min(p1.x(), p2.x()) ||
            std::max(p3.y(), p4.y()) < std::min(p1.y(), p2.y()) ||
            std::max(p1.x(), p2.x()) < std::min(p3.x(), p4.x()) ||
            std::max(p1.y(), p2.y()) < std::min(p3.y(), p4.y()))
        {
            return 0;
        }
        else
        {
            if (CrossProduct(p1 - p3, p4 - p3) * CrossProduct(p2 - p3, p4 - p3) <= 0 &&
                CrossProduct(p3 - p2, p1 - p2) * CrossProduct(p4 - p2, p1 - p2) <= 0)
            {
                return 1;
            }
            return 0;
        }
    }
    int IsInsidePolygon(const std::vector<Eigen::Vector2d> &polygon, const Eigen::Vector2d &point)
    {
        // Eigen::Vector2d p1, p2;
        // int out;
        // for (uint i = 0; i < polygon.size() - 1; ++i)
        // {
        //     p1 = polygon[i] - point;
        //     p2 = polygon[i + 1] - point;
        //     // p1.cross(p2);
        // }
        return 1;
    }
}
