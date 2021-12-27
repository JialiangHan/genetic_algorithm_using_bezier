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
}
