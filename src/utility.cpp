#include "utility.h"

namespace Utility
{

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
