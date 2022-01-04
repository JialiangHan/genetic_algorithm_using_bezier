#include "path_publisher.h"

using namespace GeneticAlgorithm;

//###################################################
//                                         CLEAR PATH
//###################################################

void PathPublisher::Clear()
{
  Eigen::Vector3d node;
  path_.poses.clear();

  path_points_.markers.clear();

  PublishPath();

  PublishPathPoints();
}

//###################################################
//                                         TRACE PATH
//###################################################
// __________
// TRACE PATH
void PathPublisher::UpdatePath(const std::vector<Eigen::Vector3d> &nodePath)
{
  path_.header.stamp = ros::Time::now();

  for (size_t i = 0; i < nodePath.size(); ++i)
  {
    AddSegment(nodePath[i]);
  }
}

void PathPublisher::UpdatePoint(const std::vector<Eigen::Vector3d> &point_vec)
{
  path_.header.stamp = ros::Time::now();
  int i = 0;
  for (const auto &point : point_vec)
  {
    AddPoint(point, i);
    i++;
  }
}

void PathPublisher::UpdateFitness(const std::vector<double> &fitness_vec)
{

  fitness_msg_.fitness_vec = fitness_vec;
}
// ___________
// ADD SEGMENT
void PathPublisher::AddSegment(const Eigen::Vector3d &node)
{
  geometry_msgs::PoseStamped vertex;
  vertex.pose.position.x = node.x() * params_.cell_size;
  vertex.pose.position.y = node.y() * params_.cell_size;
  vertex.pose.position.z = node.z() * params_.cell_size;
  vertex.pose.orientation.x = 0;
  vertex.pose.orientation.y = 0;
  vertex.pose.orientation.z = 0;
  vertex.pose.orientation.w = 0;
  path_.poses.push_back(vertex);
}
// ________
// ADD NODE
void PathPublisher::AddPoint(const Eigen::Vector3d &node, const int &i)
{
  visualization_msgs::Marker pathNode;
  if (i == 0)
  {
    pathNode.action = 3;
  }
  else
  {
    pathNode.action = 0;
  }

  // pathNode.action = 0;

  pathNode.header.frame_id = "path";
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
  pathNode.type = visualization_msgs::Marker::SPHERE;
  double scale = 0.2;
  pathNode.scale.x = scale;
  pathNode.scale.y = scale;
  pathNode.scale.z = scale;
  pathNode.color.a = 1.0;

  pathNode.color.r = 1;
  pathNode.color.g = 0;
  pathNode.color.b = 0;

  pathNode.pose.position.x = node.x();
  pathNode.pose.position.y = node.y();
  path_points_.markers.push_back(pathNode);
}
