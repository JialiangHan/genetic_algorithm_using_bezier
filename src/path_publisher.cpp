#include "path_publisher.h"

using namespace GeneticAlgorithm;

//###################################################
//                                         CLEAR PATH
//###################################################

void PathPublisher::Clear()
{
  Eigen::Vector3d node;
  path_.poses.clear();
  path_nodes_.markers.clear();
  path_vehicles_.markers.clear();
  AddNode(node, 0);
  AddVehicle(node, 1);
  PublishPath();
  PublishPathNodes();
  PublishPathVehicles();
}

//###################################################
//                                         TRACE PATH
//###################################################
// __________
// TRACE PATH
void PathPublisher::UpdatePath(const std::vector<Eigen::Vector3d> &nodePath)
{
  path_.header.stamp = ros::Time::now();
  int k = 0;

  for (size_t i = 0; i < nodePath.size(); ++i)
  {
    AddSegment(nodePath[i]);
    AddNode(nodePath[i], k);
    k++;
    AddVehicle(nodePath[i], k);
    k++;
  }

  return;
}
// ___________
// ADD SEGMENT
void PathPublisher::AddSegment(const Eigen::Vector3d &node)
{
  geometry_msgs::PoseStamped vertex;
  vertex.pose.position.x = node.x() * params_.cell_size;
  vertex.pose.position.y = node.y() * params_.cell_size;
  vertex.pose.position.z = 0;
  vertex.pose.orientation.x = 0;
  vertex.pose.orientation.y = 0;
  vertex.pose.orientation.z = 0;
  vertex.pose.orientation.w = 0;
  path_.poses.push_back(vertex);
}

// ________
// ADD NODE
void PathPublisher::AddNode(const Eigen::Vector3d &node, int i)
{
  visualization_msgs::Marker pathNode;

  // delete all previous markers
  if (i == 0)
  {
    pathNode.action = 3;
  }

  pathNode.header.frame_id = "path";
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
  pathNode.type = visualization_msgs::Marker::SPHERE;
  pathNode.scale.x = 0.1;
  pathNode.scale.y = 0.1;
  pathNode.scale.z = 0.1;
  pathNode.color.a = 1.0;

  // if (smoothed_)
  // {
  //   pathNode.color.r = params_.pink.red;
  //   pathNode.color.g = params_.pink.green;
  //   pathNode.color.b = params_.pink.blue;
  // }
  // else
  // {
  pathNode.color.r = params_.purple.red;
  pathNode.color.g = params_.purple.green;
  pathNode.color.b = params_.purple.blue;
  // }

  pathNode.pose.position.x = node.x() * params_.cell_size;
  pathNode.pose.position.y = node.y() * params_.cell_size;
  path_nodes_.markers.push_back(pathNode);
}

void PathPublisher::AddVehicle(const Eigen::Vector3d &node, int i)
{
  visualization_msgs::Marker pathVehicle;

  // delete all previous markersg
  if (i == 1)
  {
    pathVehicle.action = 3;
  }

  pathVehicle.header.frame_id = "path";
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.id = i;
  pathVehicle.type = visualization_msgs::Marker::CUBE;
  pathVehicle.scale.x = params_.vehicle_length - params_.bloating * 2;
  pathVehicle.scale.y = params_.vehicle_width - params_.bloating * 2;
  pathVehicle.scale.z = 1;
  pathVehicle.color.a = 0.1;

  // if (smoothed_)
  // {
  //   pathVehicle.color.r = params_.orange.red;
  //   pathVehicle.color.g = params_.orange.green;
  //   pathVehicle.color.b = params_.orange.blue;
  // }
  // else
  // {
  pathVehicle.color.r = params_.teal.red;
  pathVehicle.color.g = params_.teal.green;
  pathVehicle.color.b = params_.teal.blue;
  // }

  pathVehicle.pose.position.x = node.x() * params_.cell_size;
  pathVehicle.pose.position.y = node.y() * params_.cell_size;
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.z());
  path_vehicles_.markers.push_back(pathVehicle);
}
