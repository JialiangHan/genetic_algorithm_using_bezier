
#include "parameter_manager.h"

#include "glog/logging.h"
#include "gflags/gflags.h"

namespace GeneticAlgorithm
{
  void ParameterManager::LoadParams()
  {

    LoadPathPublisherParams();
    LoadPlannerParams();
    LoadCollisionDetectionParams();
    LoadGeneticAlgorithmParams();
  }

  template <typename T>
  void ParameterManager::GetSingleParam(const std::string &param_name, T &param_data)
  {
    nh_.getParam(param_name, param_data);
    DLOG(INFO) << "Load param: " << param_name << " value is :" << param_data;
  }

  std::shared_ptr<ParameterContainer> ParameterManager::GetAllParams()
  {
    return param_container_ptr_;
  }

  ParameterPlanner ParameterManager::GetPlannerParams()
  {
    return param_container_ptr_->planner_params;
  }

  ParameterPathPublisher ParameterManager::GetPathPublisherParams()
  {
    return param_container_ptr_->path_publisher_params;
  }
  ParameterCollisionDetection ParameterManager::GetCollisionDetectionParams()
  {
    return param_container_ptr_->collision_detection_params;
  }

  ParameterGeneticAlgorithm ParameterManager::GetGeneticAlgorithmParams()
  {
    return param_container_ptr_->genetic_algorithm_params;
  }
  void ParameterManager::LoadGeneticAlgorithmParams()
  {
    std::string ros_param_name;
    std::string node_prefix = "/genetic_algorithm_using_bezier/";

    ros_param_name = "population_size";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->genetic_algorithm_params.population_size);

    ros_param_name = "penalty_fitness";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->genetic_algorithm_params.penalty_fitness);

    ros_param_name = "local_search_range_radius";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->genetic_algorithm_params.local_search_range_radius);

    ros_param_name = "local_search_range_angle";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->genetic_algorithm_params.local_search_range_angle);

    ros_param_name = "h_s";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->genetic_algorithm_params.h_s);

    ros_param_name = "h_t";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->genetic_algorithm_params.h_t);
    ros_param_name = "number_of_points";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->genetic_algorithm_params.number_of_points);
    ros_param_name = "number_of_gene_in_chromosome";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->genetic_algorithm_params.number_of_gene_in_chromosome);
  }
  void ParameterManager::LoadCollisionDetectionParams()
  {
    std::string ros_param_name;
    std::string node_prefix = "/genetic_algorithm_using_bezier/";
  }
  void ParameterManager::LoadPlannerParams()
  {
    std::string ros_param_name;
    std::string node_prefix = "/genetic_algorithm_using_bezier/";

       ros_param_name = "cell_size";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->planner_params.cell_size);
    ros_param_name = "manual";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->planner_params.manual);
  }

  void ParameterManager::LoadPathPublisherParams()
  {
    std::string ros_param_name;
    std::string node_prefix = "/genetic_algorithm_using_bezier/";

    ros_param_name = "cell_size";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->path_publisher_params.cell_size);

    ros_param_name = "bloating";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->path_publisher_params.bloating);

    ros_param_name = "vehicle_width";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->path_publisher_params.vehicle_width);

    ros_param_name = "vehicle_length";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->path_publisher_params.vehicle_length);
  }
}
