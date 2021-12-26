
#include "parameter_manager.h"

#include "glog/logging.h"
#include "gflags/gflags.h"

namespace GeneticAlgorithm
{
  void ParameterManager::LoadParams()
  {

    LoadGeneticAlgorithmParams();
  }

  template <typename T>
  void ParameterManager::GetSingleParam(const std::string &param_name, T &param_data)
  {
    nh_.getParam(param_name, param_data);
    // DLOG(INFO) << "Load param: " << param_name << " value is :" << param_data;
  }

  std::shared_ptr<ParameterContainer> ParameterManager::GetAllParams()
  {
    return param_container_ptr_;
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

    ros_param_name = "h_s";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->genetic_algorithm_params.h_s);

    ros_param_name = "h_t";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->genetic_algorithm_params.h_t);
  }
}
