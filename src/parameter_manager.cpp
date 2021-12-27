
#include "parameter_manager.h"

#include "glog/logging.h"
#include "gflags/gflags.h"

namespace GeneticAlgorithm
{
  void ParameterManager::LoadParams()
  {
    // LoadSmootherParams();
    // LoadAlgorithmParams();
    LoadPathPublisherParams();
    LoadPlannerParams();
    // LoadPathParams();
    // LoadVisualizeParams();
    LoadCollisionDetectionParams();
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

  // ParameterAlgorithm ParameterManager::GetAlgorithmParams()
  // {
  //   return param_container_ptr_->algorithm_params;
  // }

  // ParameterSmoother ParameterManager::GetSmootherParams()
  // {
  //   return param_container_ptr_->smoother_params;
  // }

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
  // ParameterVisualize ParameterManager::GetVisualizeParams()
  // {
  //   return param_container_ptr_->visualize_params;
  // }
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
  void ParameterManager::LoadCollisionDetectionParams()
  {
    std::string ros_param_name;
    std::string node_prefix = "/hybrid_astar/";

    ros_param_name = "position_resolution";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->collision_detection_params.position_resolution);
    ros_param_name = "headings";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->collision_detection_params.headings);
  }
  void ParameterManager::LoadPlannerParams()
  {
    std::string ros_param_name;
    std::string node_prefix = "/hybrid_astar/";

    ros_param_name = "headings";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->planner_params.headings);

    ros_param_name = "cell_size";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->planner_params.cell_size);

    ros_param_name = "manual";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->planner_params.manual);

    ros_param_name = "dubins_Lookup";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->planner_params.dubins_lookup);

    ros_param_name = "control_point_1_x";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->planner_params.control_point_1_x);

    ros_param_name = "control_point_1_y";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->planner_params.control_point_1_y);

    ros_param_name = "control_point_2_x";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->planner_params.control_point_2_x);

    ros_param_name = "control_point_2_y";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->planner_params.control_point_2_y);
  }
  // void ParameterManager::LoadAlgorithmParams()
  // {
  //   std::string ros_param_name;
  //   std::string node_prefix = "/hybrid_astar/";

  //   ros_param_name = "penalty_turning";
  //   GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->algorithm_params.penalty_turning);
  //   ros_param_name = "penalty_reverse";
  //   GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->algorithm_params.penalty_reverse);
  //   ros_param_name = "penalty_change_of_direction";
  //   GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->algorithm_params.penalty_change_of_direction);

  //   ros_param_name = "reverse";
  //   GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->algorithm_params.reverse);
  //   ros_param_name = "visualization";
  //   GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->algorithm_params.visualization);
  //   ros_param_name = "max_iterations";
  //   GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->algorithm_params.max_iterations);
  //   ros_param_name = "dubins_shot_distance";
  //   GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->algorithm_params.dubins_shot_distance);
  //   ros_param_name = "tie_breaker";
  //   GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->algorithm_params.tie_breaker);
  //   ros_param_name = "visualization2D";
  //   GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->algorithm_params.visualization2D);
  //   ros_param_name = "epsilon";
  //   GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->algorithm_params.epsilon);
  //   ros_param_name = "dubins";
  //   GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->algorithm_params.dubins);
  //   ros_param_name = "dubins_step_size";
  //   GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->algorithm_params.dubins_step_size);
  //   ros_param_name = "min_turning_radius";
  //   GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->algorithm_params.min_turning_radius);
  //   ros_param_name = "two_D";
  //   GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->algorithm_params.two_D);
  //   ros_param_name = "headings";
  //   GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->algorithm_params.headings);
  //   ros_param_name = "dubins_width";
  //   GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->algorithm_params.dubins_width);
  // }
  void ParameterManager::LoadPathPublisherParams()
  {
    std::string ros_param_name;
    std::string node_prefix = "/hybrid_astar/";

    ros_param_name = "cell_size";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->path_publisher_params.cell_size);

    ros_param_name = "bloating";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->path_publisher_params.bloating);

    ros_param_name = "vehicle_width";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->path_publisher_params.vehicle_width);

    ros_param_name = "vehicle_length";
    GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->path_publisher_params.vehicle_length);
  }

  // void ParameterManager::LoadVisualizeParams()
  // {
  //   std::string ros_param_name;
  //   std::string node_prefix = "/hybrid_astar/";

  //   ros_param_name = "cell_size";
  //   GetSingleParam(node_prefix + ros_param_name, param_container_ptr_->visualize_params.cell_size);
  // }
}
