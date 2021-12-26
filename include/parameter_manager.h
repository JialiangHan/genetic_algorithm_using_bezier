#pragma once

#include <ros/ros.h>
namespace GeneticAlgorithm
{
  //this struct contains some used parameters in collisiondetection class
  struct ParameterGeneticAlgorithm
  {
    /**
     * @brief population size of chromosome 
     * 
     */
    int population_size = 20;
    /**
     * @brief penalty for fitness function
     * 
     */
    float penalty_fitness = 50;
    /**
     * @brief the range for local mutation, unit is [meter]
     * 
     */
    float local_search_range_radius = 0.2;
    /**
     * @brief use to switch mutation mode 
     * 
     */
    int h_s = 5;
    /**
     * @brief use to terminate revolution process
     * 
     */
    int h_t = 10;
  };

  struct ParameterCollisionDetection
  {
  };
  struct ParameterPlanner
  {
  };
  struct ParameterContainer
  {
    ParameterPlanner planner_params;
    ParameterCollisionDetection collision_detection_params;
    ParameterGeneticAlgorithm genetic_algorithm_params;
  };

  class ParameterManager
  {

  public:
    ParameterManager() {}

    ParameterManager(const ros::NodeHandle &in)
    {
      nh_ = in;
      param_container_ptr_.reset(new ParameterContainer);
    }

    virtual ~ParameterManager() {}

    virtual void Initialization() {}
    virtual void LoadParams();

    virtual void LoadGeneticAlgorithmParams();
    //load param by param name, member variable, param type
    template <typename T>
    void GetSingleParam(const std::string &param_name, T &param_data);

    //get the ptr of all param
    std::shared_ptr<ParameterContainer> GetAllParams();
    ParameterCollisionDetection GetCollisionDetectionParams();
    ParameterPlanner GetPlannerParams();
    ParameterGeneticAlgorithm GetGeneticAlgorithmParams();

  private:
    std::shared_ptr<ParameterContainer> param_container_ptr_;
    ros::NodeHandle nh_;
  };
}
