#pragma once

#include <ros/ros.h>
namespace GeneticAlgorithm
{
  struct ParameterPathPublisher
  {
    /// [m] --- The cell size of the 2D grid of the world,used in planner.cpp
    double cell_size = 1;
    // #********************** vehicle parameters *****************

    /// [m] --- Uniformly adds a padding around the vehicle
    double bloating = 0;
    /// [m] --- The width of the vehicle
    double vehicle_width = 1.75;
    /// [m] --- The length of the vehicle
    double vehicle_length = 2.65;
  };
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
    double penalty_fitness = 50;
    /**
     * @brief the range for local mutation, unit is [meter],for x and y
     * 
     */
    double local_search_range_radius = 2;

    /**
     * @brief the range for local mutation, unit is [rad],for angle, this value must small than 2*pi
     * 
     */
    double local_search_range_angle = 0.2;
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

    int number_of_points = 50;

    int number_of_gene_in_chromosome = 1;
  };

  struct ParameterCollisionDetection
  {
    double position_resolution = 1;
    int headings = 72;
  };
  struct ParameterPlanner
  {
    /// A flag for the mode (true = manual; false = dynamic). Manual for  map or dynamic for dynamic map. used in planner.cpp
    bool manual = true;

    /// [m] --- The cell size of the 2D grid of the world,used in planner.cpp
    double cell_size = 1;
  };
  struct ParameterContainer
  {
    ParameterPlanner planner_params;
    ParameterCollisionDetection collision_detection_params;
    ParameterGeneticAlgorithm genetic_algorithm_params;
    ParameterPathPublisher path_publisher_params;
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

    ~ParameterManager() {}

    void LoadParams();
    void LoadPathPublisherParams();
    void LoadGeneticAlgorithmParams();
    void LoadPlannerParams();
    void LoadCollisionDetectionParams();
    //load param by param name, member variable, param type
    template <typename T>
    void GetSingleParam(const std::string &param_name, T &param_data);

    //get the ptr of all param
    std::shared_ptr<ParameterContainer> GetAllParams();
    ParameterCollisionDetection GetCollisionDetectionParams();
    ParameterPlanner GetPlannerParams();
    ParameterGeneticAlgorithm GetGeneticAlgorithmParams();
    ParameterPathPublisher GetPathPublisherParams();

  private:
    std::shared_ptr<ParameterContainer> param_container_ptr_;
    ros::NodeHandle nh_;
  };
}
