#include "planner.h"
#include <chrono>

using namespace GeneticAlgorithm;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner()
{
    //load all params
    param_manager_.reset(new ParameterManager(nh_));
    param_manager_->LoadParams();
    params_ = param_manager_->GetPlannerParams();
    collision_detection_ptr_.reset(new CollisionDetection(param_manager_->GetCollisionDetectionParams()));
    algorithm_ptr_.reset(new GeneticAlgorithm(param_manager_->GetGeneticAlgorithmParams()));
    path_publisher_ptr_.reset(new PathPublisher(param_manager_->GetPathPublisherParams()));
    genetic_algorithm_ptr_.reset(new GeneticAlgorithm(param_manager_->GetGeneticAlgorithmParams()));
    // _________________
    // TOPICS TO PUBLISH
    pub_start_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);

    // ___________________
    // TOPICS TO SUBSCRIBE

    sub_map_ = nh_.subscribe("/map", 1, &Planner::SetMap, this);
    sub_goal_ = nh_.subscribe("/move_base_simple/goal", 1, &Planner::SetGoal, this);
    sub_start_ = nh_.subscribe("/initialpose", 1, &Planner::SetStart, this);
    DLOG(INFO) << "Initialized finished planner!!";
};

//###################################################
//                                                MAP
//###################################################
void Planner::SetMap(const nav_msgs::OccupancyGrid::Ptr &map)
{

    grid_ = map;
    // valid_start_ = true;
    // valid_goal_ = true;
    // MakePlan();
    // plan if the switch is not set to manual and a transform is available
    if (!params_.manual && listener_.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr))
    {

        listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform_);

        // assign the values to start from base_link
        start_.pose.pose.position.x = transform_.getOrigin().x();
        start_.pose.pose.position.y = transform_.getOrigin().y();
        tf::quaternionTFToMsg(transform_.getRotation(), start_.pose.pose.orientation);

        if (grid_->info.height >= start_.pose.pose.position.y && start_.pose.pose.position.y >= 0 &&
            grid_->info.width >= start_.pose.pose.position.x && start_.pose.pose.position.x >= 0)
        {
            // set the start as valid and plan
            valid_start_ = true;
        }
        else
        {
            valid_start_ = false;
        }

        MakePlan();
    }
}

//###################################################
//                                   INITIALIZE START
//###################################################
void Planner::SetStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial)
{
    double x = initial->pose.pose.position.x / params_.cell_size;
    double y = initial->pose.pose.position.y / params_.cell_size;
    double t = tf::getYaw(initial->pose.pose.orientation);
    // publish the start without covariance for rviz
    geometry_msgs::PoseStamped startN;
    startN.pose.position = initial->pose.pose.position;
    startN.pose.orientation = initial->pose.pose.orientation;
    startN.header.frame_id = "map";
    startN.header.stamp = ros::Time::now();

    DLOG(INFO) << "I am seeing a new start x:" << x << " y:" << y << " t:" << Utility::ConvertRadToDeg(t);

    if (grid_->info.height >= y && y >= 0 && grid_->info.width >= x && x >= 0)
    {
        valid_start_ = true;
        start_ = *initial;

        if (params_.manual)
        {
            MakePlan();
        }

        // publish start for RViz
        pub_start_.publish(startN);
    }
    else
    {
        DLOG(INFO) << "invalid start x:" << x << " y:" << y << " t:" << Utility::ConvertRadToDeg(t);
    }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::SetGoal(const geometry_msgs::PoseStamped::ConstPtr &end)
{
    // retrieving goal position
    double x = end->pose.position.x / params_.cell_size;
    double y = end->pose.position.y / params_.cell_size;
    double t = tf::getYaw(end->pose.orientation);

    DLOG(INFO) << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Utility::ConvertRadToDeg(t);

    if (grid_->info.height >= y && y >= 0 && grid_->info.width >= x && x >= 0)
    {
        valid_goal_ = true;
        goal_ = *end;

        if (params_.manual)
        {
            MakePlan();
        }
    }
    else
    {
        DLOG(INFO) << "invalid goal x:" << x << " y:" << y << " t:" << Utility::ConvertRadToDeg(t);
    }
}

//###################################################
//                                      PLAN THE PATH
//###################################################
void Planner::MakePlan()
{

    // if a start as well as goal are defined go ahead and plan
    if (valid_start_ && valid_goal_)
    {
        DLOG(INFO) << "valid start and valid goal, start to make plan!";

        // ___________________________
        // LISTS ALLOCATED ROW MAJOR ORDER
        uint width = grid_->info.width;
        uint height = grid_->info.height;
        DLOG(INFO) << "map size is " << width << " * " << height;

        // ________________________
        // retrieving goal position
        double x = goal_.pose.position.x / params_.cell_size;
        double y = goal_.pose.position.y / params_.cell_size;
        double t = tf::getYaw(goal_.pose.orientation);
        // set theta to a value (0,2PI]
        t = Utility::RadNormalization(t);
        const Eigen::Vector3d goal(x, y, t);
        DLOG(INFO) << "goal x:" << x << " y:" << y << " t:" << Utility::ConvertRadToDeg(t);
        // const Eigen::Vector3d goal(55, 5, 0);
        // const Eigen::Vector3d goal(22, 20, 0);
        // _________________________
        // retrieving start position
        x = start_.pose.pose.position.x / params_.cell_size;
        y = start_.pose.pose.position.y / params_.cell_size;
        t = tf::getYaw(start_.pose.pose.orientation);
        // set theta to a value (0,2PI]
        t = Utility::RadNormalization(t);
        const Eigen::Vector3d start(x, y, t);
        DLOG(INFO) << "start x:" << x << " y:" << y << " t:" << Utility::ConvertRadToDeg(t);
        // const Eigen::Vector3d start(2, 28, 0);

        std::vector<Eigen::Vector3d> path, points;
        std::vector<double> fitness_vec;
        // //check piecewise cubic bezier

        // // ___________________________
        // START AND TIME THE PLANNING
        bool use_ga = true;
        if (use_ga == true)
        {

            auto t1 = std::chrono::high_resolution_clock::now();
            // GeneticAlgorithm ga(param_manager_->GetGeneticAlgorithmParams());
            // ga.Initialize(start, goal, grid_);
            // path = ga.GetPath();
            // points = ga.GetPoints();
            // fitness_vec = ga.GetFitnessVec();
            genetic_algorithm_ptr_->Initialize(start, goal, grid_);
            path = genetic_algorithm_ptr_->GetPath();
            points = genetic_algorithm_ptr_->GetPoints();
            fitness_vec = genetic_algorithm_ptr_->GetFitnessVec();
            auto t2 = std::chrono::high_resolution_clock::now();

            std::chrono::duration<double, std::milli> ms_double = t2 - t1;

            LOG(INFO) << "TIME in ms: " << ms_double.count() << " frequency is : " << 1 / (ms_double.count() / 1000) << " Hz"
                      << " number of generation is " << fitness_vec.size() << " final fitness value is " << fitness_vec.back();
        }
        // CLEAR THE PATH
        path_publisher_ptr_->Clear();
        // CREATE THE UPDATED PATH
        path_publisher_ptr_->UpdatePath(path);
        path_publisher_ptr_->UpdatePoint(points);
        path_publisher_ptr_->UpdateFitness(fitness_vec);
        // _________________________________
        // PUBLISH THE RESULTS OF THE SEARCH
        path_publisher_ptr_->PublishPath();
        path_publisher_ptr_->PublishPathPoints();
        path_publisher_ptr_->PublishFitness();

        valid_start_ = false;
        valid_goal_ = false;
    }
    else
    {
        DLOG(INFO) << "missing goal or start";
    }
}