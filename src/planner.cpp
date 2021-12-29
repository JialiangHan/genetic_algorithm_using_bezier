#include "planner.h"

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
    // _________________
    // TOPICS TO PUBLISH
    pub_start_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);

    // ___________________
    // TOPICS TO SUBSCRIBE
    if (params_.manual)
    {
        sub_map_ = nh_.subscribe("/map", 1, &Planner::SetMap, this);
    }
    else
    {
        sub_map_ = nh_.subscribe("/occ_map", 1, &Planner::SetMap, this);
    }

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

    // collision_detection_ptr_->SetMap(map);
    //create array for Voronoi diagram
    //  ros::Time t0 = ros::Time::now();
    // int height = map->info.height;
    // int width = map->info.width;
    // bool **binMap;
    // binMap = new bool *[width];

    // for (int x = 0; x < width; x++)
    // {
    //   binMap[x] = new bool[height];
    // }

    // for (int x = 0; x < width; ++x)
    // {
    //   for (int y = 0; y < height; ++y)
    //   {
    //     binMap[x][y] = map->data[y * width + x] ? true : false;
    //   }
    // }

    // voronoi_diagram_.initializeMap(width, height, binMap);
    // voronoi_diagram_.update();
    // voronoi_diagram_.visualize();
    //  ros::Time t1 = ros::Time::now();
    //  ros::Duration d(t1 - t0);
    //  DLOG(INFO) << "created Voronoi Diagram in ms: " << d * 1000 ;

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
    float x = initial->pose.pose.position.x / params_.cell_size;
    float y = initial->pose.pose.position.y / params_.cell_size;
    float t = tf::getYaw(initial->pose.pose.orientation);
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
    float x = end->pose.position.x / params_.cell_size;
    float y = end->pose.position.y / params_.cell_size;
    float t = tf::getYaw(end->pose.orientation);

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
        int width = grid_->info.width;
        int height = grid_->info.height;
        // int depth = params_.headings;
        // int length = width * height * depth;

        // ________________________
        // retrieving goal position
        float x = goal_.pose.position.x / params_.cell_size;
        float y = goal_.pose.position.y / params_.cell_size;
        float t = tf::getYaw(goal_.pose.orientation);
        // set theta to a value (0,2PI]
        t = Utility::RadNormalization(t);
        const Eigen::Vector3d goal(x, y, t);
        // __________
        // DEBUG GOAL
        //    const Node3D nGoal(155.349, 36.1969, 0.7615936, 0, 0, nullptr);

        // _________________________
        // retrieving start position
        x = start_.pose.pose.position.x / params_.cell_size;
        y = start_.pose.pose.position.y / params_.cell_size;
        t = tf::getYaw(start_.pose.pose.orientation);
        // set theta to a value (0,2PI]
        t = Utility::RadNormalization(t);

        const Eigen::Vector3d start(x, y, t);
        // ___________
        // DEBUG START
        //    Node3D nStart(108.291, 30.1081, 0, 0, 0, nullptr);

        // ___________________________
        // START AND TIME THE PLANNING
        ros::Time t0 = ros::Time::now();
        // Eigen::Vector2d p1(4.06, 15.67), p2(7.74, 11.88), p3(14.05, 13.54), p4(19.6, 12.77), p5(25.19, 12.00), p6(30.01, 8.79), p7(35.17, 9.88);
        // std::vector<Eigen::Vector2d> points_vec{p1, p2, p3, p4};

        // cubic_bezier_ = CubicBezier::CubicBezier(points_vec);
        // cubic_bezier_ = CubicBezier::CubicBezier(start, goal, width, height);
        // std::vector<Eigen::Vector2d> control_points;
        // Eigen::Vector2d control_point;
        // control_point.x() = params_.control_point_1_x;
        // control_point.y() = params_.control_point_1_y;
        // control_points.emplace_back(control_point);
        // control_point.x() = params_.control_point_2_x;
        // control_point.y() = params_.control_point_2_y;
        // control_points.emplace_back(control_point);
        // cubic_bezier_.SetControlPoints(control_points);
        // std::vector<Eigen::Vector3d> path;
        // path = cubic_bezier_.ConvertCubicBezierToVector3d();
        piecewise_cubic_bezier_ = PiecewiseCubicBezier(start, goal, width, height);
        std::vector<Eigen::Vector3d> anchor_points;
        uint number_of_anchor_points = 0;
        for (uint i = 0; i < number_of_anchor_points; ++i)
        {
            Eigen::Vector3d anchor_point;
            if (anchor_points.size() == 0)
            {
                anchor_point = ((start + goal) / 2);
            }
            else
            {
                anchor_point = (anchor_points.back() + goal) / 2;
            }
            anchor_points.emplace_back(anchor_point);
        }

        piecewise_cubic_bezier_.SetAnchorPoints(anchor_points);
        std::vector<Eigen::Vector3d> path = piecewise_cubic_bezier_.ConvertPiecewiseCubicBezierToVector3d();
        std::vector<Eigen::Vector2d> points = piecewise_cubic_bezier_.GetPointsVec();
        // DLOG(INFO) << "path length is : " << path.size() << " first point is: " << path.front().x() << " " << path.front().y() << " last point is : " << path.back().x() << " " << path.back().y();
        // CLEAR THE VISUALIZATION
        // visualization_ptr_->clear();
        // CLEAR THE PATH
        path_publisher_ptr_->Clear();
        // smoothed_path_ptr_->Clear();
        // FIND THE PATH
        // Node3D *nSolution = algorithm_ptr_->HybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configuration_space_ptr_, dubins_lookup_table, visualization_ptr_);
        // TRACE THE PATH
        // smoother_ptr_->TracePath(nSolution);
        // CREATE THE UPDATED PATH
        path_publisher_ptr_->UpdatePath(path);
        path_publisher_ptr_->UpdatePoint(points);
        // SMOOTH THE PATH
        // smoother_ptr_->SmoothPath(voronoi_diagram_);
        // CREATE THE UPDATED PATH
        // smoothed_path_ptr_->UpdatePath(smoother_ptr_->GetPath());
        ros::Time t1 = ros::Time::now();
        ros::Duration d(t1 - t0);
        DLOG(INFO) << "TIME in ms: " << d * 1000;

        // _________________________________
        // PUBLISH THE RESULTS OF THE SEARCH
        path_publisher_ptr_->PublishPath();
        path_publisher_ptr_->PublishPathNodes();
        path_publisher_ptr_->PublishPathVehicles();
        path_publisher_ptr_->PublishPathPoints();
        // smoothed_path_ptr_->PublishPath();
        // smoothed_path_ptr_->PublishPathNodes();
        // smoothed_path_ptr_->PublishPathVehicles();
        // visualization_ptr_->publishNode3DCosts(nodes3D, width, height, depth);
        // visualization_ptr_->publishNode2DCosts(nodes2D, width, height);
        //set these two flag to false when finished planning to avoid unwanted planning
        valid_start_ = false;
        valid_goal_ = false;
    }
    else
    {
        DLOG(INFO) << "missing goal or start";
    }
}