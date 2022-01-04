/**
 * @file genetic_algorithm.cpp
 * @author Jialiang Han (hanjiali@umich.edu)
 * @brief this is a definations for GA
 * @version 0.1
 * @date 2021-12-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "genetic_algorithm.h"

namespace GeneticAlgorithm
{
    GeneticAlgorithm::~GeneticAlgorithm()
    {
        //detach our threads
        pub_thread_->join();
    };
    int GeneticAlgorithm::PreCheck()
    {
        // DLOG(INFO) << "Enter PreCheck";
        Chromosome empty_chromosome;
        PiecewiseCubicBezier piecewise_cubic_bezier = GeneratePiecewiseCubicBezier(empty_chromosome);
        int collision_index = collision_detection_ptr_->FindCollsionIndex(piecewise_cubic_bezier);
        // int times_collsion = collision_detection_ptr_->GetTimesInCollision(piecewise_cubic_bezier);
        // DLOG(INFO) << "collision index of found path is " << collision_index;
        if (collision_index < 0)
        {
            path_ = piecewise_cubic_bezier.ConvertPiecewiseCubicBezierToVector3d(params_.number_of_points);
            current_best_.first = empty_chromosome;
            current_best_.second = CalculateFitness(empty_chromosome);
            fitness_vec_.emplace_back(current_best_.second);
            // DLOG(INFO) << "Exit PreCheck, cubic bezier path has been found!";
            return 1;
        }
        else
        {
            // DLOG(INFO) << "Exit PreCheck, cubic bezier is in  collision!";
            return 0;
        }
    }

    Chromosome GeneticAlgorithm::GetPoints()
    {
        // DLOG(INFO) << "GetPoints in:";
        Chromosome out;
        PiecewiseCubicBezier
            piecewise_cubic_bezier = GeneratePiecewiseCubicBezier(current_best_.first);
        out = piecewise_cubic_bezier.GetPointsVec();
        // DLOG(INFO) << "GetPoints out.";
        return out;
    }
    void GeneticAlgorithm::MainLoop()
    {
        DLOG(INFO) << "Enter the main loop";
        // fitness_vec_.clear();
        std::vector<double>().swap(fitness_vec_);
        //this value is a switch between global mutation and local mutation
        int h = 0, inner_loop_count = 0, outer_loop_count = 0;
        bool inner_flag = true, outer_flag = true;
        if (PreCheck() == 1)
        {
            inner_flag = false;
            outer_flag = false;
            DLOG(INFO) << "PreCheck ==1, set two flag to false";
        }
        while (outer_flag)
        {
            DLOG(INFO) << "outer loop count is " << outer_loop_count;
            GenerateFreePointMap();

            GenerateInitialPopulation();

            // DLOG(INFO) << "nubmer of genes in chromosome is " << params_.number_of_gene_in_chromosome;
            inner_flag = true;
            h = 0;
            best_of_best_.second = 0;
            inner_loop_count = 0;
            while (inner_flag)
            {
                current_best_.second = 0;
                // DLOG(INFO) << "inner loop count is " << inner_loop_count;
                GenerateFitnessMap();

                GenerateProbabilityMap();

                SelectAndCrossover();

                GenerateFitnessMap();

                // DLOG(INFO) << "select and crossover";
                fitness_vec_.emplace_back(current_best_.second);
                if (best_of_best_.second >= current_best_.second)
                {
                    h++;
                    // DLOG(INFO) << "current generation is not better than the best of best ,h has increased one, current h is " << h;
                }
                else
                {
                    best_of_best_ = current_best_;
                    // if (h < params_.h_s)
                    // {
                    // DLOG(INFO) << "h has been set to 0";
                    h = 0;
                    // }
                }
                if (h > params_.h_t)
                {
                    PiecewiseCubicBezier piecewise_cubic_bezier = GeneratePiecewiseCubicBezier(current_best_.first);
                    int collision_index = collision_detection_ptr_->FindCollsionIndex(piecewise_cubic_bezier);

                    path_ = piecewise_cubic_bezier.ConvertPiecewiseCubicBezierToVector3d(params_.number_of_points);
                    if (collision_index >= 0)
                    {
                        params_.number_of_gene_in_chromosome++;
                        DLOG(INFO) << "best path is in collision, increase number of genes";
                        inner_flag = false;
                    }
                    else
                    {
                        DLOG(INFO) << "path has been found!!";
                        inner_flag = false;
                        outer_flag = false;
                    }
                }
                else
                {
                    double roll;
                    int number_of_current_best = 0;
                    // DLOG(INFO) << " generation_  size is " << generation_.size();
                    // int index = 0, index_1 = 0;
                    for (auto &chromosome : generation_)
                    {

                        // for (const auto &point : chromosome)
                        // {
                        // DLOG(INFO) << index << "th point x: " << point.x() << " y " << point.y() << " z " << point.z();
                        //     index++;
                        // }
                        if (chromosome == current_best_.first && number_of_current_best == 0)
                        {
                            // DLOG(INFO) << "no mutation for current best chromosome.";
                            number_of_current_best++;
                            continue;
                        }
                        roll = ((double)rand() / (RAND_MAX));
                        if (roll < (double)h / params_.h_t)
                        {
                            if (h < params_.h_s)
                            {
                                GlobalMutation(chromosome);
                                // DLOG(INFO) << "Global mutation, after: x " << chromosome.front().x() << " y " << chromosome.front().y() << " z " << chromosome.front().z();
                            }
                            else
                            {
                                LocalMutation(chromosome);
                                // DLOG(INFO) << "local mutation, after: x " << chromosome.front().x() << " y " << chromosome.front().y() << " z " << chromosome.front().z();
                            }
                        }
                        // index_1++;
                    }
                }
                inner_loop_count++;
                // PiecewiseCubicBezier piecewise_cubic_bezier = GeneratePiecewiseCubicBezier(current_best_.first);
                // path_ = piecewise_cubic_bezier.ConvertPiecewiseCubicBezierToVector3d(params_.number_of_points);
                // return;
            }
            // DLOG(INFO) << "Inner loop exit";
            outer_loop_count++;
            // return;
        }
    }

    void GeneticAlgorithm::SelectAndCrossover()
    {
        // for (const auto &chromosome : generation_)
        // {
        //     for (const auto &point : chromosome)
        //     {
        // DLOG(INFO) << "point x: " << point.x() << " y " << point.y() << " z " << point.z();
        //     }
        // }
        MatingPool mating_pool = GenerateMatingPool();

        generation_.clear();
        for (const auto &breeding_pair : mating_pool)
        {

            // for (const auto &point : breeding_pair.first)
            // {
            // DLOG(INFO) << "point x: " << point.x() << " y " << point.y() << " z " << point.z();
            // }
            // for (const auto &point : breeding_pair.second)
            // {
            // DLOG(INFO) << "point x: " << point.x() << " y " << point.y() << " z " << point.z();
            // }
            if (params_.number_of_gene_in_chromosome == 1)
            {
                generation_.emplace_back(breeding_pair.first);
            }
            else
            {
                generation_.emplace_back(Crossover(breeding_pair.first, breeding_pair.second));
            }
        }
        // for (const auto &chromosome : generation_)
        // {
        //     for (const auto &point : chromosome)
        //     {
        // DLOG(INFO) << "point x: " << point.x() << " y " << point.y() << " z " << point.z();
        //     }
        // }
    }
    Genotype GeneticAlgorithm::GenerateRandomGeno(const int &digit)
    {
        // DLOG(INFO) << "Generate Random Geno in:";

        Genotype out;
        bool flag = true;
        while (flag)
        {
            out = free_points_map_[digit].back();
            if (collision_detection_ptr_->IsCollsion(out))
            {

                // DLOG(INFO) << "random generated geno is in collision, regenerating...";
            }
            else
            {

                flag = false;
            }
            free_points_map_[digit].pop_back();
        }
        // DLOG(INFO) << "Generate Random Geno out.";
        return out;
    }

    PiecewiseCubicBezier GeneticAlgorithm::GeneratePiecewiseCubicBezier(const Chromosome &chromosome)
    {
        // DLOG(INFO) << "GeneratePiecewiseCubicBezier in:";
        PiecewiseCubicBezier piecewise_cubic_bezier(start_, goal_);
        piecewise_cubic_bezier.SetAnchorPoints(chromosome);
        // DLOG(INFO) << "GeneratePiecewiseCubicBezier out.";
        return piecewise_cubic_bezier;
    }

    Chromosome GeneticAlgorithm::GenerateRandomChromosome(const uint &number_of_genes)
    {
        // DLOG(INFO) << "Generate Random Chromosome in:";
        Chromosome out;

        bool flag = true;
        int digit = 0;
        while (flag)
        {
            while (out.size() < number_of_genes)
            {
                Genotype random_geno = GenerateRandomGeno(digit);
                bool insert_flag = true;
                for (const auto &geno : out)
                {
                    if (geno.x() == random_geno.x() && geno.y() == random_geno.y())
                    {
                        insert_flag = false;
                    }
                }
                if (insert_flag)
                {
                    out.emplace_back(random_geno);
                    digit++;
                }
            }
            // PiecewiseCubicBezier piecewise_cubic_bezier = GeneratePiecewiseCubicBezier(out);
            // if (collision_detection_ptr_->FindCollsionIndex(piecewise_cubic_bezier) >= 0)
            // {
            // DLOG(INFO) << "random generated chromosome is in collision, regenerating...";
            //     out.clear();
            // }
            // else
            // {
            flag = false;
            // }
        }
        // DLOG(INFO) << "Generate Random Chromosome out:";
        return out;
    }

    Population GeneticAlgorithm::GenerateRandomPopulation(const uint &population_size)
    {
        // DLOG(INFO) << "Generate Random Population in:";
        Population out;
        while (out.size() < population_size)
        {
            Chromosome current;
            bool insert_flag = true;
            current = GenerateRandomChromosome(params_.number_of_gene_in_chromosome);
            for (const auto &chromosome : out)
            {
                if (chromosome == current)
                {
                    insert_flag = false;
                }
            }
            // DLOG(INFO) << "current chromosome is : x " << current.front().x() << " y " << current.front().y() << " z " << current.front().z();
            if (insert_flag)
            {
                out.emplace_back(current);
            }
        }
        // DLOG(INFO) << "Generate Random Population out.";
        return out;
    }

    void GeneticAlgorithm::GenerateInitialPopulation()
    {
        // DLOG(INFO) << "Generate Initial Population in:";
        generation_ = GenerateRandomPopulation(params_.population_size);
        // DLOG(INFO) << "Generate Initial Population out.";
        // for (const auto &chromosome : generation_)
        // {
        //     for (const auto &point : chromosome)
        //     {
        // DLOG(INFO) << "point x: " << point.x() << " y " << point.y() << " z " << point.z();
        //     }
        // }
    }

    double GeneticAlgorithm::CalculateFitness(const Chromosome &chromosome)
    {
        // DLOG(INFO) << "CalculateFitness in:";
        // for (const auto &point : chromosome)
        // {
        // DLOG(INFO) << "point x: " << point.x() << " y " << point.y() << " z " << point.z();
        // }
        double fitness;
        PiecewiseCubicBezier piecewise_cubic_bezier = GeneratePiecewiseCubicBezier(chromosome);
        double path_length = piecewise_cubic_bezier.GetLength();
        int number_of_times_in_collsion = collision_detection_ptr_->GetTimesInCollision(piecewise_cubic_bezier);
        double total_curvature = piecewise_cubic_bezier.GetTotalCurvature();
        fitness = 10000 / (path_length + params_.penalty_fitness * number_of_times_in_collsion + total_curvature);
        // DLOG_IF(INFO, std::isnan(fitness)) << "current fitness is " << fitness << " path length is " << path_length << " penalty fitness is " << params_.penalty_fitness << " number of times in collision :" << number_of_times_in_collsion << " total curvature is " << total_curvature;
        // DLOG(INFO) << "current fitness is " << fitness << " path length is " << path_length << " for obstacle is " << params_.penalty_fitness * number_of_times_in_collsion << " total curvature is " << total_curvature;
        // DLOG(INFO) << "CalculateFitness out.";
        return fitness;
    }

    void GeneticAlgorithm::GenerateFitnessMap()
    {
        // DLOG(INFO) << "GenerateFitnessMap in:";
        double current_fitness;
        fitness_map_.clear();
        for (const auto &chromosome : generation_)
        {
            current_fitness = CalculateFitness(chromosome);

            fitness_map_.emplace_back(std::make_pair(current_fitness, chromosome));
            // DLOG(INFO) << "current fitness is " << current_fitness;
            // for (const auto &point : chromosome)
            // {
            // DLOG(INFO) << "point x: " << point.x() << " y " << point.y() << " z " << point.z();
            // }
            if (current_fitness >= current_best_.second)
            {
                current_best_.first = chromosome;
                current_best_.second = current_fitness;
            }
        }
        // DLOG(INFO) << "GenerateFitnessMap out:";
        // DLOG(INFO) << "fitness map size is " << fitness_map_.size();
    }

    void GeneticAlgorithm::GenerateProbabilityMap()
    {
        // DLOG(INFO) << "GenerateProbabilityMap in:";
        probability_map_.clear();

        double total_fitness = 0;
        double accumulated_fitness = 0;
        // DLOG(INFO) << "size of fitness map is " << fitness_map_.size();
        for (const auto &chromosome_pair : fitness_map_)
        {
            total_fitness += chromosome_pair.first;
        }
        // DLOG(INFO) << "total fitness is " << total_fitness;
        // if (fitness_avg_ > total_fitness / fitness_map_.size())
        // {
        //     DLOG(WARNING) << "avg fitness is lower than previous generation!!!";
        // }
        // DLOG_IF(INFO, fitness_avg_ > total_fitness / fitness_map_.size()) << "previous AVG fitness is " << fitness_avg_ << " current avg fitness is " << total_fitness / fitness_map_.size();
        fitness_avg_ = total_fitness / fitness_map_.size();
        // fitness_vec_.emplace_back(fitness_avg_);
        for (const auto &chromosome_pair : fitness_map_)
        {
            accumulated_fitness += chromosome_pair.first;
            // DLOG(INFO) << "accumulated fitness is " << accumulated_fitness;
            probability_map_.emplace(accumulated_fitness / total_fitness, chromosome_pair.second);
            // for (const auto &point : chromosome_pair.second)
            // {
            // DLOG(INFO) << "accumulated fitness is " << accumulated_fitness / total_fitness << " point x: " << point.x() << " y " << point.y() << " z " << point.z();
            // }
        }
        // DLOG(INFO) << "GenerateProbabilityMap out.";
    }
    BreedingPair GeneticAlgorithm::Select()
    {
        // DLOG(INFO) << "Select in:";
        BreedingPair breeding_pair;
        double roll = 0.0;
        roll = rand() % RAND_MAX / (double)RAND_MAX;
        // DLOG(INFO) << "rol is " << roll;
        auto iter_1 = probability_map_.lower_bound(roll);
        roll = rand() % RAND_MAX / (double)RAND_MAX;
        // DLOG(INFO) << "rol is " << roll;
        auto iter_2 = probability_map_.lower_bound(roll);
        breeding_pair = std::make_pair(iter_1->second, iter_2->second);
        // for (const auto &point : breeding_pair.first)
        // {
        // DLOG(INFO) << "point x: " << point.x() << " y " << point.y() << " z " << point.z();
        // }
        // for (const auto &point : breeding_pair.second)
        // {
        // DLOG(INFO) << "point x: " << point.x() << " y " << point.y() << " z " << point.z();
        // }
        // DLOG(INFO) << "Select out.";
        return breeding_pair;
    }

    MatingPool GeneticAlgorithm::GenerateMatingPool()
    {
        // DLOG(INFO) << "GenerateMatingPool in:";
        MatingPool mating_pool;
        while (mating_pool.size() < (uint)params_.population_size)
        {

            mating_pool.emplace_back(Select());
        }
        // DLOG(INFO) << "GenerateMatingPool out.";
        return mating_pool;
    }

    Chromosome GeneticAlgorithm::Crossover(const Chromosome &chromosome_1, const Chromosome &chromosome_2)
    {
        // DLOG(INFO) << "Crossover in:";
        Chromosome out;

        int collision_index_1 = collision_detection_ptr_->FindCollsionIndex(GeneratePiecewiseCubicBezier(chromosome_1));
        int collision_index_2 = collision_detection_ptr_->FindCollsionIndex(GeneratePiecewiseCubicBezier(chromosome_2));

        if (collision_index_1 < 0)
        {
            out = chromosome_1;
        }
        else if (collision_index_1 > collision_index_2)
        {
            int i = 0;
            for (; i < collision_index_1; ++i)
            {
                out.emplace_back(chromosome_1[i]);
            }
            for (; (uint)i < chromosome_2.size(); ++i)
            {
                out.emplace_back(chromosome_2[i]);
            }
        }
        else
        {
            int i = 0;
            for (; i < collision_index_2; ++i)
            {
                out.emplace_back(chromosome_2[i]);
            }
            for (; (uint)i < chromosome_1.size(); ++i)
            {
                out.emplace_back(chromosome_1[i]);
            }
        }
        // DLOG(INFO) << "Crossover out.";
        return out;
    }

    void GeneticAlgorithm::GenerateFreePointMap()
    {
        // DLOG(INFO) << "GenerateFreePointMap in:";
        free_points_map_.clear();
        std::vector<Eigen::Vector3d> free_points_vec = collision_detection_ptr_->GetFreePointVec();
        for (auto index = 0; index < params_.number_of_gene_in_chromosome; ++index)
        {
            std::vector<Eigen::Vector3d> temp_vector;
            for (auto &point : free_points_vec)
            {
                for (uint i = 0; i < 7; ++i)
                {
                    point.z() = i;
                    temp_vector.emplace_back(point);
                }
            }
            free_points_map_.emplace_back(temp_vector);
            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::shuffle(free_points_map_[index].begin(), free_points_map_[index].end(), std::default_random_engine(seed));
        }
        // DLOG(INFO) << "free points map size is " << free_points_map_.size();
        // DLOG(INFO) << "first vector size in free points map is " << free_points_map_[0].size();
        // DLOG(INFO) << "GenerateFreePointMap out.";
    }
    void GeneticAlgorithm::GlobalMutation(Chromosome &chromosome)

    {
        // DLOG(INFO) << "Global mutation is in:";
        int chromosome_size = chromosome.size();

        Chromosome new_chromosome = GenerateRandomChromosome(chromosome_size);
        chromosome = new_chromosome;
        // DLOG(INFO) << "Global mutation is out!";
    }

    void GeneticAlgorithm::LocalMutation(Chromosome &chromosome)
    {
        // DLOG(INFO) << "LocalMutation in:";
        for (auto &gene : chromosome)
        {
            gene.x() = gene.x() + ((rand() % 10) / 10.0 * params_.local_search_range_radius - 1 / 2 * params_.local_search_range_radius);
            gene.y() = gene.y() + ((rand() % 10) / 10.0 * params_.local_search_range_radius - 1 / 2 * params_.local_search_range_radius);
            gene.z() = gene.z() + ((rand() % 10) / 10.0 * params_.local_search_range_angle - 1 / 2 * params_.local_search_range_angle);

            gene.z() = Utility::RadNormalization(gene.z());
        }
        // DLOG(INFO) << "LocalMutation out.";
    }

    void GeneticAlgorithm::CalculatePath(const Chromosome &chromosome)
    {
        // DLOG(INFO) << "CalculatePath in:";
        PiecewiseCubicBezier piecewise_cubic_bezier = GeneratePiecewiseCubicBezier(chromosome);
        path_ = piecewise_cubic_bezier.ConvertPiecewiseCubicBezierToVector3d(params_.number_of_points);
        // DLOG(INFO) << "CalculatePath out.";
    }
    void GeneticAlgorithm::PublishThread()
    {
        ros::Rate r(1);
        while (ros::ok())
        {
            // PublishCurrentBestPath();
            // PublishCurrentPointsOfBestPath();
            // PublishFitness();
        }
    }

    int GeneticAlgorithm::PublishFitness()
    {
        std::unique_lock<std::mutex> lock(path_access_);
        genetic_algorithm_using_bezier::FitnessMsgVec msg;
        msg.fitness_vec = fitness_vec_;
        pub_fitness_.publish(msg);
        return 1;
    }
    int GeneticAlgorithm::PublishCurrentPointsOfBestPath()
    {
        std::unique_lock<std::mutex> lock(path_access_);
        visualization_msgs::Marker pathNode;
        visualization_msgs::MarkerArray path_nodes;
        int i = 0;
        Chromosome point_vec;
        if (current_best_.first.size() != 0 && current_best_.second != 0)
        {
            point_vec = GetPoints();
        }

        for (const auto &point : point_vec)
        {
            pathNode.action = 0;

            pathNode.header.frame_id = "path";
            pathNode.header.stamp = ros::Time(0);
            pathNode.id = i;
            pathNode.type = visualization_msgs::Marker::SPHERE;
            pathNode.scale.x = 0.5;
            pathNode.scale.y = 0.5;
            pathNode.scale.z = 0.5;
            pathNode.color.a = 1.0;

            pathNode.color.r = 1;
            pathNode.color.g = 0;
            pathNode.color.b = 0;

            pathNode.pose.position.x = point.x();
            pathNode.pose.position.y = point.y();
            path_nodes.markers.push_back(pathNode);
            i++;
        }
        pub_path_nodes_.publish(path_nodes);
        return 1;
    }
    int GeneticAlgorithm::PublishCurrentBestPath()
    {
        std::unique_lock<std::mutex> lock(path_access_);
        nav_msgs::Path best_path;
        best_path = Utility::ConvertVectorVector3DToRosPath(path_);
        best_path.header.frame_id = "path";
        pub_best_path_.publish(best_path);
        return 1;
    }
}