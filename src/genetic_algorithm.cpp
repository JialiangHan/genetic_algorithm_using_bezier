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
    int GeneticAlgorithm::PreCheck()
    {
        DLOG(INFO) << "Enter PreCheck";
        Chromosome empty_chromosome;
        PiecewiseCubicBezier piecewise_cubic_bezier = GeneratePiecewiseCubicBezier(empty_chromosome);
        if (collision_detection_ptr_->FindCollsionIndex(piecewise_cubic_bezier) < 0)
        {
            path_ = piecewise_cubic_bezier.ConvertPiecewiseCubicBezierToVector3d();
            current_best_.first = empty_chromosome;
            // DLOG(INFO) << "cubic bezier path has been found!!";
            DLOG(INFO) << "Exit PreCheck";
            return 1;
        }
        else
        {
            // DLOG(INFO) << "cubic bezier is in  collision!";
            DLOG(INFO) << "Exit PreCheck";
            return 0;
        }
    }

    Chromosome GeneticAlgorithm::GetPoints()
    {
        Chromosome out;
        PiecewiseCubicBezier
            piecewise_cubic_bezier = GeneratePiecewiseCubicBezier(current_best_.first);
        out = piecewise_cubic_bezier.GetPointsVec();
        return out;
    }
    void GeneticAlgorithm::MainLoop()
    {
        DLOG(INFO) << "Enter the main loop";
        //this value is a switch between global mutation and local mutation
        int h = 0, inner_loop_count = 0, outer_loop_count = 0;
        bool inner_flag = true, outer_flag = true;
        if (PreCheck() == 1)
        {
            inner_flag = false;
            outer_flag = false;
            DLOG(INFO) << "PreCheck ==1, set two flag to false";
        }
        // DLOG(INFO) << "outer flag are " << outer_flag << " inner flag are " << inner_flag;
        while (outer_flag)
        {
            DLOG(INFO) << "outer loop count is " << outer_loop_count;
            // DLOG(INFO) << "outer loop in:";
            GenerateInitialPopulation();
            DLOG(INFO) << "nubmer of genes in chromosome is " << number_of_gene_in_chromosome_;
            inner_flag = true;
            h = 0;
            best_of_best_.second = 0;
            inner_loop_count = 0;
            while (inner_flag)
            {
                DLOG(INFO) << "inner loop count is " << inner_loop_count;
                GenerateFitnessMap();
                GenerateProbabilityMap();
                SelectAndCrossover();
                GenerateFitnessMap();
                // DLOG(INFO) << "select and crossover";
                if (best_of_best_.second >= current_best_.second)
                {
                    h++;
                    DLOG(INFO) << "h has increased one, current h is " << h;
                }
                else
                {
                    best_of_best_ = current_best_;
                    if (h < params_.h_s)
                    {
                        DLOG(INFO) << "h has been set to 0";
                        h = 0;
                    }
                }
                if (h > params_.h_t)
                {
                    PiecewiseCubicBezier piecewise_cubic_bezier = GeneratePiecewiseCubicBezier(current_best_.first);
                    int collision_index = collision_detection_ptr_->FindCollsionIndex(piecewise_cubic_bezier);
                    int times_collsion = collision_detection_ptr_->GetTimesInCollision(piecewise_cubic_bezier);
                    // DLOG(INFO) << "collision index of found path is " << collision_index << " number of time collision " << times_collsion;
                    if (collision_index >= 0)
                    {
                        number_of_gene_in_chromosome_++;
                        DLOG(INFO) << "best path is in collision, increase number of genes";
                        inner_flag = false;
                    }
                    else
                    {
                        path_ = piecewise_cubic_bezier.ConvertPiecewiseCubicBezierToVector3d();
                        DLOG(INFO) << "path has been found!!";
                        inner_flag = false;
                        outer_flag = false;
                    }
                }
                else
                {
                    float roll;
                    for (auto chromosome : generation_)
                    {
                        if (chromosome == current_best_.first)
                        {
                            DLOG(INFO) << "no mutation for current best chromosome.";
                            continue;
                        }
                        roll = ((double)rand() / (RAND_MAX));
                        if (roll < (float)h / params_.h_t)
                        {
                            if (h < params_.h_s)
                            {
                                // DLOG(INFO) << "Global mutation";
                                GlobalMutation(chromosome);
                            }
                            else
                            {
                                // DLOG(INFO) << "local mutation";
                                LocalMutation(chromosome);
                            }
                        }
                    }
                }
                inner_loop_count++;
            }
            DLOG(INFO) << "Inner loop exit";
            outer_loop_count++;
        }
    }

    void GeneticAlgorithm::SelectAndCrossover()
    {
        MatingPool mating_pool = GenerateMatingPool();
        generation_.clear();
        for (auto breeding_pair : mating_pool)
        {
            if (number_of_gene_in_chromosome_ == 1)
            {
                generation_.emplace_back(breeding_pair.first);
            }
            else
            {
                generation_.emplace_back(Crossover(breeding_pair.first, breeding_pair.second));
            }
        }
    }
    Genotype GeneticAlgorithm::GenerateRandomGeno()
    {
        // DLOG(INFO) << "Generate Random Geno in:";
        std::vector<Eigen::Vector3d> free_point_vec = collision_detection_ptr_->GetFreePointVec();
        Genotype out;
        bool flag = true;
        while (flag)
        {
            int index = rand() % free_point_vec.size();
            int theta = rand() % (int)(2 * M_PI);
            out = free_point_vec[index];
            out.z() = theta;
            if (collision_detection_ptr_->IsCollsion(out))
            {
                // DLOG(INFO) << "random generated geno is in collision, regenerating...";
            }
            else
            {
                flag = false;
            }
        }
        // DLOG(INFO) << "Generate Random Geno out.";
        return out;
    }

    PiecewiseCubicBezier GeneticAlgorithm::GeneratePiecewiseCubicBezier(const Chromosome &chromosome)
    {
        PiecewiseCubicBezier piecewise_cubic_bezier(start_, goal_);
        piecewise_cubic_bezier.SetAnchorPoints(chromosome);
        return piecewise_cubic_bezier;
    }

    Chromosome GeneticAlgorithm::GenerateRandomChromosome(const uint &number_of_genes)
    {
        // DLOG(INFO) << "Generate Random Chromosome in:";
        Chromosome out;

        bool flag = true;
        while (flag)
        {
            while (out.size() < number_of_genes)
            {
                out.emplace_back(GenerateRandomGeno());
            }
            // PiecewiseCubicBezier piecewise_cubic_bezier = GeneratePiecewiseCubicBezier(out);
            // if (collision_detection_ptr_->FindCollsionIndex(piecewise_cubic_bezier) >= 0)
            // {
            //     // DLOG(INFO) << "random generated chromosome is in collision, regenerating...";
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
            out.emplace_back(GenerateRandomChromosome(number_of_gene_in_chromosome_));
        }
        // DLOG(INFO) << "Generate Random Population out.";
        return out;
    }

    void GeneticAlgorithm::GenerateInitialPopulation()
    {
        // DLOG(INFO) << "Generate Initial Population in:";
        generation_ = GenerateRandomPopulation(params_.population_size);
        // DLOG(INFO) << "Generate Initial Population out.";
    }

    float GeneticAlgorithm::CalculateFitness(const Chromosome &chromosome)
    {
        float fitness;
        PiecewiseCubicBezier piecewise_cubic_bezier = GeneratePiecewiseCubicBezier(chromosome);
        float path_length = piecewise_cubic_bezier.GetLength();
        int number_of_times_in_collsion = collision_detection_ptr_->GetTimesInCollision(piecewise_cubic_bezier);
        fitness = 1 / (path_length + params_.penalty_fitness * number_of_times_in_collsion);
        return fitness;
    }

    void GeneticAlgorithm::GenerateFitnessMap()
    {
        float current_fitness;
        fitness_map_.clear();
        for (auto chromosome : generation_)
        {
            current_fitness = CalculateFitness(chromosome);
            fitness_map_.emplace(current_fitness, chromosome);
            if (current_fitness >= current_best_.second)
            {
                current_best_.first = chromosome;
                current_best_.second = current_fitness;
            }
        }
    }

    void GeneticAlgorithm::GenerateProbabilityMap()
    {
        probability_map_.clear();
        float total_fitness = 0;
        float accumulated_fitness = 0;
        for (auto chromosome_pair : fitness_map_)
        {
            total_fitness += chromosome_pair.first;
        }
        if (fitness_avg_ > total_fitness / fitness_map_.size())
        {
            DLOG(WARNING) << "avg fitness is lower than previous generation!!!";
        }
        DLOG(INFO) << "previous AVG fitness is " << fitness_avg_;
        DLOG(INFO) << "current avg fitness is " << total_fitness / fitness_map_.size();
        fitness_avg_ = total_fitness / fitness_map_.size();

        for (auto chromosome_pair : fitness_map_)
        {
            accumulated_fitness += chromosome_pair.first;
            probability_map_.emplace(accumulated_fitness / total_fitness, chromosome_pair.second);
        }
    }
    BreedingPair GeneticAlgorithm::Select()
    {
        BreedingPair breeding_pair;
        float roll = 0.0;
        roll = rand() % RAND_MAX / (float)RAND_MAX;
        auto iter_1 = probability_map_.lower_bound(roll);
        roll = rand() % RAND_MAX / (float)RAND_MAX;
        auto iter_2 = probability_map_.lower_bound(roll);
        breeding_pair = std::make_pair(iter_1->second, iter_2->second);
        return breeding_pair;
    }

    MatingPool GeneticAlgorithm::GenerateMatingPool()
    {
        MatingPool mating_pool;
        while (mating_pool.size() < (uint)params_.population_size)
        {

            mating_pool.emplace_back(Select());
        }
        return mating_pool;
    }

    Chromosome GeneticAlgorithm::Crossover(const Chromosome &chromosome_1, const Chromosome &chromosome_2)
    {
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
        return out;
    }

    void GeneticAlgorithm::GlobalMutation(Chromosome &chromosome)
    {
        Chromosome new_chromosome = GenerateRandomChromosome(chromosome.size());
        chromosome = new_chromosome;
    }

    void GeneticAlgorithm::LocalMutation(Chromosome &chromosome)
    {
        for (auto gene : chromosome)
        {
            gene.x() = gene.x() + (float(rand()) / float((RAND_MAX)) * params_.local_search_range_radius - 1 / 2 * params_.local_search_range_radius);
            gene.y() = gene.y() + (float(rand()) / float((RAND_MAX)) * params_.local_search_range_radius - 1 / 2 * params_.local_search_range_radius);
            gene.z() = gene.z() + (float(rand()) / float((RAND_MAX)) * params_.local_search_range_angle - 1 / 2 * params_.local_search_range_angle);

            gene.z() = Utility::RadNormalization(gene.z());
        }
    }

    void GeneticAlgorithm::CalculatePath(const Chromosome &chromosome)
    {
        PiecewiseCubicBezier piecewise_cubic_bezier = GeneratePiecewiseCubicBezier(chromosome);
        path_ = piecewise_cubic_bezier.ConvertPiecewiseCubicBezierToVector3d();
    }
}