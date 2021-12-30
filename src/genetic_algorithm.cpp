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
        Chromosome empty_chromosome;
        PiecewiseCubicBezier piecewise_cubic_bezier = GeneratePiecewiseCubicBezier(empty_chromosome);
        if (collision_detection_ptr_->FindCollsionIndex(piecewise_cubic_bezier) < 0)
        {
            path_ = piecewise_cubic_bezier.ConvertPiecewiseCubicBezierToVector3d();
            current_best_.first = empty_chromosome;
            DLOG(INFO) << "cubic bezier path has been found!!";
            return 1;
        }
        else
        {
            DLOG(INFO) << "cubic bezier is in  collision!";
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
        int h = 0;
        bool inner_flag = true, outer_flag = true;
        if (PreCheck() == 1)
        {
            inner_flag = false;
            outer_flag = false;
        }

        while (outer_flag)
        {
            GenerateInitialPopulation();
            while (inner_flag)
            {
                GenerateFitnessMap();
                GenerateProbabilityMap();
                SelectAndCrossover();
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
                    if (collision_detection_ptr_->FindCollsionIndex(piecewise_cubic_bezier) >= 0)
                    {
                        number_of_gene_in_chromosome_++;
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
                        roll = ((double)rand() / (RAND_MAX));
                        if (roll < (float)h / params_.h_t)
                        {
                            if (h < params_.h_s)
                            {
                                GlobalMutation(chromosome);
                            }
                            else
                            {
                                LocalMutation(chromosome);
                            }
                        }
                    }
                }
            }
        }
    }

    void GeneticAlgorithm::SelectAndCrossover()
    {
        MatingPool mating_pool = GenerateMatingPool();
        generation_.clear();
        for (auto breeding_pair : mating_pool)
        {
            generation_.emplace_back(Crossover(breeding_pair.first, breeding_pair.second));
        }
    }
    Genotype GeneticAlgorithm::GenerateRandomGeno()
    {
        int xi = rand() % grid_->info.width;
        int yi = rand() % grid_->info.height;
        float theta = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2 * M_PI)));
        Genotype out(xi, yi, theta);
        bool flag = true;
        while (flag)
        {
            if (collision_detection_ptr_->IsCollsion(out))
            {
                DLOG(INFO) << "random generated geno is in collision, regenerating...";
                out.x() = rand() % grid_->info.width;
                out.y() = rand() % grid_->info.height;
                out.z() = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2 * M_PI)));
            }
            else
            {
                flag = false;
            }
        }
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
        Chromosome out;
        while (out.size() < number_of_genes)
        {
            out.emplace_back(GenerateRandomGeno());
        }
        bool flag = true;
        while (flag)
        {
            PiecewiseCubicBezier piecewise_cubic_bezier = GeneratePiecewiseCubicBezier(out);
            if (collision_detection_ptr_->FindCollsionIndex(piecewise_cubic_bezier) >= 0)
            {
                DLOG(INFO) << "random generated chromosome is in collision, regenerating...";
                out.clear();
                while (out.size() < number_of_genes)
                {
                    out.emplace_back(GenerateRandomGeno());
                }
            }
            else
            {
                flag = false;
            }
        }
        return out;
    }

    Population GeneticAlgorithm::GenerateRandomPopulation(const uint &population_size)
    {
        Population out;
        while (out.size() < population_size)
        {
            out.emplace_back(GenerateRandomChromosome(number_of_gene_in_chromosome_));
        }
        return out;
    }

    void GeneticAlgorithm::GenerateInitialPopulation()
    {
        generation_ = GenerateRandomPopulation(params_.population_size);
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
        for (auto chromosome : generation_)
        {
            fitness_map_.clear();
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
        float total_fitness = 0;
        float accumulated_fitness = 0;
        for (auto chromosome_pair : fitness_map_)
        {
            total_fitness += chromosome_pair.first;
        }
        for (auto chromosome_pair : fitness_map_)
        {
            probability_map_.clear();
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
        while (mating_pool.size() < params_.population_size)
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
            uint i = 0;
            for (; i < collision_index_1; ++i)
            {
                out.emplace_back(chromosome_1[i]);
            }
            for (; i < chromosome_2.size(); ++i)
            {
                out.emplace_back(chromosome_2[i]);
            }
        }
        else
        {
            uint i = 0;
            for (; i < collision_index_2; ++i)
            {
                out.emplace_back(chromosome_2[i]);
            }
            for (; i < chromosome_1.size(); ++i)
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