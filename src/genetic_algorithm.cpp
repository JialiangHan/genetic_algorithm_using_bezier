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
    Chromosome GetPoints()
    {
        Chromosome out;
        PiecewiseCubicBezier
            piecewise_cubic_bezier = GeneratePiecewiseCubicBezier(current_best_.first);
        out = piecewise_cubic_bezier.GetPointsVec();
        return out;
    }
    void MainLoop()
    {
        int h = 0;
        bool inter_flag = true, outer_flag = true;
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
                }
                else
                {
                    best_of_best_ = current_best_;
                    if (h < params.h_s)
                    {
                        h = 0;
                    }
                }
                if (h > params.h_t)
                {
                    PiecewiseCubicBezier piecewise_cubic_bezier = GeneratePiecewiseCubicBezier(current_best_.first);
                    if (Utility::IsCollsion(piecewise_cubic_bezier))
                    {
                        number_of_gene_in_chromosome_++;
                    }
                    else
                    {
                        path_ = piecewise_cubic_bezier.ConvertPiecewiseCubicBezierToVector3d();
                        outer_flag = false;
                    }
                }
                else
                {
                    float roll;
                    for (auto chromosome : generation_)
                    {
                        roll == ((double)rand() / (RAND_MAX));
                        if (roll < (float)h / params.h_t)
                        {
                            if (h < params.h_s)
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

    void SelectAndCrossover()
    {
        MatingPool mating_pool_ = GenerateMatingPool();
        generation_.clear();
        for (auto breeding_pair : mating_pool)
        {
            generation_.emplace_back(Crossover(breeding_pair.first, breeding_pair.second));
        }
    }
    Genotype GeneticAlgorithm::GenerateRandomGeno()
    {
        int xi = rand() % grid->info.width;
        int yi = rand() % grid->info.height;
        float theta = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2 * M_PI)));
        Genotype out(xi, yi, theta);
        bool flag = true;
        while (flag)
        {
            if (collision_detection_ptr->IsCollsion(out))
            {
                DLOG(INFO) << "random generated geno is in collision, regenerating...";
                out.x() = rand() % grid->info.width;
                out.y() = rand() % grid->info.height;
                out.z() = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (2 * M_PI)));
            }
            else
            {
                flag = false;
            }
        }
        return out;
    }

    PiecewiseCubicBezier GeneratePiecewiseCubicBezier(const Chromosome &chromosome)
    {
        PiecewiseCubicBezier piecewise_cubic_bezier(start_, goal_);
        piecewise_cubic_bezier.SetAnchorPoints(chromosome);
        return piecewise_cubic_bezier;
    }

    Chromosome GeneticAlgorithm::GenerateRandomChromosome(const int &number_of_genes)
    {
        Chromosome out;
        while (out.size < number_of_genes)
        {
            out.emplace_back(GenerateRandomGeno());
        }
        bool flag = true;
        while (flag)
        {
            PiecewiseCubicBezier piecewise_cubic_bezier = GeneratePiecewiseCubicBezier(out);
            if (collision_detection_ptr->IsCollsion(piecewise_cubic_bezier))
            {
                DLOG(INFO) << "random generated chromosome is in collision, regenerating...";
                out.clear();
                while (out.size < number_of_genes)
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

    Population GeneticAlgorithm::GenerateRandomPopulation(const int &population_size)
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
        int number_of_times_in_collsion = collision_detection_ptr->GetTimesInCollision(piecewise_cubic_bezier);
        fitness = 1 / (path_length + params_.penalty_fitness * number_of_times_in_collsion);
        return fitness;
    }

    void GeneticAlgorithm::GenerateFitnessMap()
    {
        float current_fitness;
        for (auto chromosome : generate_)
        {
            fitness_map_.clear();
            current_fitness = CalculateFitness(chromosome);
            fitness_map.emplace(current_fitness, chromosome);
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
        while (mating_pool.size() < params.population_size)
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
            for (uint i = 0; i < collision_index_1; ++i)
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
            for (uint i = 0; i < collision_index_2; ++i)
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
        Chromosome new_chromosome = GenerateRandomChromosome(const int &chromosome.size());
        chromosome = new_chromosome;
    }

    void GeneticAlgorithm::LocalMutation(Chromosome &chromosome)
    {
        for (auto gene : chromosome)
        {
            out.x() + = (rand() % (params.local_search_range_radius * 200) - params.local_search_range_radius * 100) / 100.0;
            out.y() + = (rand() % (params.local_search_range_radius * 200) - params.local_search_range_radius * 100) / 100.0;
            out.z() += (rand() % (params.local_search_range_angle * 200) - params.local_search_range_angle * 100) / 100.0;
            out.z() = Utility::RadNormalization(out.z());
        }
    }

    void GeneticAlgorithm::CalculatePath(const Chromosome &chromosome)
    {
        PiecewiseCubicBezier piecewise_cubic_bezier = GeneratePiecewiseCubicBezier(chromosome);
        path_ = piecewise_cubic_bezier.ConvertPiecewiseCubicBezierToVector3d();
    }
}