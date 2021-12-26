/**
 * @file genetic_algorithm.h
 * @author Jialiang Han
 * @brief main class for GA
 * @version 0.1
 * @date 2021-12-25
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once
#include <vector>
#include <eigen3/Eigen/Dense>
#include "piecewise_cubic_bezier.h"
#include "parameter_manager.h"

namespace GeneticAlgorithm
{
   /**
    * @brief control point of bezier
    * 
    */
   typedef Eigen::Vector2d Genotype;
   /**
    * @brief list of control point, not include start and goal
    * 
    */
   typedef std::vector<Genotype> Chromosome;
   typedef std::vector<Chromosome> Population;
   /*!
   * \brief A class that encompasses the functions central to the search.
   */
   class GeneticAlgorithm
   {
      GeneticAlgorithm(){};
      GeneticAlgorithm(const ParameterGeneticAlgorithm &param)
      {
         params_ = param;
      };

   public:
      Population GenerateInitialPopulation();

      /**
       * @brief select based on probability density function
       * 
       * @param chromosome 
       * @return Population 
       */
      Population Select(const Chromosome &chromosome);
      /**
       * @brief find the location j chromosome_1 first encouter obstacle, transfer  0~ j-1 genes to chromosome_2
       * 
       * @param chromosome_1 
       * @param chromosome_2 
       */
      void Crossover(Chromosome &chromosome_1, Chromosome &chromosome_2);
      Population Mutation();
      /**
       * @brief randomly reproduce a free anchor point set
       * 
       * @param chromosome_1 
       */
      void GlobalMutation(Chromosome &chromosome_1);
      /**
       * @brief for each free anchor point, modify thse points within a predefined range(local search range radius)
       * 
       * @param chromosome_1 
       */
      void LocalMutation(Chromosome &chromosome_1);
      /**
       * @brief Calculate fitness value of a path,
             * equation is f(x)=1/(L(path)+p*q)
             * L(path): path length
             * p: penalty
             * q: number of times path encouters obstacle
       * 
       * @param chromosome 
       * @return float 
       */
      float CalculateFitness(const Chromosome &chromosome);

   private:
      ParameterGeneticAlgorithm params_;
      /**
       * @brief historical best chromosome which has highest fitness value
       * 
       */
      Chromosome best_of_best_;
      /**
       * @brief number of generations which f(best of best)>= f(x best)
       * 
       */
      int h_;
   };
}
