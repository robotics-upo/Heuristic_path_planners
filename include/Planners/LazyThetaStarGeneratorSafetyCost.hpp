#ifndef LAZYTHETASTARSAFETYCOST_HPP
#define LAZYTHETASTARSAFETYCOST_HPP
/**
 * @file CostAwareLazyThetaStarGenerator.hpp
 * @author Rafael Rey (rreyarc@upo.es)
 * @brief 
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Planners/ThetaStarGeneratorSafetyCost.hpp>

namespace Planners
{
    /**
     * @brief Lazy Theta* Algorithm Class
     * 
     */
    class LazyThetaStarGeneratorSafetyCost : public ThetaStarGeneratorSafetyCost
    {

    public:
        
        /**
         * @brief Construct a new Lazy Theta Star Generator object
         * 
         * @param _use_3d This parameter allows the user to choose between planning on a plane (8 directions possibles) or in the 3D full space (26 directions)
         */
        LazyThetaStarGeneratorSafetyCost(bool _use_3d, std::string _name );
        LazyThetaStarGeneratorSafetyCost(bool _use_3d);

        /**
         * @brief Main function of the algorithm
         * 
         * @param _source Start discrete coordinates
         * @param _target Goal discrete coordinates
         * @return PathData PathData Results stored as PathData object
         */
        virtual PathData findPath(const Vec3i &_source, const Vec3i &_target) override;

    protected:

        /**
         * @brief Compute cost function of the Lazy version of the algorithm
         * 
         * @param s_aux Pointer to first node
         * @param s2_aux Pointer to second node
         */
        virtual void ComputeCost(Node *_s_aux, Node *_s2_aux) override;

        /**
         * @brief SetVertex function
         * Line of sight is checked inside this function
         * @param s_aux 
         */
        virtual void SetVertex(Node *_s_aux);

        // Variable to ensure that the los is true between the parent of the current and one neighbour, so SetVertex function should not be executed
        bool los_neighbour_{false};

                
    };

}

#endif
