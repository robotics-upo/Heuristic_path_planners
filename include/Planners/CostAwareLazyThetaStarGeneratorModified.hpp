#ifndef COSTAWARELAZYTHETASTARMODIFIED_HPP
#define COSTAWARELAZYTHETASTARMODIFIED_HPP
/**
 * @file CostAwareLazyThetaStarGeneratorModified.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * @brief 
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Planners/CostAwareLazyThetaStarGenerator.hpp>

namespace Planners
{
    /**
     * @brief Lazy Theta* Algorithm Class
     * 
     */
    class CostAwareLazyThetaStarGeneratorModified : public CostAwareLazyThetaStarGenerator
    {

    public:
        
        /**
         * @brief Construct a new Lazy Theta Star Generator object
         * 
         * @param _use_3d This parameter allows the user to choose between planning on a plane 
         * (8 directions possibles) or in the 3D full space (26 directions)
         * @param _name Algorithm name stored internally
         * 
         */
        CostAwareLazyThetaStarGeneratorModified(bool _use_3d, std::string _name );

        /**
         * @brief Construct a new Cost Aware Lazy Theta Star Generator Modified object
         * 
         * @param _use_3d 
         */
        CostAwareLazyThetaStarGeneratorModified(bool _use_3d);

    protected:

        /**
         * @brief Compute cost function of the Lazy version of the algorithm
         * 
         * @param _s_aux Pointer to first node
         * @param _s2_aux Pointer to second node
         */
        inline virtual void ComputeCost(Node *_s_aux, Node *_s2_aux) override;

    };

}

#endif
