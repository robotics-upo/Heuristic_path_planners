#ifndef THETASTARSEMANTICCOST_HPP
#define THETASTARSEMANTICCOST_HPP
/**
 * @file ThetaStarSemantic.hpp
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * 
 * @brief This header declares the functions and class 
 * associated to the Semantic Theta* Algorithm. It inherits
 * from the original Theta* algorithm and override two functions:
 *  1. ComputeCost  
 *  2. ComputeG
 * 
 * @version 0.1
 * @date 2024-01-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Planners/ThetaStar.hpp>

namespace Planners
{

    /**
     * @brief Theta* Algorithm Class
     * 
     */
    class ThetaStarSemanticCost : public ThetaStar
    {

    public:
        
        /**
         * @brief Construct a new Semantic Theta Star object
         * 
         * @param _use_3d This parameter allows the user to choose between planning on 
         * a plane (8 directions possibles) or in the 3D full space (26 directions)
         * @param _name Algorithm name stored internally
         * 
         */
        ThetaStarSemanticCost(bool _use_3d, std::string _name );

        /**
         * @brief Construct a new Semantic Theta Star object
         * 
         * @param _use_3d 
         */
        ThetaStarSemanticCost(bool _use_3d);


    protected:

        /**
         * @brief Compute cost algorithm function
         * 
         * @param _s_aux Pointer to first node
         * @param _s2_aux Pointer to the second node
         */
        inline virtual void ComputeCost(Node *_s_aux, Node *_s2_aux) override;

         /**
         * @brief Compute edge distance
         * 
         * @param _checked_nodes 
         * @param _s 
         * @param _s2 
         * @return unsigned int 
         */
        virtual unsigned int ComputeSemanticEdgeCost(const utils::CoordinateListPtr _checked_nodes, const Node* _s, const Node* _s2);


        /**
         * @brief This functions implements the algorithm G function.  The difference
         * with the original A* function is that the returned G value here is composed of three terms:
         * 1. Dist between the current and successor (1, sqrt(2) or sqrt(3))
         * 2. The G value of the current node (How does it cost to reach to the current)
         * 3. The edge_neighbour cost which is calculated as the averaged between the _current and _suc cost 
         * scaled with the value dist_scale_factor_reduced_
         * 
         *
         * @param _current Pointer to the current node
         * @param _suc Pointer to the successor node
         * @param _n_i The index of the direction in the directions vector. 
         * Depending on this index, the distance wi
         * @param _dirs Number of directions used (to distinguish between 2D and 3D)
         * @return unsigned int The G Value calculated by the function
         */
        virtual unsigned int computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs) override;
        int coef=10;

    };

}

#endif
