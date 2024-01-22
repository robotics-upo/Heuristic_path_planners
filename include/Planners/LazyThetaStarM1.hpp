#ifndef LAZYTHETASTARM1_HPP
#define LAZYTHETASTARM1_HPP
/**
 * @file LazyThetaStarM1.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * @brief 
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Planners/LazyThetaStar.hpp>

namespace Planners
{
    /**
     * @brief Lazy Theta* Algorithm Class
     * 
     */
    class LazyThetaStarM1 : public LazyThetaStar
    {

    public:
        
        /**
         * @brief Construct a new Lazy Theta Star object
         * 
         * @param _use_3d This parameter allows the user to choose between planning on a plane 
         * (8 directions possibles) or in the 3D full space (26 directions)
         * @param _name Algorithm name stored internally
         * 
         */
        LazyThetaStarM1(bool _use_3d, std::string _name );

        /**
         * @brief Construct a new Cost Aware Lazy Theta Star object
         * 
         * @param _use_3d 
         */
        LazyThetaStarM1(bool _use_3d);

    protected:

        /**
         * @brief Compute cost function of the Lazy version of the algorithm
         * 
         * @param _s_aux Pointer to first node
         * @param _s2_aux Pointer to second node
         */
        inline virtual void ComputeCost(Node *_s_aux, Node *_s2_aux) override;

        /**
         * @brief SetVertex function
         * Line of sight is checked inside this function
         * @param _s_aux 
         */
        virtual void SetVertex(Node *_s_aux) override;


        /**
         * @brief This functions implements the algorithm G function. 
         * 
         * @param _current Pointer to the current node
         * @param _suc Pointer to the successor node
         * @param _n_i The index of the direction in the directions vector. 
         * Depending on this index, the distance wi
         * @param _dirs Number of directions used (to distinguish between 2D and 3D)
         * @return unsigned int The G Value calculated by the function
         */
        virtual unsigned int computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs) override;

                
    };

}

#endif
