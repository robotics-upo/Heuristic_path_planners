#ifndef LAZYTHETASTARSEMANTIC_HPP
#define LAZYTHETASTARSEMANTIC_HPP
/**
 * @file LazyThetaStarSemantic.hpp
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * @brief 
 * @version 0.1
 * @date 2024-01-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <Planners/ThetaStarSemantic.hpp>

namespace Planners
{
    /**
     * @brief Lazy Theta* Algorithm Class
     * 
     */
    class LazyThetaStarSemantic : public ThetaStarSemantic
    {

    public:
        
        /**
         * @brief Construct a new Semantic Lazy Theta Star object
         * 
         * @param _use_3d This parameter allows the user to choose between planning on a plane 
         * (8 directions possibles) or in the 3D full space (26 directions)
         * @param _name Algorithm name stored internally
         * 
         */
        LazyThetaStarSemantic(bool _use_3d, std::string _name );

        /**
         * @brief Construct a new Semantic Lazy Theta Star object
         * 
         * @param _use_3d 
         */
        LazyThetaStarSemantic(bool _use_3d);

        /**
         * @brief 
         * 
         * @param _source 
         * @param _target 
         * @return PathData 
         */
        virtual PathData findPath(const Vec3i &_source, const Vec3i &_target) override;

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
        virtual void SetVertex(Node *_s_aux);

        /**
         * @brief 
         * 
         * @param _current 
         * @param _suc 
         * @param _n_i 
         * @param _dirs 
         * @return unsigned int 
         */
        inline virtual unsigned int computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs) override;

        // Variable to ensure that the los is true between the parent of the current and one neighbour, so SetVertex function should not be executed
        bool los_neighbour_{false}; /*!< TODO Comment */
        int coef=3;

                
    };

}

#endif
