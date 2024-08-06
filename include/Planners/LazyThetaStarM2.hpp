#ifndef LAZYTHETASTARSAFETYCOST_HPP
#define LAZYTHETASTARSAFETYCOST_HPP
/**
 * @file LazyThetaStarM2.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * @brief 
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Planners/ThetaStarM2.hpp>

namespace Planners
{
    /**
     * @brief Lazy Theta* Algorithm Class
     * 
     */
    class LazyThetaStarM2 : public ThetaStarM2
    {

    public:
        
        /**
         * @brief Construct a new Lazy Theta Star object
         * 
         * @param _use_3d This parameter allows the user to choose between planning 
         * on a plane (8 directions possibles) or in the 3D full space (26 directions)
         * @param _name Algorithm name stored internally
         * 
         */
        LazyThetaStarM2(bool _use_3d, std::string _name );

        /**
         * @brief Construct a new Lazy Theta Star Safety Cost object
         * 
         * @param _use_3d 
         */
        LazyThetaStarM2(bool _use_3d);

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

    };

}

#endif
