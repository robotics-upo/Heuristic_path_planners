#ifndef LAZYTHETASTAR_HPP
#define LAZYTHETASTAR_HPP
/**
 * @file LazyThetaStarGenerator.hpp
 * @author Rafael Rey (rreyarc@upo.es)
 * @brief 
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Planners/ThetaStarGenerator.hpp>

namespace Planners
{
    /**
     * @brief Lazy Theta* Algorithm Class
     * 
     */
    class LazyThetaStarGenerator : public ThetaStarGenerator
    {

    public:
        
        /**
         * @brief Construct a new Lazy Theta Star Generator object
         * 
         * @param _use_3d This parameter allows the user to choose between planning on a plane (8 directions possibles) or in the 3D full space (26 directions)
         */
        LazyThetaStarGenerator(bool _use_3d ):ThetaStarGenerator(_use_3d) {}

        /**
         * @brief Main function of the algorithm
         * 
         * @param source_ Start discrete coordinates
         * @param target_ Goal discrete coordinates
         * @return PathData PathData Results stored as PathData object
         */
        PathData findPath(const Vec3i &source_, const Vec3i &target_);

    protected:

        /**
         * @brief Compute cost function of the Lazy version of the algorithm
         * 
         * @param s_aux Pointer to first node
         * @param s2_aux Pointer to second node
         */
        void ComputeCost(Node *s_aux, Node *s2_aux);

        /**
         * @brief Update vertex function
         * 
         * @param s 
         * @param s2 
         * @param openset 
         */
        void UpdateVertex(Node *s, Node *s2, NodeSet &openset);

        /**
         * @brief SetVertex function
         * Line of sight is checked inside this function
         * @param s_aux 
         */
        void SetVertex(Node *s_aux);
                
    };

}

#endif
