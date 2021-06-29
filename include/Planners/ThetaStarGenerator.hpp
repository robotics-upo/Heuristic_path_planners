#ifndef THETASTAR_HPP
#define THETASTAR_HPP
/**
 * @file ThetaStarGenerator.hpp
 * @author Rafael Rey (rreyarc@upo.es)
 * @brief 
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Planners/AStarGenerator.hpp>

namespace Planners
{

    class ThetaStarGenerator : public AStarGenerator
    {

    public:
        
        /**
         * @brief Construct a new Theta Star Generator object
         * 
         * @param _use_3d This parameter allows the user to choose between planning on a plane (8 directions possibles) or in the 3D full space (26 directions)
         */
        ThetaStarGenerator(bool _use_3d ):AStarGenerator(_use_3d) {}

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
         * @brief Update Vertex function
         * 
         * @param s Pointer to node s
         * @param s2 Pointer to node s2
         * @param openset reference to openset to erase and insert the nodes in some cases
         */
        void UpdateVertex(Node *s, Node *s2, NodeSet &openset);
        
        /**
         * @brief Compute cost algorithm function
         * 
         * @param s_aux Pointer to first node
         * @param s2_aux Pointer to the second node
         */
        void ComputeCost(Node *s_aux, Node *s2_aux);
    };

}

#endif
