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

    /**
     * @brief Theta* Algorithm Class
     * 
     */
    class ThetaStarGenerator : public AStarGenerator
    {

    public:
        
        /**
         * @brief Construct a new Theta Star Generator object
         * 
         * @param _use_3d This parameter allows the user to choose between planning on a plane (8 directions possibles) or in the 3D full space (26 directions)
         */
        ThetaStarGenerator(bool _use_3d, std::string _name );
        ThetaStarGenerator(bool _use_3d);

    protected:
        /**
         * @brief Update Vertex function
         * 
         * @param s Pointer to node s
         * @param s2 Pointer to node s2
         * @param openset reference to openset to erase and insert the nodes in some cases
         */
        virtual void UpdateVertex(Node *_s, Node *_s2, node_by_position &_index_by_pos, node_by_cost &_index_by_cost);
        
        /**
         * @brief Compute cost algorithm function
         * 
         * @param s_aux Pointer to first node
         * @param s2_aux Pointer to the second node
         */
        virtual void ComputeCost(Node *_s_aux, Node *_s2_aux);

        /**
         * @brief 
         * 
         * @param _current 
         * @param _target 
         * @param _index_by_pos 
         * @param _index_by_cost 
         */
        virtual void exploreNeighbours(Node* _current, const Vec3i &_target,node_by_position &_index_by_pos, node_by_cost &_index_by_cost) override;
    };

}

#endif
