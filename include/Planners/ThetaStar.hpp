#ifndef THETASTAR_HPP
#define THETASTAR_HPP
/**
 * @file ThetaStar.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * 
 * @brief This class inherit from the AStar Algorithm and 
 * implements the UpdateVertex and ComputeCost functions and
 * re-implements ExploreNeighbours method fromthe AStar class.
 * 
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Planners/AStar.hpp>

namespace Planners
{

    /**
     * @brief Theta* Algorithm Class
     * 
     */
    class ThetaStar : public AStar
    {

    public:
        
        /**
         * @brief Construct a new Theta Star  object
         * 
         * @param _use_3d This parameter allows the user to choose between 
         * planning on a plane (8 directions possibles) or in the 3D full space (26 directions)
         * @param _name Algorithm name stored internally
         */
        ThetaStar(bool _use_3d, std::string _name );

        /**
         * @brief Construct a new Theta Star object
         * 
         * @param _use_3d 
         */
        ThetaStar(bool _use_3d);

    protected:
        /**
         * @brief Update Vertex function
         * 
         * @param _s Pointer to node s
         * @param _s2 Pointer to node s2
         * @param _index_by_pos reference to openset to erase and insert the nodes in some cases
         */
        virtual void UpdateVertex(Node *_s, Node *_s2, node_by_position &_index_by_pos);
        
        /**
         * @brief Compute cost algorithm function
         * 
         * @param _s_aux Pointer to first node
         * @param _s2_aux Pointer to the second node
         */
        inline virtual void ComputeCost(Node *_s_aux, Node *_s2_aux);

        /**
         * @brief This function is the secondary inside the main loop of findPath 
         * function. This secondary loop iterates over the neighbours of a node, 
         * skipping the explored or occupied ones, and performs the appropiate operations
         * and calculus of the H and G values, and the corresponding parents updates.
         * 
         * @param _current A pointer to the current node, the loop will iterate over
         * the neighbours of this node by using the directions vector in the AlgorithmBase
         * class
         * 
         * @param _target A reference to the target coordinates of the GOAL.
         * 
         * @param _index_by_pos A reference to the openset to insert the non-explored nodes 
         * or to erase and insert the re-explored ones if a better path is found. 
         * This operation of erase and re-insert is performed in order to update the position
         * of the node in the container. 
         */
        virtual void exploreNeighbours(Node* _current, const Vec3i &_target,node_by_position &_index_by_pos, torch::jit::script::Module& loaded_sdf) override;

        utils::CoordinateListPtr checked_nodes, checked_nodes_current;  /*!< TODO Comment */

    };

}

#endif
