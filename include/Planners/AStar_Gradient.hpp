#ifndef ASTARGRADIENT_HPP
#define ASTAREDFGRADIENT_HPP
/**
 * @file AStarGRADIENT.hpp
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * 
 * @brief This algorithm is a variation of the A*. The only
 * difference is that it reimplements the exploreNeighbours method adding the
 * computation of the gradient to choose the explored neighbours:
 * 
 * @version 0.1
 * @date 2023-04-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <Planners/AStar.hpp>

namespace Planners{

    /**
     * @brief 
     * 
     */
    class AStarGradient : public AStar
    {
        
    public:
        /**
         * @brief Construct a new AStarM1 object
         * @param _use_3d This parameter allows the user to choose between 
         * planning on a plane (8 directions possibles) 
         * or in the 3D full space (26 directions)
         * @param _name Algorithm name stored internally
         * 
         */
        AStarGradient(bool _use_3d, std::string _name );

        /**
         * @brief Construct a new Cost Aware A Star M1 object
         * 
         * @param _use_3d 
         */
        AStarGradient(bool _use_3d);
        /**
         * @brief Main function of the algorithm
         * 
         * @param _source Start discrete coordinates. It should be a valid coordinates, i.e. it should not
         * be marked as occupied and it should be inside the configured workspace.
         * @param _target Goal discrete coordinates. It should be a valid coordinates, i.e. it should not
         * be marked as occupied and it should be inside the configured workspace.
         * @return PathData PathData Results stored as PathData object
         * Reminder: 
         * PathData    = std::map<std::string, Planners::utils::DataVariant> 
         * DataVariant = std::variant<std::string, Vec3i, CoordinateList, double, size_t, int, bool, unsigned int>;
         * TODO: Replace map here by unordered_map. Not much important, but it does not make sense to use a map.
         */
        PathData findPath(const Vec3i &_source, const Vec3i &_target) override;

    protected:
        
        /**
         * @brief Overrided ComputeG function. 
         * 
         * @param _current Pointer to the current node
         * @param _suc Pointer to the successor node
         * @param _n_i The index of the direction in the directions vector. 
         * Depending on this index, the distance wi
         * @param _dirs Number of directions used (to distinguish between 2D and 3D)
         * @return unsigned int The G Value calculated by the function
         */
        inline virtual unsigned int computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs) override;
        
        virtual void exploreNeighbours_Gradient(Node* _current, const Vec3i &_target,node_by_position &_index_by_pos);

        virtual int chooseNeighbours(float angh, float angv);
        // virtual Vec3i chooseNeighbours(float angh, float angv);

        virtual void nodesToExplore(int node);

    };

}

#endif 
