#ifndef ASTARM2_HPP
#define ASTARM2_HPP
/**
 * @file AStarM2.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * 
 * @brief This Algorithm is the same as the original A* with
 * the only difference in the ComputeG function that is re-implemented
 * It adds a edge-neighbour term to the total G value.
 * 
 * @version 0.1
 * @date 2021-09-21
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Planners/AStar.hpp>

namespace Planners{

    /**
     * @brief 
     * 
     */
    class AStarM2 : public AStar
    {
        
    public:
        /**
         * @brief Construct a new AStarM2 object
         * @param _use_3d This parameter allows the user to choose 
         * between planning on a plane (8 directions possibles) 
         * or in the 3D full space (26 directions)
         * 
         * @param _name Algorithm name stored internally
         */
        AStarM2(bool _use_3d, std::string _name );

        /**
         * @brief Construct a new AStarM2 object
         * 
         * @param _use_3d 
         */
        AStarM2(bool _use_3d);

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
        inline virtual unsigned int computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs, torch::jit::script::Module& loaded_sdf) override;

    };

}

#endif 
