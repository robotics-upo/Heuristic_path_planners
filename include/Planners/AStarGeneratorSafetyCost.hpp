#ifndef ASTARGENERATORSAFETYCOST_HPP
#define ASTARGENERATORSAFETYCOST_HPP
/**
 * @file AStarGeneratorSafety.hpp
 * @author Rafael Rey (rreyarc@upo.es)
 * @brief 
 * @version 0.1
 * @date 2021-09-21
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Planners/AStarGenerator.hpp>

namespace Planners{

    
    class AStarGeneratorSafetyCost : public AStarGenerator
    {
        
    public:
        /**
         * @brief Construct a new CostAwareAStarGenerator object
         * @param _use_3d This parameter allows the user to choose between planning on a plane (8 directions possibles) or in the 3D full space (26 directions)
         */
        AStarGeneratorSafetyCost(bool _use_3d, std::string _name );
        AStarGeneratorSafetyCost(bool _use_3d);

    protected:
        
        /**
         * @brief 
         * 
         * @param _current 
         * @param _suc 
         * @param _n_i 
         * @param _dirs 
         * @return unsigned int 
         */
        virtual unsigned int computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs) override;

    };

}

#endif 
