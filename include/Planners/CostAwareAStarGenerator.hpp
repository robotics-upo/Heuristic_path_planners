#ifndef COSTAWAREASTARGENERATOR_HPP
#define COSTAWAREASTARGENERATOR_HPP
/**
 * @file CostAwareAStarGenerator.hpp
 * @author Rafael Rey (rreyarc@upo.es)
 * @brief 
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Planners/AStarGenerator.hpp>

namespace Planners{

    
    class CostAwareAStarGenerator : public AStarGenerator
    {
        
    public:
        /**
         * @brief Construct a new CostAwareAStarGenerator object
         * @param _use_3d This parameter allows the user to choose between planning on a plane (8 directions possibles) or in the 3D full space (26 directions)
         */
        CostAwareAStarGenerator(bool _use_3d, std::string _name );
        CostAwareAStarGenerator(bool _use_3d);

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
