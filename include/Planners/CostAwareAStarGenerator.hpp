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

        /**
         * @brief Main function of the algorithm
         * 
         * @param _source Start discrete coordinates
         * @param _target Goal discrete coordinates
         * @return PathData PathData Results stored as PathData object
         */
        virtual PathData findPath(const Vec3i &_source, const Vec3i &_target) override;

    };

}

#endif 
