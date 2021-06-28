#ifndef THETASTAR_HPP
#define THETASTAR_HPP

#include <Planners/AStarGenerator.hpp>

namespace Planners
{

    class ThetaStarGenerator : public AStarGenerator
    {

    public:
        
        /**
         * @brief Construct a new Theta Star Generator object
         * 
         */
        ThetaStarGenerator(bool _use_3d ):AStarGenerator(_use_3d) {}

        /**
         * @brief 
         * 
         * @param source_ 
         * @param target_ 
         * @return PathData 
         */
        PathData findPath(const Vec3i &source_, const Vec3i &target_);

    protected:
        /**
         * @brief 
         * 
         * @param s 
         * @param s2 
         * @param openset 
         */
        void UpdateVertex(Node *s, Node *s2, NodeSet &openset);
        
        /**
         * @brief 
         * 
         * @param s_aux 
         * @param s2_aux 
         */
        void ComputeCost(Node *s_aux, Node *s2_aux);
    };

}

#endif
