#ifndef THETASTARSAFETY_HPP
#define THETASTARSAFETY_HPP
/**
 * @file ThetaStarGeneratorSafetyCost.hpp
 * @author Rafael Rey (rreyarc@upo.es)
 * @brief 
 * @version 0.1
 * @date 2021-09-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Planners/ThetaStarGenerator.hpp>

namespace Planners
{

    /**
     * @brief Theta* Algorithm Class
     * 
     */
    class ThetaStarGeneratorSafetyCost : public ThetaStarGenerator
    {

    public:
        
        /**
         * @brief Construct a new Theta Star Generator object
         * 
         * @param _use_3d This parameter allows the user to choose between planning on a plane (8 directions possibles) or in the 3D full space (26 directions)
         */
        ThetaStarGeneratorSafetyCost(bool _use_3d, std::string _name );
        ThetaStarGeneratorSafetyCost(bool _use_3d);

    protected:

        /**
         * @brief Compute cost algorithm function
         * 
         * @param s_aux Pointer to first node
         * @param s2_aux Pointer to the second node
         */
        virtual void ComputeCost(Node *_s_aux, Node *_s2_aux) override;

        /**
         * @brief Compute edge distance
         * 
         * @param _checked_nodes 
         * @param _s 
         * @param _s2 
         * @param _dist Distance between _s and _s2
         * @return unsigned int 
         */
        virtual unsigned int ComputeEdgeCost(const utils::CoordinateListPtr _checked_nodes, const Node* _s, const Node* _s2, unsigned int _dist);

        /**
         * @brief 
         * 
         * @param _current 
         * @param _suc 
         * @param _n_i 
         * @param _dirs 
         * @return unsigned int 
         */
        virtual unsigned int computeG(const Node* _current, const Node* _suc,  unsigned int _n_i, unsigned int _dirs) override;

    
    };

}

#endif
