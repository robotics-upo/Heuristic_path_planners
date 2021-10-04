#ifndef COSTAWARETHETASTAR_HPP
#define COSTAWARETHETASTAR_HPP
/**
 * @file CostAwareThetaStarGenerator.hpp
 * @author Rafael Rey (rreyarc@upo.es)
 * @brief 
 * @version 0.1
 * @date 2021-09-20
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
    class CostAwareThetaStarGenerator : public ThetaStarGenerator
    {

    public:
        
        /**
         * @brief Construct a new Theta Star Generator object
         * 
         * @param _use_3d This parameter allows the user to choose between planning on a plane (8 directions possibles) or in the 3D full space (26 directions)
         */
        CostAwareThetaStarGenerator(bool _use_3d, std::string _name );
        CostAwareThetaStarGenerator(bool _use_3d);


    protected:

        /**
         * @brief Compute cost algorithm function
         * 
         * @param s_aux Pointer to first node
         * @param s2_aux Pointer to the second node
         */
        virtual void ComputeCost(Node *_s_aux, Node *_s2_aux) override;

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
