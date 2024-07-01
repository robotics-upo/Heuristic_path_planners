#ifndef THETASTARSIREN_HPP
#define THETASTARSIREN_HPP
/**
 * @file ThetaStarSIREN.hpp
 * @author Guillermo Gil (guillermogilg99@gmail.com)
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * 
 * @brief TBD
 * 
 * @version 0.1
 * @date 2024-06-27
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <Planners/ThetaStar.hpp>

namespace Planners
{

    /**
     * @brief Theta* SIREN Algorithm Class
     * 
     */
    class ThetaStarSIREN : public ThetaStar
    {

    public:
        
        /**
         * @brief Construct a new Theta Star SIREN object
         * 
         * @param _use_3d This parameter allows the user to choose between planning on 
         * a plane (8 directions possibles) or in the 3D full space (26 directions)
         * @param _name Algorithm name stored internally
         * 
         */
        ThetaStarSIREN(bool _use_3d, std::string _name );

        /**
         * @brief Construct a new Theta Star SIREN object
         * 
         * @param _use_3d 
         */
        ThetaStarSIREN(bool _use_3d);


    protected:

        /**
         * @brief Compute cost algorithm function
         * 
         * @param _s_aux Pointer to first node
         * @param _s2_aux Pointer to the second node
         */
        inline virtual void ComputeCost(Node *_s_aux, Node *_s2_aux, torch::jit::script::Module& loaded_sdf) override;


        /**
         * @brief TBD
         * 
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
