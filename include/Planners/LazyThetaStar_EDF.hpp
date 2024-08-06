#ifndef LAZYTHETASTAREDF_HPP
#define LAZYTHETASTAREDF_HPP
/**
 * @file LazyThetaStar_EDF.hpp
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * 
 * @brief This header contains the Lazy Theta* algorithm
 * implementation. It inherits from the Theta* class
 * and reimplement the findPath and the ComputeCosts 
 * function and implement the new SetVertex function
 *  
 * @version 0.1
 * @date 2023-04-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <Planners/ThetaStar.hpp>

namespace Planners
{
    /**
     * @brief Lazy Theta* Algorithm Class considering the gradient and properties of the EDF described in IROS22
     * 
     */
    class LazyThetaStarEDF : public ThetaStar
    {

    public:
        
        /**
         * @brief Construct a new Lazy Theta Star  object
         * 
         * @param _use_3d This parameter allows the user to choose between planning on a plane (
         * 8 directions possibles) or in the 3D full space (26 directions)
         * @param _name Algorithm name stored internally
         * 
         */
        LazyThetaStarEDF(bool _use_3d, std::string _name );

        /**
         * @brief Construct a new Lazy Theta Star object
         * 
         * @param _use_3d 
         */
        LazyThetaStarEDF(bool _use_3d);

        /**
         * @brief Main function of the algorithm
         * 
         * @param _source Start discrete coordinates
         * @param _target Goal discrete coordinates
         * @return PathData PathData Results stored as PathData object
         */
        virtual PathData findPath(const Vec3i &_source, const Vec3i &_target) override;

    protected:

        /**
         * @brief Compute cost function of the Lazy Theta* algorithm
         * 
         * @param _s_aux Pointer to first node
         * @param _s2_aux Pointer to second node
         */
        virtual void ComputeCost(Node *_s_aux, Node *_s2_aux) override;

        /**
         * @brief SetVertex function
         * Line of sight is checked inside this function
         * @param _s_aux 
         */
        virtual void SetVertex(Node *_s_aux);

        inline virtual unsigned int computeG(const Node* _current, Node* _suc,  unsigned int _n_i, unsigned int _dirs) override;
                
    };

}

#endif
