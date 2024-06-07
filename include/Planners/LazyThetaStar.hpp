#ifndef LAZYTHETASTAR_HPP
#define LAZYTHETASTAR_HPP
/**
 * @file LazyThetaStar.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * 
 * @brief This header contains the Lazy Theta* algorithm
 * implementation. It inherits from the Theta* class
 * and reimplement the findPath and the ComputeCosts 
 * function and implement the new SetVertex function
 *  
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Planners/ThetaStar.hpp>

namespace Planners
{
    /**
     * @brief Lazy Theta* Algorithm Class
     * 
     */
    class LazyThetaStar : public ThetaStar
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
        LazyThetaStar(bool _use_3d, std::string _name );

        /**
         * @brief Construct a new Lazy Theta Star object
         * 
         * @param _use_3d 
         */
        LazyThetaStar(bool _use_3d);

        /**
         * @brief Main function of the algorithm
         * 
         * @param _source Start discrete coordinates
         * @param _target Goal discrete coordinates
         * @return PathData PathData Results stored as PathData object
         */
        virtual PathData findPath(const Vec3i &_source, const Vec3i &_target, HIOSDFNet& sdf_net) override;

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
                
    };

}

#endif
