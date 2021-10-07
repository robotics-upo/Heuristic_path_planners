#ifndef WORLD_HPP
#define WORLD_HPP
/**
 * @file world.hpp
 * @author Rafael Rey (rreyarc@upo.es)
 * @brief This header contains an implementation of a discrete cell-like world to use with the planning algorithms
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <vector>
#include <math.h>
#include "utils/utils.hpp"

namespace Planners{

namespace utils
{
    class Node;
    class Vec3i;

    /**
     * @brief Class implementing the data structure storing the map related data that algorithms uses internally
     * 
     */
    class DiscreteWorld
    {

    public:
        /**
         * @brief Construct a new Discrete World object
         * 
         */
        DiscreteWorld()
        {
        }
        /**
         * @brief Overloaded resizeWorld function for Vec3i objects
         * 
         * @param _world_size Vec3i object with world size data
         * @param _resolution resolution to create the internal world vector
         */
        void resizeWorld(const Vec3i &_world_size, const double &_resolution){
            return resizeWorld(_world_size.x, _world_size.y, _world_size.z, _resolution);
        }
        /**
         * @brief It configures the inner world vector. Internally the world coordinates 
         *  goes from [0, world_x_size], [0, world_y_size], [0, world_z_size]
         *  It clears the previous world and create a new one. 
         * @param _world_x_size 
         * @param _world_y_size 
         * @param _world_z_size 
         * @param _resolution 
         */
        void resizeWorld(const unsigned int &_world_x_size,
                         const unsigned int &_world_y_size,
                         const unsigned int &_world_z_size,
                         const double &_resolution)
        {
            world_x_size_ = _world_x_size;
            world_y_size_ = _world_y_size;
            world_z_size_ = _world_z_size;
            resolution_   = _resolution;

            discrete_world_vector_.clear();
            discrete_world_vector_.resize(world_x_size_ * world_y_size_ * _world_z_size);
            Node node;
            std::fill(discrete_world_vector_.begin(), discrete_world_vector_.end(), node);
            
            for(long unsigned int i = 0; i < discrete_world_vector_.size(); ++i){
                discrete_world_vector_[i].coordinates =  getDiscreteWorldPositionFromIndex(i);
            }
            
        }
        /**
         * @brief Set the Node Cost object overloaded function for continous coordinates
         * 
         * @param _x continous coordinate
         * @param _y continous coordinate
         * @param _z continous coordinate
         * @param _cost 
         * @return true if node is valid
         * @return false if requested coordinates correspond to invalid node (outside the world)
         */
        bool setNodeCost(const double &_x, const double &_y, const double &_z, const unsigned int _cost){
            
            return setNodeCost( Vec3i{ static_cast<int>(std::round(_x / resolution_)),
                                       static_cast<int>(std::round(_y / resolution_)),
                                       static_cast<int>(std::round(_z / resolution_))}, _cost);
        }
        /**
         * @brief Set the Node Cost object 
         * 
         * @param _vec coordinates of the node
         * @param _cost cost value assigned to it
         * @return true if node is valid
         * @return false if requested coordinates correspond to invalid node (outside the world)
         */
        bool setNodeCost(const Vec3i &_vec, const unsigned int _cost){

            if(!checkValid(_vec))
                return false;
            
            discrete_world_vector_[getWorldIndex(_vec)].cost = _cost;

        return true;
        }
        /**
         * @brief Set to its default state the flags, cost values, and parent values inside 
         * each world's node
         */
        void resetWorld(){
            
            for(auto &it: discrete_world_vector_){
                it.isInClosedList = false;
                it.isInOpenList = false;
                it.H = it.G = it.C = 0;
                it.parent = nullptr;
            }
        }
        /**
         * @brief Function to check is the node is valid 
         * 
         * @param _x discrete coordinate
         * @param _y discrete coordinate
         * @param _z discrete coordinate
         * @return true if node valid and not occupied
         * @return false if node is outside the workspace of is marked as occupied
         */
        bool isOccupied(const int _x, const int _y, const int _z) const
        {
            if (!checkValid(_x, _y, _z))
                return true;

            if (discrete_world_vector_[getWorldIndex(_x, _y, _z)].occuppied)
            {
                return true;
            }

            return false;
        }
        /**
         * @brief Overloaded isOccupied function for Vec3i objects
         * 
         * @param _coord discrete coordinates vector
         * @return true if node valid and not occupied 
         * @return false if node is outside the workspace of is marked as occupied 
         */
        bool isOccupied(const Vec3i &_coord) const{
            return isOccupied(_coord.x, _coord.y, _coord.z);
        }
        /**
         * @brief Overloaded isOccupied function for Node objects
         * 
         * @param _node node object
         * @return true if node valid and not occupied 
         * @return false if node is outside the workspace of is marked as occupied 
         */
        bool isOccupied(const Node &_node) const{
            return isOccupied(_node.coordinates.x, _node.coordinates.y, _node.coordinates.z);
        }
        /**
         * @brief Set the world's node associated
         * to these coordinates as occupied. 
         * @param _x discrete coordinates
         * @param _y discrete coordinates
         * @param _z discrete coordinates
         */
        void setOccupied(const int _x, const int _y, const int _z)
        {

            if (!checkValid(_x, _y, _z))
                return;

            discrete_world_vector_[getWorldIndex(_x, _y, _z)].occuppied = true;
        }
        /**
         * @brief Set the world's node associated
         * 
         * @param _pos discrete node position vector
         */
        void setOccupied(const Vec3i &_pos){

            return setOccupied(_pos.x, _pos.y, _pos.z);
        }
        /**
         * @brief Checks the value of the internal flag of the node
         * that is used to mark that the node is in the open list
         * @param _x discrete coordinates
         * @param _y discrete coordinates
         * @param _z discrete coordinates
         * @return true if the node is valid and is in the open list
         * @return false if the node is not valid (outside workspace) or not in the open list
         */
        bool isInOpenList(const int _x, const int _y, const int _z){
            if (!checkValid(_x, _y, _z))
                return false;

            if (discrete_world_vector_[getWorldIndex(_x, _y, _z)].isInOpenList)
                return true;

            return false;
        }
        /**
         * @brief Overloaded function for Vec3i objects
         * 
         * @param _coord Discrete coordinates
         * @return true if the node is valid and is in the open list 
         * @return false if the node is not valid (outside workspace) or not in the open list 
         */
        bool isInOpenList(const Vec3i &_coord){
            return isInOpenList(_coord.x, _coord.y, _coord.z);
        }
        /**
         * @brief Overloaded function for Node objects
         * 
         * @param _node Node object 
         * @return true if the node is valid and is in the open list 
         * @return false if the node is not valid (outside workspace) or not in the open list 
         */
        bool isInOpenList(const Node &_node){
            return isInOpenList(_node.coordinates.x, _node.coordinates.y, _node.coordinates.z);
        }
        /**
         * @brief Analogous to isInOpenList
         * 
         * @param _x discrete coordinates
         * @param _y discrete coordinates
         * @param _z discrete coordinates
         * @return true if node is valid and it is in the closed list
         * @return false if node is not in the closed list or it is not valid
         */
        bool isInClosedList(const int _x, const int _y, const int _z){
            if (!checkValid(_x, _y, _z))
                return false;

            if (discrete_world_vector_[getWorldIndex(_x, _y, _z)].isInClosedList)
                return true;

            return false;
        }
        /**
         * @brief Overloaded function for Vec3i objects
         * 
         * @param _node 
         * @return true if node is valid and it is in the closed list
         * @return false if node is not in the closed list or it is not valid
         */
        bool isInClosedList(const Vec3i &_node){
            return isInClosedList(_node.x, _node.y, _node.z);
        }
        /**
         * @brief Overloaded function for Node objects
         * 
         * @param _node 
         * @return true if node is valid and it is in the closed list
         * @return false if node is not in the closed list or it is not valid
         */
        bool isInClosedList(const Node &_node){
            return isInClosedList(_node.coordinates.x, _node.coordinates.y, _node.coordinates.z);
        }
        /**
         * @brief Set the is in closed list internal flag of the node associated to the 
         * discrete coordinates 
         * @param _x discrete_coordinated
         * @param _y discrete_coordinated
         * @param _z discrete_coordinated
         * @param value desired value
         */
        void setClosedValue(const int _x, const int _y, const int _z, const bool _value){
            if (!checkValid(_x, _y, _z))
                return;

            discrete_world_vector_[getWorldIndex(_x, _y, _z)].isInClosedList = _value;
        }
        /**
         * @brief Set the Closed Value object overloaded function for Vec3i
         * 
         * @param _pos discrete position of the node
         * @param _value the desired value 
         */
        void setClosedValue(const Vec3i &_pos, const bool _value){
            if (!checkValid(_pos))
                return;

            discrete_world_vector_[getWorldIndex(_pos)].isInClosedList = _value;
        }
        /**
         * @brief Set the Closed Value object
         * 
         * @param _node 
         * @param _value 
         */
        void setClosedValue(const Node &_node, const bool _value){
            setClosedValue(_node.coordinates,_value);
        }
        /**
         * @brief SSet the is in open list internal flag of the node associated to 
         * the discrete coordinates
         * 
         * @param _x discrete coordinates
         * @param _y discrete coordinates
         * @param _z discrete coordinates
         * @param _value desired value
         */
        void setOpenValue(const int _x, const int _y, const int _z, const bool _value){
            if (!checkValid(_x, _y, _z))
                return;

            discrete_world_vector_[getWorldIndex(_x, _y, _z)].isInOpenList = _value;
        }
        /**
         * @brief Overloaded function for Vec3i. Set the is in open list internal flag of the node associated to 
         * the discrete coordinates
         * 
         * @param _pos discrete coordinates
         * @param _value desired value
         */
        void setOpenValue(const Vec3i &_pos, const bool _value){

            if(!checkValid(_pos))
                return;
            
            discrete_world_vector_[getWorldIndex(_pos)].isInOpenList = _value;
        }
        /**
         * @brief Set the Open Value object
         * 
         * @param _node 
         * @param _value 
         */
        void setOpenValue(const Node &_node, const bool _value){
            
            setOpenValue(_node.coordinates, _value);
        }
        /**
         * @brief Get the pointer to the node corresponding to a discrete set of coordinates
         * 
         * @param _vec discrete coordinates
         * @return Pointer to the corresponding node* object or nullptr if requested set of coordinates are not valid
         */
        Node* getNodePtr(const Vec3i &_vec){

            if(!checkValid(_vec))
                return nullptr;

            return &discrete_world_vector_[getWorldIndex(_vec)];
        }
        /**
         * @brief get the inner world object
         * 
         * @return const std::vector<Planners::utils::Node>& Reference to the world object stored inside
         */
        const std::vector<Planners::utils::Node>& getElements() const{
            return discrete_world_vector_;
        }
        /**
         * @brief Get the Resolution stored inside
         * 
         * @return double resolution used internally
         */
        double getResolution() const{
            return resolution_;
        }
        /**
         * @brief Get the World Size internal object (discrete)
         * 
         * @return Vec3i 
         */
        Vec3i getWorldSize() const{
            return { static_cast<int>(world_x_size_),
                     static_cast<int>(world_y_size_), 
                     static_cast<int>(world_z_size_)};
        }
    private:
        /**
         * @brief checkValid overloaded function for Vec3i objects
         * 
         * @param _pos discrete position object
         * @return true if position inside the workspace 
         * @return false if any of the coordinates is bigger than the associated world size dimension
         */
        bool checkValid(const Vec3i &_pos) const{

            return checkValid(_pos.x, _pos.y, _pos.z);
        }
        /**
         * @brief 
         * 
         * @param _x 
         * @param _y 
         * @param _z 
         * @return true if position inside the workspace 
         * @return false if any of the coordinates is bigger than the associated world size dimension
         */
        bool checkValid(const unsigned int _x, 
                        const unsigned int _y, 
                        const unsigned int _z) const{

            if ( _x >= world_x_size_ ||
                 _y >= world_y_size_ ||
                 _z >= world_z_size_ )
                return false;

            return true;
        }
        /**
         * @brief getWorldIndex overloaded function for Vec3i coordinates
         * 
         * @param _pos discrete position 
         * @return unsigned int world index associated to the requested discrete position
         */
        unsigned int getWorldIndex(const Vec3i &_pos) const{

            return getWorldIndex(_pos.x, _pos.y, _pos.z);
        }
        /**
         * @brief Get the world index associated to a set of discrete coordinates
         * 
         * @param x discrete coordinates
         * @param y discrete coordinates
         * @param z discrete coordinates
         * @return unsigned int world index of the vector
         */
        unsigned int getWorldIndex(const int _x, const int _y, const int _z) const
        {
            return (unsigned int)( _z * world_x_size_ * world_y_size_ + _y * world_x_size_ + _x);
        }
        /**
         * @brief Get the Discrete World Position From Index object
         * 
         * @param _index Discrete index of the internal vector
         * @return Vec3i discrete coordinates vector
         */
        Vec3i getDiscreteWorldPositionFromIndex(const int _index){

            int z = std::floor(_index / (world_x_size_ * world_y_size_));
            int ind = _index - ( z * world_y_size_ * world_x_size_ );
            int y = std::floor(ind /world_x_size_);
            int x = std::floor(ind % world_x_size_);

            return {x, y, z};
        }

        std::vector<Planners::utils::Node> discrete_world_vector_;

        unsigned int world_x_size_, world_y_size_, world_z_size_;
        double resolution_;
    };

}
}

#endif