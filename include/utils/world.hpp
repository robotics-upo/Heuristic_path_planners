#ifndef WORLD_HPP
#define WORLD_HPP

#include <vector>
#include <math.h>
#include "utils/utils.hpp"

namespace Planners{

namespace utils
{
    class Node;
    class Vec3i;
    class DiscreteWorld
    {

    public:
        DiscreteWorld()
        {
        }
        void resizeWorld(const Vec3i &_world_size, const double &_resolution){
            return resizeWorld(_world_size.x, _world_size.y, _world_size.z, _resolution);
        }
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
        bool setNodeCost(const double &_x, const double &_y, const double &_z, const unsigned int &_cost){
            
            return setNodeCost( Vec3i{ static_cast<int>(std::round(_x / resolution_)),
                                       static_cast<int>(std::round(_y / resolution_)),
                                       static_cast<int>(std::round(_z / resolution_))}, _cost);
        }
        bool setNodeCost(const Vec3i &_vec, const unsigned int &_cost){

            if(!checkValid(_vec))
                return false;
            
            auto i = getWorldIndex(_vec);
            discrete_world_vector_[i].cost = _cost;

        return true;
        }
        void resetWorld(){
            
            for(auto &it: discrete_world_vector_){
                it.isInClosedList = false;
                it.isInOpenList = false;
                it.H = it.G = 0;
                it.cost = 0;
                it.parent = nullptr;
            }
        }
        bool isOccupied(const int &_x, const int &_y, const int &_z)
        {
            if (!checkValid(_x, _y, _z))
                return true;

            if (discrete_world_vector_[getWorldIndex(_x, _y, _z)].occuppied)
            {
                return true;
            }

            return false;
        }
        bool isOccupied(const Vec3i &_node){
            return isOccupied(_node.x, _node.y, _node.z);
        }
        bool isOccupied(const Node &_node){
            return isOccupied(_node.coordinates.x, _node.coordinates.y, _node.coordinates.z);
        }

        void setOccupied(const int &_x, const int &_y, const int &_z)
        {

            if (!checkValid(_x, _y, _z))
                return;

            discrete_world_vector_[getWorldIndex(_x, _y, _z)].occuppied = true;
        }
        void setOccupied(const Vec3i &_pos){

            return setOccupied(_pos.x, _pos.y, _pos.z);
        }
        bool isInOpenList(const int &_x, const int &_y, const int &_z){
            if (!checkValid(_x, _y, _z))
                return false;

            if (discrete_world_vector_[getWorldIndex(_x, _y, _z)].isInOpenList)
                return true;

            return false;
        }
        bool isInOpenList(const Vec3i &_node){
            return isInOpenList(_node.x, _node.y, _node.z);
        }
        bool isInOpenList(const Node &_node){
            return isInOpenList(_node.coordinates.x, _node.coordinates.y, _node.coordinates.z);
        }
        bool isInClosedList(const int &_x, const int &_y, const int &_z){
            if (!checkValid(_x, _y, _z))
                return false;

            if (discrete_world_vector_[getWorldIndex(_x, _y, _z)].isInClosedList)
                return true;

            return false;
        }
        bool isInClosedList(const Vec3i &_node){
            return isInClosedList(_node.x, _node.y, _node.z);
        }
        bool isInClosedList(const Node &_node){
            return isInClosedList(_node.coordinates.x, _node.coordinates.y, _node.coordinates.z);
        }
        void setClosedValue(const int &_x, const int &_y, const int &_z, const bool value){
            if (!checkValid(_x, _y, _z))
                return;

            discrete_world_vector_[getWorldIndex(_x, _y, _z)].isInClosedList = value;
        }
        void setClosedValue(const Vec3i &_pos, const bool _value){
            if (!checkValid(_pos))
                return;

            discrete_world_vector_[getWorldIndex(_pos)].isInClosedList = _value;
        }
        void setOpenValue(const int &_x, const int &_y, const int &_z, const bool _value){
            if (!checkValid(_x, _y, _z))
                return;

            discrete_world_vector_[getWorldIndex(_x, _y, _z)].isInOpenList = _value;
        }
        void setOpenValue(const Vec3i &_pos, const bool _value){

            if(!checkValid(_pos))
                return;
            
            discrete_world_vector_[getWorldIndex(_pos)].isInOpenList = _value;
        }

        Node* getNodePtr(const Vec3i &_vec){

            if(!checkValid(_vec))
                return nullptr;

            return &discrete_world_vector_[getWorldIndex(_vec)];
        }
        const std::vector<Planners::utils::Node>& getElements() const{
            return discrete_world_vector_;
        }
        double getResolution() const{
            return resolution_;
        }
    private:
        bool checkValid(const Vec3i &_pos){

            return checkValid(_pos.x, _pos.y, _pos.z);
        }
        bool checkValid(const unsigned int &_x, 
                        const unsigned int &_y, 
                        const unsigned int &_z) {

            if ( _x > world_x_size_ ||
                 _y > world_y_size_ ||
                 _z > world_z_size_ )
                return false;

            return true;
        }

        unsigned int getWorldIndex(const Vec3i &_pos){

            return getWorldIndex(_pos.x, _pos.y, _pos.z);
        }
        unsigned int getWorldIndex(const int &x, const int &y, const int &z)
        {
            return (unsigned int)( z * world_x_size_ * world_y_size_ + y * world_x_size_ + x);
        }
        Vec3i getDiscreteWorldPositionFromIndex(const int _index){

            int z = std::floor(_index / (world_x_size_ * world_y_size_));
            int ind = _index - ( z * world_y_size_ * world_x_size_ );
            int y = std::floor(ind /world_x_size_);
            int x = std::floor(ind % world_x_size_);

            Vec3i vec{x, y, z};
            return vec;
        }

        std::vector<Planners::utils::Node> discrete_world_vector_;

        unsigned int world_x_size_, world_y_size_, world_z_size_;
        double resolution_;
    };

}
}

#endif