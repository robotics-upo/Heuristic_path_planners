#ifndef UTILS_HPP
#define UTILS_HPP
/**
 * @file utils.hpp
 * @author Rafael Rey (rreyarc@upo.es)
 * @brief A set of utils used alongside the project 
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <variant>
#include <math.h>
#include <memory>
#include <Eigen/Dense>

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/key_extractors.hpp>

using namespace ::boost;
using namespace ::boost::multi_index;

namespace Planners
{
    namespace utils
    {
        class Vec3i;
        class Node;
        struct NodeComparator;

        using CoordinateList     = std::vector<Planners::utils::Vec3i>;
        using CoordinateListPtr  = std::shared_ptr<std::vector<Planners::utils::Vec3i>>;
        using NodeSet            = std::set<Node*, NodeComparator>;
        using DataVariant        = std::variant<std::string, Vec3i, CoordinateList, double, size_t, int, bool, unsigned int>;
        using PathData           = std::map<std::string, DataVariant>;
        
        struct IndexByCost {};
        struct IndexByWorldPosition {};


        //Compile time constants
        static constexpr int const dist_scale_factor_{100};
        //To use with costs
        static constexpr int const dist_scale_factor_reduced_{ dist_scale_factor_ / 100 };
        //Dont touch these ones (diagonal distances in 2D and 3D)
        //The static cast from floating point to integer returns the truncated value 
        //i.e discards the decimal part
        static constexpr int const dd_2D_{static_cast<int>( dist_scale_factor_ * 1.41421356237 )}; //sqrt(2)
        static constexpr int const dd_3D_{static_cast<int>( dist_scale_factor_ * 1.73205080757 )}; //sqrt(3)

        /**
         * @brief 
         * 
         */
        struct gridCell
	    {
	    	float dist{0};
	    	float prob{0};
	    };
        /**
         * @brief 
         * 
         */
        struct gridData
        {
            unsigned int grid_size{0};
            unsigned int grid_size_x{0};
            unsigned int grid_size_y{0};
            unsigned int grid_size_z{0};
            unsigned int sensor_dev{0};
        };
        /**
         * @brief Overload ofstream operator << for std::vector<<CoordinateList>
         * 
         * @tparam T 
         * @param os 
         * @param v 
         * @return std::ostream& 
         */
        template <class T>
        inline std::ostream& operator << (std::ostream& os, const std::vector<T>& v) 
        {

            for (typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end()-1; ++ii)
                os << *ii << ", ";
            
            os << *(v.end()-1);

            return os;
        }

        /**
         * @brief This is one of the main classes used internally by the algorithms. 
         * It only stores a set of 3D discrete coordinates values 
         * and has some operator overloaded implementations to easily work with objects
         */
        class Vec3i
        {
            public:
            int x, y, z;    
            /**
             * @brief 
             * 
             * @param coordinates_ 
             * @return true 
             * @return false 
             */
            bool operator==(const Vec3i &coordinates_);
            /**
             * @brief 
             * 
             * @param coordinates_ 
             * @return true if all of the coordinates are greather or equal than the others
             * @return false if any of the coordinates are not greather or equal than the others
             */
            bool operator>=(const Vec3i &coordinates_);
            /**
             * @brief 
             * 
             * @param coordinates_ 
             * @return true if all the coordinates are <= of the corresponding ones
             * @return false if any of the coordinates are not <=
             */
            bool operator<=(const Vec3i &coordinates_);
            
            /**
             * @brief 
             * 
             * @param _divid 
             * @return Planners::utils::Vec3i& 
             */
            Planners::utils::Vec3i &operator/=(float _divid);

            /**
             * @brief 
             * 
             * @param right_ 
             * @return Vec3i 
             */
            Vec3i operator+(const Vec3i &right_) const
            {
                return {this->x + right_.x, this->y + right_.y, this->z + right_.z};
            }

            /**
             * @brief 
             * 
             * @param right_ 
             * @return Vec3i 
             */
            Vec3i operator-(const Vec3i &right_) const
            {
                return {this->x - right_.x, this->y - right_.y, this->z - right_.z};
            }

            /**
             * @brief 
             * 
             * @param _mult 
             * @return Vec3i 
             */
            inline Vec3i operator*(int &_mult) const
            {
                return {this->x * _mult, this->y * _mult, this->z * _mult };
            }

            Eigen::Vector3d toEigen() const{
                return Eigen::Vector3d(x, y, y);
            }

        };
        /**
         * @brief 
         * 
         * @param _mult 
         * @param _vec 
         * @return Planners::utils::Vec3i 
         */
        inline Planners::utils::Vec3i operator*(const int &_mult, const Planners::utils::Vec3i &_vec){
                    
            return { _vec.x * _mult, _vec.y * _mult, _vec.z * _mult};
        }
        /**
         * @brief 
         * 
         * @param os 
         * @param dt 
         * @return std::ostream& 
         */
        inline std::ostream& operator<<(std::ostream& os, const Vec3i& dt)
        {
            os << "[" << dt.x << ", " << dt.y << ", " << dt.z << "]";
            return os;
        }
        /**
         * @brief This object store the main information used by the algorithms 
         * 
         */
        class Node
        {
            public:
            Node *parent{nullptr};

            Planners::utils::Vec3i coordinates;
            // unsigned int G{0}, H{0}, C{0};
            // unsigned int cost{0}, non_uni{0};
            //JAC
            unsigned int G{0}, H{0};
            unsigned int non_uni{0}; //This is unused currently
            double cost{0};
            double C{0};
            unsigned int gplush{0};
            unsigned int world_index{0};

            bool occuppied{false};
            bool isInOpenList{false};
            bool isInClosedList{false};

            Node(Planners::utils::Vec3i coord_, Node *parent_ = nullptr);
            Node();
            unsigned int getScore();
            unsigned int getScoreWithSafetyCost();

        };
        using MagicalMultiSet = boost::multi_index_container<
          Node*, // the data type stored
          boost::multi_index::indexed_by< // list of indexes
            boost::multi_index::hashed_unique<  //hashed index over 'l'
              boost::multi_index::tag<IndexByWorldPosition>, // give that index a name
              boost::multi_index::member<Node, unsigned int, &Node::world_index> // what will be the index's key
            >,
            boost::multi_index::ordered_non_unique<  //ordered index over 'i1'
              boost::multi_index::tag<IndexByCost>, // give that index a name
              boost::multi_index::member<Node, unsigned int, &Node::gplush> // what will be the index's key
            >
          >
        >;


        typedef MagicalMultiSet::index<IndexByWorldPosition>::type node_by_position;
        typedef MagicalMultiSet::index<IndexByCost>::type          node_by_cost;
        /**
         * @brief Comparator passed to the Open and Closed sets to automatically order the lists depending
         * on the nodes total costs 
         */
        struct NodeComparator{
            
            bool operator()(const Node *const &_lhs, const Node *const &_rhs) const
	        {
		        auto res = static_cast<long int>(_lhs->G + _lhs->H) - static_cast<long int>(_rhs->G + _rhs->H);
		        if (res == 0)
		        {
		        	res = _lhs->coordinates.x - _rhs->coordinates.x;
		        }
		        if (res == 0)
		        {
		        	res = _lhs->coordinates.y - _rhs->coordinates.y;
		        }
		        if (res == 0)
		        {
		        	res = _lhs->coordinates.z - _rhs->coordinates.z;
		        }

		        if (res == 0)
		        {
		        	res = _lhs->parent->coordinates.x - _rhs->parent->coordinates.x;
		        }
		        if (res == 0)
		        {
		        	res = _lhs->parent->coordinates.y - _rhs->parent->coordinates.y;
		        }
		        if (res == 0)
		        {
		        	res = _lhs->parent->coordinates.z - _rhs->parent->coordinates.z;
		        }
		    return res < 0;
        	}
        };

    }
}

#endif