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

namespace Planners
{
    namespace utils
    {
        class Vec3i;
        class Node;
        struct NodeComparator;

        using CoordinateList  = std::vector<Planners::utils::Vec3i>;
        using NodeSet         = std::set<Node*, NodeComparator>;
        using DataVariant     = std::variant<std::string, Vec3i, CoordinateList, double, size_t, int, bool>;
        using PathData        = std::map<std::string, DataVariant>;
        
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
            unsigned int G{0}, H{0};
            unsigned int cost{0};
            
            bool occuppied{false};
            bool isInOpenList{false};
            bool isInClosedList{false};

            Node(Planners::utils::Vec3i coord_, Node *parent_ = nullptr);
            Node();
            unsigned int getScore();

        };

        /**
         * @brief Comparator passed to the Open and Closed sets to automatically order the lists depending
         * on the nodes total costs 
         */
        struct NodeComparator{
            
            bool operator()(const Node *const &lhs__, const Node *const &rhs__) const
	        {
		        const Node *lhs = lhs__;
		        const Node *rhs = rhs__;
                long int lc = (lhs->G + lhs->H);
                long int rc = (rhs->G + rhs->H);
		        auto res = lc - rc;
		        if (res == 0)
		        {
		        	res = lhs->coordinates.x - rhs->coordinates.x;
		        }
		        if (res == 0)
		        {
		        	res = lhs->coordinates.y - rhs->coordinates.y;
		        }
		        if (res == 0)
		        {
		        	res = lhs->coordinates.z - rhs->coordinates.z;
		        }

		        if (res == 0)
		        {
		        	res = lhs->parent->coordinates.x - rhs->parent->coordinates.x;
		        }
		        if (res == 0)
		        {
		        	res = lhs->parent->coordinates.y - rhs->parent->coordinates.y;
		        }
		        if (res == 0)
		        {
		        	res = lhs->parent->coordinates.z - rhs->parent->coordinates.z;
		        }
		    return res < 0;
        	}
        };

    }
}

#endif