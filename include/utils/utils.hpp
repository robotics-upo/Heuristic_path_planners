#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <any>

#include <math.h>

namespace Planners
{
    namespace utils
    {
        class Vec3i;
        class Node;
        struct NodeComparator;

        using CoordinateList = std::vector<Planners::utils::Vec3i>;
        using NodeSet = std::set<Node*, NodeComparator>;
        using PathData = std::map<std::string, std::any>;
        
        class Vec3i
        {
            public:
            int x, y, z;    

            bool operator==(const Vec3i &coordinates_);
            Planners::utils::Vec3i &operator/=(float _divid);
            Vec3i operator+(const Vec3i &right_) const
            {
                return {this->x + right_.x, this->y + right_.y, this->z + right_.z};
            }
            Vec3i operator-(const Vec3i &right_) const
            {
                return {this->x - right_.x, this->y - right_.y, this->z - right_.z};
            }
            inline Vec3i operator*(int &_mult) const
            {
                return {this->x * _mult, this->y * _mult, this->z * _mult };
            }
        };
        inline Planners::utils::Vec3i operator*(const int &_mult, const Planners::utils::Vec3i &_vec){
                    
            return { _vec.x * _mult, _vec.y * _mult, _vec.z * _mult};
        }

        inline std::ostream& operator<<(std::ostream& os, const Vec3i& dt)
        {
            os << "[" << dt.x << ", " << dt.y << ", " << dt.y << "]";
            return os;
        }

        class Node
        {
            public:
            Node *parent{nullptr};

            Planners::utils::Vec3i coordinates;
            unsigned int G{0}, H{0};
            
            bool occuppied{false};
            bool isInOpenList{false};
            bool isInClosedList{false};

            Node(Planners::utils::Vec3i coord_, Node *parent_ = nullptr);
            Node();
            unsigned int getScore();

        };

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