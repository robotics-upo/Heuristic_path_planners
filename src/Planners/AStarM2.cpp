#include "Planners/AStarM2.hpp"

namespace Planners
{

    AStarM2::AStarM2(bool _use_3d) : AStar(_use_3d, "astarm2") {}
    AStarM2::AStarM2(bool _use_3d, std::string _name = "astarm2") : AStar(_use_3d, _name) {}

    inline unsigned int AStarM2::computeG(const Planners::utils::Node *_current, Planners::utils::Node *_suc, unsigned int _n_i, unsigned int _dirs)
    {

        unsigned int cost = 0;

        if (_dirs == 8)
        {
            cost = (_n_i < 4 ? dist_scale_factor_ : dd_2D_); // This is more efficient
        }
        else
        {
            cost = (_n_i < 6 ? dist_scale_factor_ : (_n_i < 18 ? dd_2D_ : dd_3D_)); // This is more efficient
        }

        double cc = (_current->cost + _suc->cost) / 2;

        auto edge_neighbour = static_cast<unsigned int>(cc * cost_weight_ * dist_scale_factor_reduced_);

        cost += (_current->G + edge_neighbour);

        _suc->C = edge_neighbour;

        return cost;
    }
}
