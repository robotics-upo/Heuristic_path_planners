#include "utils/LineOfSight.hpp"

namespace Planners
{
    namespace utils
    {
        namespace LineOfSight
        {

            bool bresenham3D(const Node *_lnode, const Node *_rnode, DiscreteWorld &_world)
            {
                int xs, ys, zs, d1, d2, tam;

                int x0 = _lnode->coordinates.x;
                int y0 = _lnode->coordinates.y;
                int z0 = _lnode->coordinates.z;

                int x1 = _rnode->coordinates.x;
                int y1 = _rnode->coordinates.y;
                int z1 = _rnode->coordinates.z;

                int dx = std::abs(x1 - x0);
                int dy = std::abs(y1 - y0);
                int dz = std::abs(z1 - z0);

                x1 > x0 ? xs = 1 : xs = -1;
                y1 > y0 ? ys = 1 : ys = -1;
                z1 > z0 ? zs = 1 : zs = -1;

                //Driving axis is X-axis
                if (dx >= dy &&
                    dx >= dz)
                {
                    d1 = 2 * dy - dx;
                    d2 = 2 * dz - dx;
                    while (x0 != x1)
                    {
                        x0 += xs;
                        if (d1 >= 0)
                        {
                            y0 += ys;
                            d1 -= 2 * dx;
                        }
                        if (d2 >= 0)
                        {
                            z0 += zs;
                            d2 -= 2 * dx;
                        }
                        d1 += 2 * dy;
                        d2 += 2 * dz;
                        //Check if visitor is occupied and add visitor
                        if (_world.isOccupied(x0, y0, z0))
                            return false;

                    }
                }
                //Driving axis is Y-axis
                else if (dy >= dx &&
                         dy >= dz)
                {
                    d1 = 2 * dx - dy;
                    d2 = 2 * dz - dy;
                    while (y0 != y1)
                    {
                        y0 += ys;
                        if (d1 >= 0)
                        {
                            x0 += xs;
                            d1 -= 2 * dy;
                        }
                        if (d2 >= 0)
                        {
                            z0 += zs;
                            d2 -= 2 * dy;
                        }
                        d1 += 2 * dx;
                        d2 += 2 * dz;
                        //Check if visitor is occupied and add visitor
                        if (_world.isOccupied(x0, y0, z0))
                            return false;

                    }
                }
                //Driving axis is Z-axis
                else
                {
                    d1 = 2 * dy - dz;
                    d2 = 2 * dx - dz;
                    while (z0 != z1)
                    {
                        z0 += zs;
                        if (d1 >= 0)
                        {
                            y0 += ys;
                            d1 -= 2 * dz;
                        }
                        if (d2 >= 0)
                        {
                            x0 += xs;
                            d2 -= 2 * dz;
                        }
                        d1 += 2 * dy;
                        d2 += 2 * dx;
                        //Check if visitor is occupied and add visitor
                        if (_world.isOccupied(x0, y0, z0))
                            return false;

                    }
                }

                return true;
            }
        }
    }
}