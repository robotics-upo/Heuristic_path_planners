#ifndef BRESENHAM_HPP
#define BRESENHAM_HPP

#include "utils/utils.hpp"
#include "utils/world.hpp"
#include "utils/geometry_utils.hpp"

namespace Planners
{

    namespace utils
    {
        namespace LineOfSight
        {
//             bool lineofsight(Node &p1, Node &p2)
//             {
//                 //printf("Checking LofS: [%d, %d, %d] to [%d, %d, %d]\n", p1.point.x, p1.point.y, p1.point.z, p2.point.x, p2.point.y, p2.point.z);

//                 // Added variables
//                 float step = 0.05;  // Chech the value
//                 float ws_x_max=56;
//                 float ws_x_min=56;
//                 float ws_y_max=37;
//                 float ws_y_min=12;
//                 float ws_z_max=15;
//                 float ws_z_min=0;

//                 //min distance for no collision, in discrete space
//                 float min_distance = 5.0 * step; // 1.9; //sqrt(8);
//                 int extra_cells = 0;             // min(2.0f,min_distance);

//                 // std::cout << "workspace_x_min: " << ws_x_min << std::endl;
//                 // std::cout << "workspace_x_max: " << ws_x_max << std::endl;
//                 // std::cout << "workspace_y_min: " << ws_y_min << std::endl;
//                 // std::cout << "workspace_y_max: " << ws_y_max << std::endl;
//                 // std::cout << "workspace_z_min: " << ws_z_min << std::endl;
//                 // std::cout << "workspace_z_max: " << ws_z_max << std::endl;

//                 // std::cout << "P1_x: " << p1.point.x << std::endl;
//                 // std::cout << "P1_y: " << p1.point.y << std::endl;
//                 // std::cout << "P1_z: " << p1.point.z << std::endl;
//                 // std::cout << "P2_x: " << p2.point.x << std::endl;
//                 // std::cout << "P2_y: " << p2.point.y << std::endl;
//                 // std::cout << "P2_z: " << p1.point.z << std::endl;

//                 //distance doble triangle.
//                 // int x0 = max(min(p1.point.x, p2.point.x) - extra_cells, ws_x_min);
//                 // int x1 = min(max(p1.point.x, p2.point.x) + extra_cells, ws_x_max);
//                 // int y0 = max(min(p1.point.y, p2.point.y) - extra_cells, ws_y_min);
//                 // int y1 = min(max(p1.point.y, p2.point.y) + extra_cells, ws_y_max);
//                 // int z0 = max(min(p1.point.z, p2.point.z) - extra_cells, ws_z_min);
//                 // int z1 = min(max(p1.point.z, p2.point.z) + extra_cells, ws_z_max);

//                 int x0 = max(min(p1.coordinates.x, p2.coordinates.x) - extra_cells, ws_x_min);
//                 int x1 = min(max(p1.coordinates.x, p2.coordinates.x) + extra_cells, ws_x_max);
//                 int y0 = max(min(p1.coordinates.y, p2.coordinates.y) - extra_cells, ws_y_min);
//                 int y1 = min(max(p1.coordinates.y, p2.coordinates.y) + extra_cells, ws_y_max);
//                 int z0 = max(min(p1.coordinates.z, p2.coordinates.z) - extra_cells, ws_z_min);
//                 int z1 = min(max(p1.coordinates.z, p2.coordinates.z) + extra_cells, ws_z_max);

//                 if (isOccupied(p1) || isOccupied(p2))
//                     return false;

//                 float base = distanceBetween2nodes(p1, p2);
//                 if (base > line_of_sight / step)
//                     return false;

//                 for (int x = x0; x <= x1; x++)
//                     for (int y = y0; y <= y1; y++)
//                         for (int z = z0; z <= z1; z++)
//                         {
//                             //If the point is occupied, we have to calculate distance to the line.
//                             if (!discrete_world[getWorldIndex(x, y, z)].notOccupied) //notOccupied false means that the cell is occupied.
//                             {
//                                 //If base is zero and node is occcupied, directly does not exist line of sight
//                                 if (base == 0)
//                                 {
// #ifdef SEND_NO_LOFS_NODES_MARKERS
//                                     geometry_msgs::Point p;
//                                     p.x = p2->coordinates.x * step;
//                                     p.y = p2->coordinates.x * step;
//                                     p.z = p2->coordinates.x * step;
//                                     marker_no_los.header.stamp = ros::Time();
//                                     marker_no_los.header.seq++;
//                                     marker_no_los.points.push_back(p);
//                                     no_los_marker_pub_.publish(marker_no_los);
// #endif

//                                     return false;
//                                 }

//                                 //cout << "Cell is occupied" << endl;
//                                 float a = sqrt(pow(x - p1.coordinates.x, 2) + pow(p1.coordinates.y - y, 2) + pow(p1.coordinates.z - z, 2));
//                                 float c = sqrt(pow(p2.coordinates.x - x, 2) + pow(p2.coordinates.y - y, 2) + pow(p2.coordinates.z - z, 2));
//                                 float l = (pow(base, 2) + pow(a, 2) - pow(c, 2)) / (2 * base);
//                                 float distance = sqrt(abs(pow(a, 2) - pow(l, 2)));

//                                 if (distance <= min_distance)
//                                 {
// #ifdef SEND_NO_LOFS_NODES_MARKERS
//                                     geometry_msgs::Point p;
//                                     p.x = p2.coordinates.x * step;
//                                     p.y = p2.coordinates.y * step;
//                                     p.z = p2.coordinates.z * step;
//                                     marker_no_los.header.stamp = ros::Time();
//                                     marker_no_los.header.seq++;
//                                     marker_no_los.points.push_back(p);
//                                     no_los_marker_pub_.publish(marker_no_los);
// #endif
//                                     return false;
//                                 }
//                             }
//                         }

//                 return true;
//             }

            //bool bresenham3D(Node &_lnode, Node &_rnode, DiscreteWorld &_world)
            bool bresenham3D(Node *_lnode, Node *_rnode, DiscreteWorld &_world)
            {
                //Save solution in visitors.point.x, visitors.point.y and visitors.point.z
                std::vector<Vec3i> list_visitors;
                int xs, ys, zs, px, py, pz, d1, d2, tam;

                int x0 = _lnode->coordinates.x;
                int y0 = _lnode->coordinates.y;
                int z0 = _lnode->coordinates.z;

                int x1 = _rnode->coordinates.x;
                int y1 = _rnode->coordinates.y;
                int z1 = _rnode->coordinates.z;

                // int x0 = _lnode.coordinates.x;
                // int y0 = _lnode.coordinates.y;
                // int z0 = _lnode.coordinates.z;

                // int x1 = _rnode.coordinates.x;
                // int y1 = _rnode.coordinates.y;
                // int z1 = _rnode.coordinates.z;

                int dx = std::abs(x1 - x0);
                int dy = std::abs(y1 - y0);
                int dz = std::abs(z1 - z0);

                //
                if (_world.isOccupied(_lnode->coordinates.x, _lnode->coordinates.y, _lnode->coordinates.z) ||
                    _world.isOccupied(_rnode->coordinates.x, _rnode->coordinates.y, _rnode->coordinates.z))
                    return false;

                float base = Planners::utils::geometry::distanceBetween2nodesOTRO(_lnode, _rnode);
                //float base = distanceBetween2nodes(_lnode, _rnode);

                /*
                Think if this is the best place to put this
                
                if (base > line_of_sight / step)
                    return false;
                */

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

                        list_visitors.insert(list_visitors.begin(), {x0, y0, z0});
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

                        list_visitors.insert(list_visitors.begin(), {x0, y0, z0});
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

                        list_visitors.insert(list_visitors.begin(), {x0, y0, z0});
                    }
                }

                return true;
            }
        }
    }
}

#endif