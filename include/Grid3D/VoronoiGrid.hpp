#ifndef __VORONOIGRID_HPP__
#define __VORONOIGRID_HPP__

#include <sys/time.h>
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <stdio.h>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include "utils/utils.hpp"
#include "Grid3D/grid3d.hpp"
#include "voro++-0.4.6/src/voro++.hh"

class VoronoiGrid : public Grid3d
{
protected:
	ros::Subscriber map2d_sub_;
	// ros::Publisher
	nav_msgs::OccupancyGrid occ_grid_2d_;

	inline Vec3i indexToXY(const unsigned int &_index, const unsigned int _grid_width)
	{

		return {static_cast<int>(std::floor(_index % _grid_width)),
				static_cast<int>(std::floor(_index / _grid_width)),
				0};
	}

	void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr &_grid)
	{

		std::cout << "Received occuapncy grid" << std::endl;
		occ_grid_2d_ = *_grid;

		// Set up constants for the container geometry
		const double x_min = 0, x_max = occ_grid_2d_.info.width * occ_grid_2d_.info.resolution;
		const double y_min = 0, y_max = occ_grid_2d_.info.height * occ_grid_2d_.info.resolution;
		const double z_min = 0, z_max = occ_grid_2d_.info.resolution;

		const double cvol = (x_max - x_min) * (y_max - y_min) * (x_max - x_min);

		// Set up the number of blocks that the container is divided into
		const int n_x = 6, n_y = 6, n_z = 6;

		voro::container con(x_min, x_max, y_min, y_max, z_min, z_max, n_x, n_y, n_z,
							false, false, false, 8);
		int n = 0;
		Vec3i pos;
		for (int i = 0; i < occ_grid_2d_.data.size(); ++i)
		{
			if (occ_grid_2d_.data.at(i) == 100)
			{
				pos = indexToXY(i, occ_grid_2d_.info.width);
				std::cout << "Adding pos: " << pos << std::endl;
			}
				con.put(n, pos.x*occ_grid_2d_.info.resolution, pos.y*occ_grid_2d_.info.resolution, 0);
		}
		con.print_custom("%q", "/home/fali/test_v.voro");
		// Output the particle positions in gnuplot format
		// con.draw_particles("voronoi_ros_map_p.gnu");
		// Output the Voronoi cells in gnuplot format
		// con.draw_cells_gnuplot("voronoi_ros_map_v.gnu");
	}

public:
	VoronoiGrid()
	{
		std::cout << "Hello from Voronoi Grid!" << std::endl;
		voro::voronoicell v;
		file_ext_ = ".vgridm";
		std::cout << "Voronoi potential maps have " << file_ext_ << " extension!" << std::endl;

		ros::NodeHandle lnh("~");

		map2d_sub_ = lnh.subscribe<nav_msgs::OccupancyGrid>("/grid", 1, &VoronoiGrid::occupancyGridCallback, this);

		// Load paraeters
		double value;
		lnh.param("name", m_nodeName, std::string("vorogrid3d"));
		if (!lnh.getParam("global_frame_id", m_globalFrameId))
			m_globalFrameId = "map";
		if (!lnh.getParam("map_path", m_mapPath))
			m_mapPath = "map.ot";
		if (!lnh.getParam("publish_point_cloud", m_publishPc))
			m_publishPc = true;
		if (!lnh.getParam("publish_point_cloud_rate", m_publishPointCloudRate))
			m_publishPointCloudRate = 0.2;
		if (!lnh.getParam("publish_grid_slice", value))
			value = 1.0;
		if (!lnh.getParam("publish_grid_slice_rate", m_publishGridSliceRate))
			m_publishGridSliceRate = 0.2;
		m_gridSlice = (float)value;
		if (!lnh.getParam("sensor_dev", value))
			value = 0.2;
		m_sensorDev = (float)value;

		lnh.param("cost_scaling_factor", cost_scaling_factor, 0.8);
		lnh.param("robot_radius", robot_radius, 0.4);
		lnh.param("use_costmap_function", use_costmap_function, (bool)true);

		// Load octomap
		m_octomap = NULL;
		m_grid = NULL;

		if (loadOctomap(m_mapPath))
		{
			// Compute the point-cloud associated to the ocotmap
			computePointCloud();

			// Try to load tha associated grid-map from file
			std::string path;
			if (m_mapPath.compare(m_mapPath.length() - 3, 3, ".bt") == 0)
				path = m_mapPath.substr(0, m_mapPath.find(".bt")) + file_ext_;

			if (!loadGrid(path))
			{
				std::cout << "Computing Voronoi 3D occupancy grid. This will take some time..." << std::endl;
				computeGrid();
				std::cout << "\tdone!" << std::endl;

				if (saveGrid(path))
					std::cout << "Grid map successfully saved on " << path << std::endl;
			}

			// Build the msg with a slice of the grid if needed
			if (m_gridSlice >= 0 && m_gridSlice <= m_maxZ)
			{
				buildGridSliceMsg(m_gridSlice);
				m_gridSlicePub = m_nh.advertise<nav_msgs::OccupancyGrid>(m_nodeName + "/grid_slice", 1, true);
				// gridTimer = m_nh.createTimer(ros::Duration(1.0 / m_publishGridSliceRate), &VoronoiGrid::publishGridSliceTimer, this);
			}

			// Setup point-cloud publisher
			if (m_publishPc)
			{
				m_pcPub = m_nh.advertise<sensor_msgs::PointCloud2>(m_nodeName + "/map_point_cloud", 1, true);
				// mapTimer = m_nh.createTimer(ros::Duration(1.0 / m_publishPointCloudRate), &VoronoiGrid::publishMapPointCloudTimer, this);
			}
		}
	}

protected:
	virtual void computeGrid(void)
	{
		//Publish percent variable
		std_msgs::Float32 percent_msg;
		percent_msg.data = 0;
		// Alloc the 3D grid
		m_gridSizeX = (int)(m_maxX * m_oneDivRes);
		m_gridSizeY = (int)(m_maxY * m_oneDivRes);
		m_gridSizeZ = (int)(m_maxZ * m_oneDivRes);
		m_gridSize = m_gridSizeX * m_gridSizeY * m_gridSizeZ;
		m_gridStepY = m_gridSizeX;
		m_gridStepZ = m_gridSizeX * m_gridSizeY;
		m_grid = new Planners::utils::gridCell[m_gridSize];

		// Setup kdtree
		m_kdtree.setInputCloud(m_cloud);

		// Compute the distance to the closest point of the grid
		int index;
		float dist;
		float gaussConst1 = 1. / (m_sensorDev * sqrt(2 * M_PI));
		float gaussConst2 = 1. / (2 * m_sensorDev * m_sensorDev);
		pcl::PointXYZI searchPoint;
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);
		double count = 0;
		double percent;
		double size = m_gridSizeX * m_gridSizeY * m_gridSizeZ;
		for (int iz = 0; iz < m_gridSizeZ; iz++)
		{
			for (int iy = 0; iy < m_gridSizeY; iy++)
			{
				for (int ix = 0; ix < m_gridSizeX; ix++)
				{
					searchPoint.x = ix * m_resolution;
					searchPoint.y = iy * m_resolution;
					searchPoint.z = iz * m_resolution;
					index = ix + iy * m_gridStepY + iz * m_gridStepZ;
					++count;
					percent = count / size * 100.0;
					ROS_INFO_THROTTLE(0.5, "Progress: %lf %%", percent);
					if (percent > percent_msg.data + 0.5)
					{
						percent_msg.data = percent;
						// percent_computed_pub_.publish(percent_msg);
					}

					if (m_kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
					{
						dist = pointNKNSquaredDistance[0];
						m_grid[index].dist = dist;
						if (!use_costmap_function)
						{
							m_grid[index].prob = gaussConst1 * exp(-dist * dist * gaussConst2);
						}
						else
						{
							double prob = 100 * exp(-cost_scaling_factor * std::fabs((dist - robot_radius)));
							// ROS_INFO("[%f, %f, %f] Dist: %f Probability: %f", searchPoint.x, searchPoint.y, searchPoint.z, dist, prob);
							m_grid[index].prob = prob;
						}
					}
					else
					{
						m_grid[index].dist = -1.0;
						m_grid[index].prob = 0.0;
					}
				}
			}
		}
		percent_msg.data = 100;
		// percent_computed_pub_.publish(percent_msg);
	}
};

#endif
