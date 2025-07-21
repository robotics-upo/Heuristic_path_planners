 #ifndef __LOCAL_GRID3D_HPP__
#define __LOCAL_GRID3D_HPP__

/**
 * @file prob_map.cpp
 * @brief This file includes the ROS node implementation.
 * @author Francisco J. Perez Grau, Fernando Caballero and José Antonio Cobano
 */

#include <sys/time.h>
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32.h>
#include <stdio.h> 
// PCL
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

#include <pcl/pcl_exports.h> // for PCL_EXPORTS
#include <pcl/common/point_tests.h> // for pcl::isFinite
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "utils/utils.hpp"

#include <torch/torch.h>
#include <torch/script.h>
#include <ros/ros.h>


#include <ctime> 
#include <sstream> //for std::ostringstream
#include <fstream>
#include <iomanip> //for std::setw, std::hex, and std::setfill
#include <openssl/evp.h> //for all other OpenSSL function calls
#include <openssl/sha.h> //for SHA512_DIGEST_LENGTH
#include <chrono>
// #include "utils/ros/ROSInterfaces.hpp"

// #ifdef BUILD_VORONOI
// #include "voro++-0.4.6/src/voro++.hh"
// #endif

struct TrilinearParams
{
	float a0, a1, a2, a3, a4, a5, a6, a7;

	TrilinearParams(void)
	{
		a0 = a1 = a2 = a3 = a4 = a5 = a6 = a7 = 0.0;
	}

	double interpolate(double x, double y, double z)
	{
		return a0 + a1*x + a2*y + a3*z + a4*x*y + a5*x*z + a6*y*z + a7*x*y*z;
	}
};

class Local_Grid3d
{
private:
	
	// Ros parameters
	ros::NodeHandle m_nh;
	bool m_publishPc;
	std::string m_mapPath, m_nodeName;
	std::string m_globalFrameId;
	float m_sensorDev, m_gridSlice;
	double m_publishPointCloudRate, m_publishGridSliceRate;
	
	// Octomap parameters
	float m_maxX, m_maxY, m_maxZ;
	float m_oneDivRes;
	octomap::OcTree *m_octomap;
	
	
	// 3D point cloud representation of the map
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud, filter_cloud;
	pcl::KdTreeFLANN<pcl::PointXYZ> m_kdtree;
	
	// Visualization of the map as pointcloud
	sensor_msgs::PointCloud2 m_pcMsg;
	ros::Publisher m_pcPub, percent_computed_pub_;
	ros::Timer mapTimer;
			
	// Visualization of a grid slice as 2D grid map msg
	nav_msgs::OccupancyGrid m_gridSliceMsg;
	ros::Publisher m_gridSlicePub;
	ros::Timer gridTimer;

	//Parameters added to allow a new exp function to test different gridmaps
	double cost_scaling_factor, robot_radius;
	bool use_costmap_function;
	
public:
	// 3D probabilistic grid cell
	Planners::utils::gridCell *m_grid;
	int m_gridSize, m_gridSizeX, m_gridSizeY, m_gridSizeZ;
	int m_gridStepY, m_gridStepZ;
	float m_resolution;

	// Local_Grid3d(): m_cloud(new pcl::PointCloud<pcl::PointXYZI>)
	Local_Grid3d(): m_cloud(new pcl::PointCloud<pcl::PointXYZ>)
	{
		// Load parameters INCLUDE PARAMETERS OF THE SIZE OF THE LOCAL SIZE
		double value;
		ros::NodeHandle lnh("~");
		lnh.param("name", m_nodeName, std::string("local_grid3d"));
		if(!lnh.getParam("global_frame_id", m_globalFrameId)) // JAC: Local planner --> base_link or occupancy map?
			m_globalFrameId = "map";	
		// if(!lnh.getParam("map_path", m_mapPath))
		// 	m_mapPath = "map.ot";
		if(!lnh.getParam("publish_point_cloud", m_publishPc))
			m_publishPc = true;
		if(!lnh.getParam("publish_point_cloud_rate", m_publishPointCloudRate))
			m_publishPointCloudRate = 0.2;	
		if(!lnh.getParam("publish_grid_slice", value))
			value = 1.0;
		if(!lnh.getParam("publish_grid_slice_rate", m_publishGridSliceRate))
			m_publishGridSliceRate = 0.2;
		m_gridSlice = (float)value;
		if(!lnh.getParam("sensor_dev", value))
			value = 0.2;
		m_sensorDev = (float)value;

		lnh.param("cost_scaling_factor", cost_scaling_factor, 0.8); //0.8		
		lnh.param("robot_radius", robot_radius, 0.4);		//0.4
		lnh.param("use_costmap_function", use_costmap_function, (bool)true);	

		m_grid = NULL;
        if(m_grid != NULL)
			delete []m_grid;

		// configureParameters();
		if(configureParameters())
		{
            //computeLocalGrid(m_cloud);
			
			// Build the msg with a slice of the grid if needed
			if(m_gridSlice >= 0 && m_gridSlice <= m_maxZ)
			{
				buildGridSliceMsg(m_gridSlice);
				m_gridSlicePub = m_nh.advertise<nav_msgs::OccupancyGrid>(m_nodeName+"/grid_slice", 1, true);
				gridTimer      = m_nh.createTimer(ros::Duration(1.0/m_publishGridSliceRate), &Local_Grid3d::publishGridSliceTimer, this);	
			}
			
			// Setup point-cloud publisher
			if(m_publishPc)
			{
				m_pcPub  = m_nh.advertise<sensor_msgs::PointCloud2>(m_nodeName+"/map_point_cloud", 1, true);
				mapTimer = m_nh.createTimer(ros::Duration(1.0/m_publishPointCloudRate), &Local_Grid3d::publishMapPointCloudTimer, this);
			}
			percent_computed_pub_ = m_nh.advertise<std_msgs::Float32>(m_nodeName+"/percent_computed", 1, false);
        }
        else{
			// std::cout << "\tCOMPUTE GRID" << std::endl;
            // computeLocalGrid(m_cloud);
            // JAC: Here "Build the msg with a slice of the grid if needed" should be?
        }
            
	}

	~Local_Grid3d(void)
	{
		// if(m_octomap != NULL)
		// 	delete m_octomap;
		if(m_grid != NULL)
			delete []m_grid;
	}
	bool setCostParams(const double &_cost_scaling_factor, const double &_robot_radius){

		cost_scaling_factor = std::fabs(_cost_scaling_factor);
		robot_radius 		= std::fabs(_robot_radius);

		return true;
	}
	
	void setGridSliceHeight(const double _height){
		if(_height < m_maxZ && _height > 0.0 ){
			m_gridSlice = _height;
			buildGridSliceMsg(m_gridSlice);
		}
	}
  
	void publishMapPointCloud(void)
	{
		m_pcMsg.header.stamp = ros::Time::now();
		m_pcPub.publish(m_pcMsg);
	}
	
	void publishGridSlice(void)
	{
		m_gridSliceMsg.header.stamp = ros::Time::now();
		m_gridSlicePub.publish(m_gridSliceMsg);
	}

	TrilinearParams computeDistInterpolation(const double x, const double y, const double z)
	{
		TrilinearParams r;

		if(isIntoMap(x, y, z) && isIntoMap(x+m_resolution, y+m_resolution, z+m_resolution))
		{
			// Get 3D point index
			uint64_t i = point2grid(x, y, z); 

			// Get neightbour values to compute trilinear interpolation
			float c000, c001, c010, c011, c100, c101, c110, c111;
			c000 = m_grid[i].dist; 
			c001 = m_grid[i+m_gridStepZ].dist; 
			c010 = m_grid[i+m_gridStepY].dist; 
			c011 = m_grid[i+m_gridStepY+m_gridStepZ].dist; 
			c100 = m_grid[i+1].dist; 
			c101 = m_grid[i+1+m_gridStepZ].dist; 
			c110 = m_grid[i+1+m_gridStepY].dist; 
			c111 = m_grid[i+1+m_gridStepY+m_gridStepZ].dist; 

			// Compute trilinear parameters
			const float div = -m_oneDivRes*m_oneDivRes*m_oneDivRes;
			float x0, y0, z0, x1, y1, z1;
			x0 = ((int)(x*m_oneDivRes))*m_resolution;
			x1 = x0+m_resolution;
			y0 = ((int)(y*m_oneDivRes))*m_resolution;
			y1 = y0+m_resolution;
			z0 = ((int)(z*m_oneDivRes))*m_resolution;
			z1 = z0+m_resolution;
			r.a0 = (-c000*x1*y1*z1 + c001*x1*y1*z0 + c010*x1*y0*z1 - c011*x1*y0*z0 
			+ c100*x0*y1*z1 - c101*x0*y1*z0 - c110*x0*y0*z1 + c111*x0*y0*z0)*div;
			r.a1 = (c000*y1*z1 - c001*y1*z0 - c010*y0*z1 + c011*y0*z0
			- c100*y1*z1 + c101*y1*z0 + c110*y0*z1 - c111*y0*z0)*div;
			r.a2 = (c000*x1*z1 - c001*x1*z0 - c010*x1*z1 + c011*x1*z0 
			- c100*x0*z1 + c101*x0*z0 + c110*x0*z1 - c111*x0*z0)*div;
			r.a3 = (c000*x1*y1 - c001*x1*y1 - c010*x1*y0 + c011*x1*y0 
			- c100*x0*y1 + c101*x0*y1 + c110*x0*y0 - c111*x0*y0)*div;
			r.a4 = (-c000*z1 + c001*z0 + c010*z1 - c011*z0 + c100*z1 
			- c101*z0 - c110*z1 + c111*z0)*div;
			r.a5 = (-c000*y1 + c001*y1 + c010*y0 - c011*y0 + c100*y1 
			- c101*y1 - c110*y0 + c111*y0)*div;
			r.a6 = (-c000*x1 + c001*x1 + c010*x1 - c011*x1 + c100*x0 
			- c101*x0 - c110*x0 + c111*x0)*div;
			r.a7 = (c000 - c001 - c010 + c011 - c100
			+ c101 + c110 - c111)*div;
		}

		return r;
	}

	
	bool isIntoMap(float x, float y, float z)
	{
		return (x >= 0.0 && y >= 0.0 && z >= 0.0 && x <= (2*m_maxX) && y <= (2*m_maxY) && z <= (2*m_maxZ));
	}
	// int getCellCost(const float &_x, const float &_y, const float &_z){
	// JAC Precision
	float getCellCost(const float &_x, const float &_y, const float &_z){
		
		if( !isIntoMap(_x, _y, _z) ){
			std::cout << "OUT " << std::endl;
			return 0;
		}
			

		int index = point2grid(_x, _y, _z);
		
		// return m_grid[index].prob;
		return m_grid[index].dist;
	}

	// NEW VERSION -- GUILLERMO

	void computeLocalGrid(torch::jit::script::Module& loaded_sdf, float drone_x, float drone_y, float drone_z)
	{
		// Build global positions vector
		std::vector<std::vector<float>> coordinates_vector;
        coordinates_vector.reserve(m_gridSizeX*m_gridSizeY*m_gridSizeZ);
		// std::cout << "Drone position: " << "x = " << drone_x << " | y = " << drone_y << " | z = " << drone_z << std::endl;
		for(int iz=0; iz<m_gridSizeZ; iz++)
		{
			for(int iy=0; iy<m_gridSizeY; iy++)
			{
				for(int ix=0; ix<m_gridSizeX; ix++)
				{
				float xc = drone_x + (ix - (m_gridSizeX -1) / 2) * m_resolution;
				float yc = drone_y + (iy - (m_gridSizeY -1) / 2) * m_resolution;
				float zc = drone_z + (iz - (m_gridSizeZ -1) / 2) * m_resolution;

				coordinates_vector.push_back({xc, yc, zc});
				}
			}
		}

		// Convert vector tu libtorch array and query the neural network
        auto num_points = coordinates_vector.size();
        torch::Tensor coordinates_tensor = torch::zeros({static_cast<long>(num_points), 3}, torch::kFloat);
        for (size_t i = 0; i < num_points; ++i) {
            coordinates_tensor[i][0] = coordinates_vector[i][0];
            coordinates_tensor[i][1] = coordinates_vector[i][1];
            coordinates_tensor[i][2] = coordinates_vector[i][2];
        }

		auto start = std::chrono::high_resolution_clock::now();
		torch::Tensor grid_output_tensor = loaded_sdf.forward({coordinates_tensor}).toTensor();
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;
        std::cout << "Points queried: " << num_points <<" |  Time taken to query model: " << duration.count() << " ms" << std::endl;

		// // Convert vector tu libtorch array and query the neural network
        // torch::Tensor coordinates_tensor_2 = torch::zeros({1, 3}, torch::kFloat);
        // coordinates_tensor_2[0][0] = coordinates_vector[0][0];
        // coordinates_tensor_2[0][1] = coordinates_vector[0][1];
        // coordinates_tensor_2[0][2] = coordinates_vector[0][2];

		// start = std::chrono::high_resolution_clock::now();
		// torch::Tensor grid_output_tensor_2 = loaded_sdf.forward({coordinates_tensor_2}).toTensor();
        // end = std::chrono::high_resolution_clock::now();
        // duration = end - start;
        // std::cout << "Points queried: 1 |  Time taken to query model: " << duration.count() << " ms" << std::endl;

		// Place the queried values into the m_grid distance data field
		for(int iz=0; iz<m_gridSizeZ; iz++)
		{
			for(int iy=0; iy<m_gridSizeY; iy++)
			{
				for(int ix=0; ix<m_gridSizeX; ix++)
				{
					int index = ix + iy*m_gridStepY + iz*m_gridStepZ;
					m_grid[index].dist = grid_output_tensor[index].item<float>();
				}
			}
		}
	}

	void bypassLocalGrid()
	{
		// Place positive placeholders in the local grid
		for(int iz=0; iz<m_gridSizeZ; iz++)
		{
			for(int iy=0; iy<m_gridSizeY; iy++)
			{
				for(int ix=0; ix<m_gridSizeX; ix++)
				{
					int index = ix + iy*m_gridStepY + iz*m_gridStepZ;
					m_grid[index].dist = 1;
				}
			}
		}
	}

	void computeLocalGridFIESTA(std::vector<double> esdf_buffer, float drone_x, float drone_y, float drone_z, int fiesta_range_x, int fiesta_range_y, int fiesta_range_z, double fiesta_resolution, double fiesta_x_min, double fiesta_y_min, double fiesta_z_min)
	{
		// Place the queried values into the m_grid distance data field
		for(int iz=0; iz<m_gridSizeZ; iz++)
		{
			for(int iy=0; iy<m_gridSizeY; iy++)
			{
				for(int ix=0; ix<m_gridSizeX; ix++)
				{
					// Calculate index in mgrid
					int index_mgrid = ix + iy*m_gridStepY + iz*m_gridStepZ;
					
					// Calculate global point to be computed
					float xc = drone_x + (ix - (m_gridSizeX -1) / 2) * m_resolution;
					float yc = drone_y + (iy - (m_gridSizeY -1) / 2) * m_resolution;
					float zc = drone_z + (iz - (m_gridSizeZ -1) / 2) * m_resolution;

					// Convert local point to local point in FIESTA's grid
					int index_fiesta = floor((zc-fiesta_z_min)/fiesta_resolution) + floor((yc-fiesta_y_min)/fiesta_resolution)*fiesta_range_z + floor((xc-fiesta_x_min)/fiesta_resolution)*fiesta_range_y*fiesta_range_z;

					// Populate the local grid
					if(esdf_buffer[index_fiesta] == -10000)
					{
						m_grid[index_mgrid].dist = 0.001;
					}
					else
					{
						m_grid[index_mgrid].dist = esdf_buffer[index_fiesta];
					}

				}
			}
		}
	}

	
	std::pair<Planners::utils::Vec3i, double>  getClosestObstacle(const Planners::utils::Vec3i& _coords){

		// pcl::PointXYZI searchPoint;
		pcl::PointXYZ searchPoint;

		searchPoint.x = _coords.x * m_resolution;
		searchPoint.y = _coords.y * m_resolution;
		searchPoint.z = _coords.z * m_resolution;

		int k = 1;
		m_kdtree.setInputCloud(m_cloud);
		std::vector<int> pointIdxNKNSearch(k);
		std::vector<float> pointNKNSquaredDistance(k);

		if(m_kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){

			Planners::utils::Vec3i result;

			result.x = std::round(m_cloud->points[pointIdxNKNSearch[0]].x / m_resolution);
			result.y = std::round(m_cloud->points[pointIdxNKNSearch[0]].y / m_resolution);
			result.z = std::round(m_cloud->points[pointIdxNKNSearch[0]].z / m_resolution);

			return std::make_pair(result, std::sqrt(pointNKNSquaredDistance[k-1]));	
		}else{
			return std::make_pair(Planners::utils::Vec3i{}, std::numeric_limits<double>::max());
		}	
	}

protected:

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
	void publishMapPointCloudTimer(const ros::TimerEvent& event)
	{
		publishMapPointCloud();
	}
	
	void publishGridSliceTimer(const ros::TimerEvent& event)
	{
		publishGridSlice();
	}
#pragma GCC diagnostic pop

    // JAC: Configure parameters of the size of the Local Grid
    bool configureParameters(void)
    {
		if(m_grid != NULL)
			delete []m_grid;

        // Get map parameters: They have to take from local_world_size_x, local_world_size_y , local_world_size_z of the launch
		double maxX, maxY, maxZ, res;
        
		maxX = 3.0; // distancia a cada lado del dron (en x)
        maxY = 3.0; // distancia a cada lado del dron (en y)
        maxZ = 1.6; // distancia a cada lado del dron (en z)
        res = 0.05;


		m_maxX = (float)(maxX);
		m_maxY = (float)(maxY);
		m_maxZ = (float)(maxZ);
		m_resolution = (float)res;
		m_oneDivRes = 1.0/m_resolution;

		// Grid parameters
		m_gridSizeX = (int)(m_maxX*m_oneDivRes)*2 + 1; // drone is in the center
		m_gridSizeY = (int)(m_maxY*m_oneDivRes)*2 + 1; 
		m_gridSizeZ = (int)(m_maxZ*m_oneDivRes)*2 + 1;
		m_gridSize = m_gridSizeX*m_gridSizeY*m_gridSizeZ;
		m_gridStepY = m_gridSizeX;
		m_gridStepZ = m_gridSizeX*m_gridSizeY;

		// Delete previous data (if any) and create new grid
		// if(m_grid != NULL)
		// {
		// printf("Deleting %p\n", m_grid);
		// delete []m_grid;
		// }
		m_grid = new Planners::utils::gridCell[m_gridSize];
		printf("Creating %p, of size %d\n", m_grid, m_gridSize);


		// float workspace_x, workspace_y, workspace_z;

		// m_nh.param("local_world_size_x", workspace_x, (float)2.0); // In meters
        // m_nh.param("local_world_size_y", workspace_y, (float)2.0); // In meters
        // m_nh.param("local_world_size_z", workspace_z, (float)1.0); // In meters
        // m_nh.param("resolution", m_resolution, (float)0.1);

		// maxX = (float)(2*workspace_x);
        // minX = (float)(0.0);
        // maxY = (float)(2*workspace_y);
        // minY = (float)(0.0);
        // maxZ = (float)(2*workspace_z);
        // minZ = (float)(0.0);

		// std::cout << "workspace_x: " << workspace_x  << std::endl;
		// std::cout << "maxX: " << maxX  << std::endl;

		// m_maxX = (float)(maxX-minX);
		// m_maxY = (float)(maxY-minY);
		// m_maxZ = (float)(maxZ-minZ);
		// // m_resolution = (float)res;
		// m_oneDivRes = 1.0/m_resolution;

		return false;
    }

	std::string bytes_to_hex_string(const std::vector<uint8_t>& bytes)
	{
	    std::ostringstream stream;
	    for (uint8_t b : bytes)
	    {
	        stream << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(b);
	    }
	    return stream.str();
	}
	//perform the SHA3-512 hash
	std::string sha3_512(const char * _input,int _size)
	{
	    uint32_t digest_length = SHA512_DIGEST_LENGTH;
	    const EVP_MD* algorithm = EVP_sha3_512();
	    uint8_t* digest = static_cast<uint8_t*>(OPENSSL_malloc(digest_length));
	    EVP_MD_CTX* context = EVP_MD_CTX_new();
	    EVP_DigestInit_ex(context, algorithm, nullptr);
	    EVP_DigestUpdate(context, _input, _size);
	    EVP_DigestFinal_ex(context, digest, &digest_length);
	    EVP_MD_CTX_destroy(context);
	    std::string output = bytes_to_hex_string(std::vector<uint8_t>(digest, digest + digest_length));
	    OPENSSL_free(digest);
	    return output;
	}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"	
#pragma GCC diagnostic pop
	
	void buildGridSliceMsg(float z)
	{
		static int seq = 0;
		
		// Setup grid msg
		m_gridSliceMsg.header.frame_id = m_globalFrameId;
		m_gridSliceMsg.header.stamp = ros::Time::now();
		m_gridSliceMsg.header.seq = seq++;
		m_gridSliceMsg.info.map_load_time = ros::Time::now();
		m_gridSliceMsg.info.resolution = m_resolution;
		m_gridSliceMsg.info.width = m_gridSizeX;
		m_gridSliceMsg.info.height = m_gridSizeY;
		m_gridSliceMsg.info.origin.position.x = 0.0;
		m_gridSliceMsg.info.origin.position.y = 0.0;
		m_gridSliceMsg.info.origin.position.z = z;
		m_gridSliceMsg.info.origin.orientation.x = 0.0;
		m_gridSliceMsg.info.origin.orientation.y = 0.0;
		m_gridSliceMsg.info.origin.orientation.z = 0.0;
		m_gridSliceMsg.info.origin.orientation.w = 1.0;
		m_gridSliceMsg.data.resize(static_cast<long>(m_gridSizeX)*m_gridSizeY);

		// Extract max probability
		int offset = (int)(z*m_oneDivRes)*m_gridSizeX*m_gridSizeY;
		int end = offset + m_gridSizeX*m_gridSizeY;
		float maxProb = -1.0;
		for(int i=offset; i<end; i++)
			if(m_grid[i].prob > maxProb)
				maxProb = m_grid[i].prob;

		// Copy data into grid msg and scale the probability to [0,100]
		if(maxProb < 0.000001)
			maxProb = 0.000001;
		maxProb = 100.0/maxProb;
		for(int i=0; i<m_gridSizeX*m_gridSizeY; i++)
			m_gridSliceMsg.data[i] = (int8_t)(m_grid[i+offset].prob*maxProb);
	}
	
	inline int point2grid(const float &x, const float &y, const float &z)
	{
		return (int)(x*m_oneDivRes) + (int)(y*m_oneDivRes)*m_gridStepY + (int)(z*m_oneDivRes)*m_gridStepZ;
	}
};


#endif
