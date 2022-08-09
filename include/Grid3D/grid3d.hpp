#ifndef __GRID3D_HPP__
#define __GRID3D_HPP__

/**
 * @file prob_map.cpp
 * @brief This file includes the ROS node implementation.
 * @author Francisco J. Perez Grau and Fernando Caballero
 */

// #include <ros/ros.h>
#include <chrono>
#include <pcl-1.10/pcl/impl/point_types.hpp>
#include <pcl-1.10/pcl/kdtree/kdtree_flann.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <stdio.h> 

// PCL
#include <sys/time.h>
// #include <pcl_ros/point_cloud.hpp>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

#include "utils/utils.hpp"

#include <sstream> //for std::ostringstream
#include <fstream>
#include <iomanip> //for std::setw, std::hex, and std::setfill
#include <openssl/evp.h> //for all other OpenSSL function calls
#include <openssl/sha.h> //for SHA512_DIGEST_LENGTH
// #include "utils/ros/ROSInterfaces.hpp"

// #ifdef BUILD_VORONOI
// #include "voro++-0.4.6/src/voro++.hh"
// #endif

class Grid3d {
private:
  rclcpp::Node* node_ptr_;
	
	// Ros parameters
	// ros::NodeHandle m_nh;
	bool m_publishPc;
  
	std::string m_mapPath, m_nodeName;
	std::string m_globalFrameId;
	float m_sensorDev, m_gridSlice;
	double m_publishPointCloudRate, m_publishGridSliceRate;
	
	// Octomap parameters
	float m_maxX, m_maxY, m_maxZ;
	float m_resolution, m_oneDivRes;
	octomap::OcTree *m_octomap;
	
	// 3D probabilistic grid cell
	Planners::utils::gridCell *m_grid;
	int m_gridSize, m_gridSizeX, m_gridSizeY, m_gridSizeZ;
	int m_gridStepY, m_gridStepZ;
	
	// 3D point clound representation of the map
  	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> m_cloud;
	pcl::KdTreeFLANN<pcl::PointXYZI> m_kdtree;
	
	// Visualization of the map as pointcloud
	sensor_msgs::msg::PointCloud2 m_pcMsg;
  	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pcPub;
  	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr percent_computed_pub_;
  	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_gridSlicePub;

	// ros::Timer mapTimer;
  	rclcpp::TimerBase::SharedPtr mapTimer;
			
	// Visualization of a grid slice as 2D grid map msg
	nav_msgs::msg::OccupancyGrid m_gridSliceMsg;
	// ros::Timer gridTimer;
  	rclcpp::TimerBase::SharedPtr gridTimer;

	//Parameters added to allow a new exp function to test different gridmaps
	double cost_scaling_factor, robot_radius;
	bool use_costmap_function;
	
public:
	Grid3d(rclcpp::Node* node_ptr):node_ptr_(node_ptr), m_cloud(new pcl::PointCloud<pcl::PointXYZI>)
	{
    

    node_ptr_->declare_parameter<std::string>("name", "grid3d");
    node_ptr_->declare_parameter<std::string>("global_frame_id", "map");
    node_ptr_->declare_parameter<std::string>("map_path", "map.ot");
    node_ptr_->declare_parameter<bool>("publish_point_cloud", true);
    node_ptr_->declare_parameter<double>("publish_point_cloud_rate", 0.2);
    node_ptr_->declare_parameter<double>("publish_grid_slice", 1.0);
    node_ptr_->declare_parameter<double>("publish_grid_slice_rate", 0.2);
    node_ptr_->declare_parameter<double>("sensor_dev", 0.2);
    // node_ptr_->declare_parameter<double>("cost_scaling_factor", 0.8);
    // node_ptr_->declare_parameter<double>("robot_radius", 0.4);
    node_ptr_->declare_parameter<bool>("use_costmap_function", true);

    node_ptr_->get_parameter("name", m_nodeName);
    node_ptr_->get_parameter("global_frame_id", m_globalFrameId);
    node_ptr_->get_parameter("map_path", m_mapPath);
    node_ptr_->get_parameter("publish_point_cloud", m_publishPc);
    node_ptr_->get_parameter("publish_point_cloud_rate", m_publishPointCloudRate);
    node_ptr_->get_parameter("publish_grid_slice", m_gridSlice);
    node_ptr_->get_parameter("publish_grid_slice_rate", m_publishGridSliceRate);
    node_ptr_->get_parameter("sensor_dev", m_sensorDev);
    node_ptr_->get_parameter("cost_scaling_factor", cost_scaling_factor);
    node_ptr_->get_parameter("robot_radius", robot_radius);
    node_ptr_->get_parameter("use_costmap_function", use_costmap_function);

		// Load octomap 
		m_octomap = NULL;
		m_grid = NULL;

    RCLCPP_WARN(node_ptr_->get_logger(), "Loading octomap from %s", m_mapPath.c_str());
		if(loadOctomap(m_mapPath))
		{
			// Compute the point-cloud associated to the ocotmap
			computePointCloud();
			
			// Try to load tha associated grid-map from file
			std::string path;
			if(m_mapPath.compare(m_mapPath.length()-3, 3, ".bt") == 0)
				path = m_mapPath.substr(0,m_mapPath.find(".bt"))+".gridm";
			if(m_mapPath.compare(m_mapPath.length()-3, 3, ".ot") == 0)
				path = m_mapPath.substr(0,m_mapPath.find(".ot"))+".gridm";
			if(!loadGrid(path))
			{						
				// Compute the gridMap using kdtree search over the point-cloud
				std::cout << "Computing 3D occupancy grid. This will take some time..." << std::endl;
				computeGrid();
				std::cout << "\tdone!" << std::endl;
				
				// Save grid on file
				if(saveGrid(path))
					std::cout << "Grid map successfully saved on " << path << std::endl;
			}
			
			// Build the msg with a slice of the grid if needed
			if(m_gridSlice >= 0 && m_gridSlice <= m_maxZ)
			{
				buildGridSliceMsg(m_gridSlice);
        		m_gridSlicePub = node_ptr_->create_publisher<nav_msgs::msg::OccupancyGrid>(m_nodeName+"/grid_slice", 1);

        		// gridTimer = node_ptr_->create_wall_timer(
            	// std::chrono::seconds(1.0f/m_publishGridSliceRate), [this]() { node_ptr_->publishGridSlice();});
            	// FIXME:
        		gridTimer = node_ptr_->create_wall_timer(
            	std::chrono::seconds(1), [this]() {this->publishGridSlice();});

			}
			
			// Setup point-cloud publisher
			if(m_publishPc)
			{
				// m_pcPub  = m_nh.advertise<sensor_msgs::msg::PointCloud2>(m_nodeName+"/map_point_cloud", 1, true);
        		m_pcPub = node_ptr_-> create_publisher<sensor_msgs::msg::PointCloud2>(m_nodeName+"/map_point_cloud", 1);
        		
				// mapTimer = node_ptr_->create_wall_timer(
        		//     std::chrono::millisecondsseconds(1.0/m_publishPointCloudRate), [this]() { node_ptr_-> publishMapPointCloud();});
        		//     FIXME:
        		// mapTimer = node_ptr_->create_wall_timer(
            	// std::chrono::seconds(1), [this]() { node_ptr_-> publishMapPointCloud();});	
				//COMPILE			
				// mapTimer = node_ptr_->create_wall_timer(
            	// std::chrono::milliseconds(500), std::bind(&Grid3d::publishMapPointCloud, this));
				
				mapTimer = node_ptr_->create_wall_timer(std::chrono::duration<double>(1.0f/1),std::bind(&Grid3d::publishMapPointCloud, this));
			}
      		percent_computed_pub_ = node_ptr_->create_publisher<std_msgs::msg::Float32>(m_nodeName+"/percent_computed", 10);

		}
	}

	~Grid3d(void)
	{
		if(m_octomap != NULL)
			delete m_octomap;
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
	float computeCloudWeight(std::vector<pcl::PointXYZI> &points)
	{
		float weight = 0.;
		int n = 0;

		for(long unsigned int i=0; i<points.size(); i++)
		{
			const pcl::PointXYZI& p = points[i];
			if(p.x >= 0.0 && p.y >= 0.0 && p.z >= 0.0 && p.x < m_maxX && p.y < m_maxY && p.z < m_maxZ)
			{
				int index = point2grid(p.x, p.y, p.z);
				weight += m_grid[index].prob;
				n++;
			}
		}

		if(n > 10)
			return weight/n;
		else
			return 0;
	}
  
	void publishMapPointCloud(void)
	{
		m_pcMsg.header.stamp = node_ptr_->now();
		m_pcPub->publish(m_pcMsg);
	}
	
	void publishGridSlice(void)
	{
		m_gridSliceMsg.header.stamp = node_ptr_->now();
		m_gridSlicePub->publish(m_gridSliceMsg);
	}
	
	bool isIntoMap(float x, float y, float z)
	{
		return (x >= 0.0 && y >= 0.0 && z >= 0.0 && x < m_maxX && y < m_maxY && z < m_maxZ);
	}
	// int getCellCost(const float &_x, const float &_y, const float &_z){
	// JAC Precision
	float getCellCost(const float &_x, const float &_y, const float &_z){
		
		if( !isIntoMap(_x, _y, _z) )
			return 0;

		int index = point2grid(_x, _y, _z);
		
		return m_grid[index].prob;
	}
	
	std::pair<Planners::utils::Vec3i, double>  getClosestObstacle(const Planners::utils::Vec3i& _coords){

		pcl::PointXYZI searchPoint;

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
	// void publishMapPointCloudTimer(const ros::TimerEvent& event)
	// {
	// 	publishMapPointCloud();
	// }
	// void publishGridSliceTimer(const ros::TimerEvent& event)
	// {
	// 	publishGridSlice();
	// }
#pragma GCC diagnostic pop

	bool loadOctomap(std::string &path)
	{
		// release previously loaded data
		if(m_octomap != NULL)
			delete m_octomap;
		if(m_grid != NULL)
			delete []m_grid;

		// Load octomap
		octomap::AbstractOcTree *tree;
		if(path.length() > 3 && (path.compare(path.length()-3, 3, ".bt") == 0))
		{
			octomap::OcTree* binaryTree = new octomap::OcTree(0.1);
			if (binaryTree->readBinary(path) && binaryTree->size() > 1)
				tree = binaryTree;
			else 
				return false;
		} 
		else if(path.length() > 3 && (path.compare(path.length()-3, 3, ".ot") == 0))
		{
			tree = octomap::AbstractOcTree::read(path);
			if(!tree){
				return false;
			}
		}	
		else
			return false;
		/*
		// Load octomap
		octomap::AbstractOcTree *tree = octomap::AbstractOcTree::read(path);
		if(!tree)
			return false;*/
		m_octomap = dynamic_cast<octomap::OcTree*>(tree);
		std::cout << "Octomap loaded" << std::endl;

		if(m_octomap == NULL)
		{
			std::cout << "Error: NULL octomap!!" << std::endl;
			return false;
		}
		
		// Get map parameters
		double minX, minY, minZ, maxX, maxY, maxZ, res;
		m_octomap->getMetricMin(minX, minY, minZ);
		m_octomap->getMetricMax(maxX, maxY, maxZ);
		res = m_octomap->getResolution();
		m_maxX = (float)(maxX-minX);
		m_maxY = (float)(maxY-minY);
		m_maxZ = (float)(maxZ-minZ);
		m_resolution = (float)res;
		m_oneDivRes = 1.0/m_resolution;
		RCLCPP_INFO(node_ptr_->get_logger(),"Map size");
		RCLCPP_INFO(node_ptr_->get_logger(),"\tx: %.2f to %.2f", minX, maxX);
		RCLCPP_INFO(node_ptr_->get_logger(),"\ty: %.2f to %.2f", minY, maxY);
		RCLCPP_INFO(node_ptr_->get_logger(),"\tz: %.2f to %.2f", minZ, maxZ);
		RCLCPP_INFO(node_ptr_->get_logger(),"\tRes: %.2f", m_resolution);

		// ROS_INFO("Map size:\n");
		// ROS_INFO("\tx: %.2f to %.2f", minX, maxX);
		// ROS_INFO("\ty: %.2f to %.2f", minZ, maxZ);
		// ROS_INFO("\tz: %.2f to %.2f", minZ, maxZ);
		// ROS_INFO("\tRes: %.2f" , m_resolution );

		
		
		return true;
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
	bool saveGrid(std::string &fileName)
	{
		FILE *pf;
		
		// Open file
		pf = fopen(fileName.c_str(), "wb");
		if(pf == NULL)
		{
			std::cout << "Error opening file " << fileName << " for writing" << std::endl;
			return false;
		}
		
		// Write grid general info 
		fwrite(&m_gridSize, sizeof(int), 1, pf);
		fwrite(&m_gridSizeX, sizeof(int), 1, pf);
		fwrite(&m_gridSizeY, sizeof(int), 1, pf);
		fwrite(&m_gridSizeZ, sizeof(int), 1, pf);
		fwrite(&m_sensorDev, sizeof(float), 1, pf);
		
		// Write grid cells
		fwrite(m_grid, sizeof(Planners::utils::gridCell), m_gridSize, pf);
		
		auto sha_value = sha3_512((const char*)m_grid, m_gridSize);
		std::cout << "Sha512 value: " << sha_value << std::endl;
		std::ofstream sha_file;
		sha_file.open(fileName+"sha", std::ofstream::trunc);
		sha_file << sha_value; 
		sha_file.close();
		// Close file
		fclose(pf);
		
		return true;
	}


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"	
	bool loadGrid(std::string &fileName)
	{
		FILE *pf;
		
		// Open file
		pf = fopen(fileName.c_str(), "rb");
		if(pf == NULL)
		{
			std::cout << "Error opening file " << fileName << " for reading" << std::endl;
			return false;
		}
		
		// Write grid general info 
		fread(&m_gridSize, sizeof(int), 1, pf);
		fread(&m_gridSizeX, sizeof(int), 1, pf);
		fread(&m_gridSizeY, sizeof(int), 1, pf);
		fread(&m_gridSizeZ, sizeof(int), 1, pf);
		fread(&m_sensorDev, sizeof(float), 1, pf);
		m_gridStepY = m_gridSizeX;
		m_gridStepZ = m_gridSizeX*m_gridSizeY;
		
		// Write grid cells
		if(m_grid != NULL)
			delete []m_grid;
		m_grid = new Planners::utils::gridCell[m_gridSize];
		fread(m_grid, sizeof(Planners::utils::gridCell), m_gridSize, pf);
		fclose(pf);
		
		std::ifstream input_sha;
		auto sha_value = sha3_512((const char*)m_grid, m_gridSize);
		RCLCPP_INFO(node_ptr_->get_logger(),"[Grid3D] Calculated SHA512 value: %s", sha_value.c_str());
		// ROS_INFO("[Grid3D] Calculated SHA512 value: %s", sha_value.c_str());

		input_sha.open(fileName+"sha");
		std::string readed_sha;
		if(input_sha.is_open()){
			getline(input_sha, readed_sha);
			RCLCPP_INFO(node_ptr_->get_logger(),"[Grid3D] Readed SHA512 from file: %s", readed_sha.c_str());
			// ROS_INFO("[Grid3D] Readed SHA512 from file: %s", readed_sha.c_str());
			input_sha.close();
		}else{
			// ROS_ERROR("[Grid3D] Couldn't read SHA512 data of gridm file, recomputing grid");
			RCLCPP_INFO(node_ptr_->get_logger(),"[Grid3D] Couldn't read SHA512 data of gridm file, recomputing grid");
			return false;
		}
		//Check that both are the same
		if(readed_sha.compare(sha_value) != 0){
			// ROS_ERROR("[Grid3D] Found different SHA values between for gridm!");
			RCLCPP_INFO(node_ptr_->get_logger(),"[Grid3D] Found different SHA values between for gridm!");
			return false;
		}
		
		return true;
	}
#pragma GCC diagnostic pop
	
	void computePointCloud(void)
	{
		// Get map parameters
		double minX, minY, minZ;
		m_octomap->getMetricMin(minX, minY, minZ);
		
		// Load the octomap in PCL for easy nearest neighborhood computation
		// The point-cloud is shifted to have (0,0,0) as min values
		int i = 0;
		m_cloud->width = m_octomap->size();
		m_cloud->height = 1;
		m_cloud->points.resize(static_cast<long>(m_cloud->width) * m_cloud->height);
		for(octomap::OcTree::leaf_iterator it = m_octomap->begin_leafs(), end = m_octomap->end_leafs(); it != end; ++it)
		{
			if(it != NULL && m_octomap->isNodeOccupied(*it))
			{
				m_cloud->points[i].x = it.getX()-minX;
				m_cloud->points[i].y = it.getY()-minY;
				m_cloud->points[i].z = it.getZ()-minZ;
				
				i++;
			}
		}
		m_cloud->width = i;
		m_cloud->points.resize(i);
		
		// Create the point cloud msg for publication
		pcl::toROSMsg(*m_cloud, m_pcMsg);
		m_pcMsg.header.frame_id = m_globalFrameId;
	}

	void computeGrid(void)
	{
		//Publish percent variable
		std_msgs::msg::Float32 percent_msg;
		percent_msg.data = 0;
		// Alloc the 3D grid
		m_gridSizeX = (int)(m_maxX*m_oneDivRes);
		m_gridSizeY = (int)(m_maxY*m_oneDivRes); 
		m_gridSizeZ = (int)(m_maxZ*m_oneDivRes);
		m_gridSize = m_gridSizeX*m_gridSizeY*m_gridSizeZ;
		m_gridStepY = m_gridSizeX;
		m_gridStepZ = m_gridSizeX*m_gridSizeY;
		m_grid = new Planners::utils::gridCell[m_gridSize];

		// Setup kdtree
		m_kdtree.setInputCloud(m_cloud);

		// Compute the distance to the closest point of the grid
		int index;
		float dist;
		float gaussConst1 = 1./(m_sensorDev*sqrt(2*M_PI));
		float gaussConst2 = 1./(2*m_sensorDev*m_sensorDev);
		pcl::PointXYZI searchPoint;
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);
		double count=0;
		double percent;
		double size=m_gridSizeX*m_gridSizeY*m_gridSizeZ;
		for(int iz=0; iz<m_gridSizeZ; iz++)
		{
			for(int iy=0; iy<m_gridSizeY; iy++)
			{
				for(int ix=0; ix<m_gridSizeX; ix++)
				{
					searchPoint.x = ix*m_resolution;
					searchPoint.y = iy*m_resolution;
					searchPoint.z = iz*m_resolution;
					index = ix + iy*m_gridStepY + iz*m_gridStepZ;
					++count;
					percent = count/size *100.0;
					// ROS_INFO_THROTTLE(0.5,"Progress: %lf %%", percent);	
					if(percent > percent_msg.data + 0.5){
						percent_msg.data = percent;
						// percent_computed_pub_.publish(percent_msg);
					}
					
					if(m_kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
					{
						dist = pointNKNSquaredDistance[0];
						m_grid[index].dist = dist;
						if(!use_costmap_function){
							m_grid[index].prob = gaussConst1*exp(-dist*dist*gaussConst2);
						}else{
							double prob =  100*exp(-cost_scaling_factor*std::fabs((dist - robot_radius)));
							// ROS_INFO("[%f, %f, %f] Dist: %f Probability: %f", searchPoint.x, searchPoint.y, searchPoint.z, dist, prob);
							//JAC: Include the computation of prob considering the distance to the nearest voronoi edge.
							m_grid[index].prob = prob;
						}
					}
					else
					{
						m_grid[index].dist = -1.0;
						m_grid[index].prob =  0.0;
					}

				}
			}
		}
		percent_msg.data = 100;
		// percent_computed_pub_->publish(percent_msg);
	}
	
	void buildGridSliceMsg(float z)
	{
		// static int _seq = 0;
		
		// Setup grid msg
		m_gridSliceMsg.header.frame_id = m_globalFrameId;
		m_gridSliceMsg.header.stamp = node_ptr_->now();

    // TODO: add seq
		// m_gridSliceMsg.header.seq = _seq++;
		m_gridSliceMsg.info.map_load_time = node_ptr_->now();
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
