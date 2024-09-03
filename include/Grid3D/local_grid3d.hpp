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
// #include "utils/ros/ROSInterfaces.hpp"

// #ifdef BUILD_VORONOI
// #include "voro++-0.4.6/src/voro++.hh"
// #endif

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
	float m_resolution, m_oneDivRes;
	octomap::OcTree *m_octomap;
	
	// 3D probabilistic grid cell
	Planners::utils::gridCell *m_grid;
	int m_gridSize, m_gridSizeX, m_gridSizeY, m_gridSizeZ;
	int m_gridStepY, m_gridStepZ;
	
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
	
	bool isIntoMap(float x, float y, float z)
	{
		return (x >= 0.0 && y >= 0.0 && z >= 0.0 && x < m_maxX && y < m_maxY && z < m_maxZ);
	}
	// int getCellCost(const float &_x, const float &_y, const float &_z){
	// JAC Precision
	float getCellCost(const float &_x, const float &_y, const float &_z){
		
		if( !isIntoMap(_x, _y, _z) ){
			std::cout << "OUT " << _x << "," << _y << "," << _z << "Expected: " << m_maxX << "," << m_maxY << "," << m_maxZ << std::endl;
			return 0;
		}
			

		int index = point2grid(_x, _y, _z);
		
		// return m_grid[index].prob;
		return m_grid[index].dist;
	}

	// // JAC - OLD VERSION
	// void computeLocalGrid(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &_cloud) //void
	// {
	// 	// unsigned t0, t1, t2, t3, t4, t5;        

	// 	//Publish percent variable
	// 	// std_msgs::Float32 percent_msg;
	// 	// percent_msg.data = 0;

	// 	// Alloc the 3D grid JAC: Alloc the size of the local grid
	// 	m_gridSizeX = (int)(m_maxX*m_oneDivRes);
	// 	m_gridSizeY = (int)(m_maxY*m_oneDivRes); 
	// 	m_gridSizeZ = (int)(m_maxZ*m_oneDivRes);
	// 	m_gridSize = m_gridSizeX*m_gridSizeY*m_gridSizeZ;
	// 	m_gridStepY = m_gridSizeX;
	// 	m_gridStepZ = m_gridSizeX*m_gridSizeY;

	// 	// JAC: Pongo los mismo datos que en global planner y no funciona
	// 	// m_gridSizeX = 341;
	// 	// m_gridSizeY = 241; 
	// 	// m_gridSizeZ = 101;
	// 	// m_gridSize = m_gridSizeX*m_gridSizeY*m_gridSizeZ;
	// 	// m_gridStepY = m_gridSizeX;
	// 	// m_gridStepZ = m_gridSizeX*m_gridSizeY;

	// 	// JAC: Alloc the 3D local Grid from input parameters. These parameters should be defined once at the beginning.
	// 	// release previously loaded data
	// 	if(m_grid != NULL)
	// 		delete []m_grid;
	// 	// t0 = clock();
	// 	m_grid = new Planners::utils::gridCell[m_gridSize]; // 2-3 milliseconds
	// 	// t1 = clock();
	// 	// double time = (double(t1-t0)/CLOCKS_PER_SEC);
    //     // std::cout << "Execution Time: " << time << std::endl;

	// 	// std::cout << "_maxX: " << m_maxX  << std::endl;
	// 	// std::cout << "_gridSizeX: " << m_gridSizeX  << std::endl;

	// 	// Setup kdtree
	// 	// std::vector<int> ind;
	// 	// pcl::removeNaNFromPointCloud(*_cloud, *filter_cloud, ind);  // JAC: Falla aqui
	// 	// std::cout << "no. of pts in input=" << _cloud->size() << std::endl;  //JAC: Confirmado que el pointcloud se pasa bien
	// 	// std::cout << "no. of pts in output="<< filter_cloud->size() << std::endl;
	// 	// t2 = clock();
	// 	m_kdtree.setInputCloud(_cloud);   //Less than 1 millisecond
	// 	// t3 = clock();
	// 	// double time2 = (double(t3-t2)/CLOCKS_PER_SEC);
    //     // std::cout << "Execution Time2: " << time2 << std::endl;
		
	// 	// Compute the distance to the closest point of the grid
	// 	int index;
	// 	float dist;
	// 	// float gaussConst1 = 1./(m_sensorDev*sqrt(2*M_PI));
	// 	// float gaussConst2 = 1./(2*m_sensorDev*m_sensorDev);
	// 	// pcl::PointXYZI searchPoint;
	// 	pcl::PointXYZ searchPoint;
	// 	std::vector<int> pointIdxNKNSearch(1);
	// 	std::vector<float> pointNKNSquaredDistance(1);
	// 	double count=0;
	// 	// double percent;
	// 	// double size=m_gridSizeX*m_gridSizeY*m_gridSizeZ;
	// 	// t4 = clock();
	// 	// JAC: 2-3 seconds
	// 	for(int iz=0; iz<m_gridSizeZ; iz++)
	// 	{
	// 		for(int iy=0; iy<m_gridSizeY; iy++)
	// 		{
	// 			for(int ix=0; ix<m_gridSizeX; ix++)
	// 			{
	// 				searchPoint.x = ix*m_resolution;
	// 				searchPoint.y = iy*m_resolution;
	// 				searchPoint.z = iz*m_resolution;
	// 				index = ix + iy*m_gridStepY + iz*m_gridStepZ;
	// 				++count;
	// 				// percent = count/size *100.0;
	// 				// ROS_INFO_THROTTLE(0.5,"Progress: %lf %%", percent);	
					
	// 				// if(percent > percent_msg.data + 0.5){
	// 				// 	percent_msg.data = percent;
	// 				// 	// percent_computed_pub_.publish(percent_msg);
	// 				// }

	// 				// JAC Error:Define correctly size of local map
	// 				// JAC: AQUI PETA CUANDO NO SE RECIBE POINTCLOUD
	// 				if(m_kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	// 				{	
	// 					dist = pointNKNSquaredDistance[0];
	// 					m_grid[index].dist = dist; // dist in the discrete world
	// 					// if(!use_costmap_function){
	// 					// 	m_grid[index].prob = gaussConst1*exp(-dist*dist*gaussConst2);
	// 					// }else{
	// 					// 	double prob =  100*exp(-cost_scaling_factor*std::fabs((dist - robot_radius)));
	// 					// 	// ROS_INFO("[%f, %f, %f] Dist: %f Probability: %f", searchPoint.x, searchPoint.y, searchPoint.z, dist, prob);
	// 					// 	//JAC: Include the computation of prob considering the distance to the nearest voronoi edge.
	// 					// 	m_grid[index].prob = prob;
	// 					// }
	// 				}
	// 				else
	// 				{
	// 					m_grid[index].dist = -1.0;
	// 					// m_grid[index].prob =  0.0;
	// 				}

	// 			}
	// 		}
	// 	}
	// 	// t5 = clock();
	// 	// double time3 = (double(t5-t4)/CLOCKS_PER_SEC);
    //     // std::cout << "Execution Time3: " << time3 << std::endl;

	// 	// percent_msg.data = 100;
	// 	// percent_computed_pub_.publish(percent_msg);
	// }

	// GUILLERMO - NEW VERSION WITH NN SDF
	void computeLocalGrid(torch::jit::script::Module& loaded_sdf, float drone_x, float drone_y, float drone_z) //void
	{
		// unsigned t0, t1, t2, t3, t4, t5;        

		//Publish percent variable
		// std_msgs::Float32 percent_msg;
		// percent_msg.data = 0;
		std::cout << "---!!!--- Entered computeLocalGrid ---!!!---" << std::endl;
		// Alloc the 3D grid JAC: Alloc the size of the local grid
		m_gridSizeX = (int)(m_maxX*m_oneDivRes);
		m_gridSizeY = (int)(m_maxY*m_oneDivRes); 
		m_gridSizeZ = (int)(m_maxZ*m_oneDivRes);
		m_gridSize = m_gridSizeX*m_gridSizeY*m_gridSizeZ;
		m_gridStepY = m_gridSizeX;
		m_gridStepZ = m_gridSizeX*m_gridSizeY;
		//std::cout << "m_gridSizeX = " << m_gridSizeX << "m_gridSizeY = " << m_gridSizeY << "m_gridSizeZ = " << m_gridSizeZ << std::endl;
		//std::cout << "m_gridSize = " << m_gridSize << std::endl;
		// JAC: Pongo los mismo datos que en global planner y no funciona
		// m_gridSizeX = 341;
		// m_gridSizeY = 241; 
		// m_gridSizeZ = 101;
		// m_gridSize = m_gridSizeX*m_gridSizeY*m_gridSizeZ;
		// m_gridStepY = m_gridSizeX;
		// m_gridStepZ = m_gridSizeX*m_gridSizeY;

		// JAC: Alloc the 3D local Grid from input parameters. These parameters should be defined once at the beginning.
		// release previously loaded data
		if(m_grid != NULL)
			delete []m_grid;
		// t0 = clock();
		m_grid = new Planners::utils::gridCell[m_gridSize]; // 2-3 milliseconds
		// t1 = clock();
		// double time = (double(t1-t0)/CLOCKS_PER_SEC);
        // std::cout << "Execution Time: " << time << std::endl;

		// std::cout << "_maxX: " << m_maxX  << std::endl;
		// std::cout << "_gridSizeX: " << m_gridSizeX  << std::endl;

		// Setup kdtree
		// std::vector<int> ind;
		// pcl::removeNaNFromPointCloud(*_cloud, *filter_cloud, ind);  // JAC: Falla aqui
		// std::cout << "no. of pts in input=" << _cloud->size() << std::endl;  //JAC: Confirmado que el pointcloud se pasa bien
		// std::cout << "no. of pts in output="<< filter_cloud->size() << std::endl;
		// t2 = clock();


		//Construir vector de posiciones globales para pasar a la red
		std::vector<std::vector<float>> coordinates_vector;
        coordinates_vector.reserve(m_gridSizeX*m_gridSizeY*m_gridSizeZ);

		//std::cout << "m_resolution = " << m_resolution << std::endl;
		std::cout << "Drone position: " << "x = " << drone_x << " | y = " << drone_y << " | z = " << drone_z << std::endl;

		for(int iz=0; iz<m_gridSizeZ; iz++)
		{
			for(int iy=0; iy<m_gridSizeY; iy++)
			{
				for(int ix=0; ix<m_gridSizeX; ix++)
				{
				float xc = drone_x + (ix - (m_gridSizeX -1) / 2) * m_resolution;
				float yc = drone_y + (iy - (m_gridSizeY -1) / 2) * m_resolution;
				float zc = drone_z + (iz - (m_gridSizeZ -1) / 2) * m_resolution;

				// Almacenar la coordenada (x, y, z) en la matriz
				coordinates_vector.push_back({xc, yc, zc});
				}
			}
		}

		//std::cout << "---!!!--- Completed coordinates_vector ---!!!---" << std::endl;

		// Convertir el vector de puntos a un tensor de libtorch
        auto num_points = coordinates_vector.size();
        torch::Tensor coordinates_tensor = torch::zeros({static_cast<long>(num_points), 3}, torch::kFloat);
        for (size_t i = 0; i < num_points; ++i) {
            coordinates_tensor[i][0] = coordinates_vector[i][0];
            coordinates_tensor[i][1] = coordinates_vector[i][1];
            coordinates_tensor[i][2] = coordinates_vector[i][2];
        }

		//std::cout << "---!!!--- Completed coordinates_tensor ---!!!---" << std::endl;
		//std::cout << coordinates_tensor << std::endl;

        // Pasar el tensor por la red neuronal
        auto start = std::chrono::high_resolution_clock::now();
		//std::cout << "---!!!--- Started timer ---!!!---" << std::endl;
        torch::Tensor grid_output_tensor = loaded_sdf.forward({coordinates_tensor}).toTensor();
		//std::cout << "---!!!--- Query done ---!!!---" << std::endl;
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start;
		// usleep(1e4);
        // std::cout << "Please a key to go to the next iteration..." << std::endl;
        // getchar(); // Comentar para no usar tecla.
        std::cout << "Points queried: " << num_points <<" |  Time taken to query model: " << duration.count() << " ms" << std::endl;

		//std::cout << "---!!!--- Completed query ---!!!---" << std::endl;

		int index;
		//Asignar cada valor a la posición correcta del grid
		for(int iz=0; iz<m_gridSizeZ; iz++)
		{
			for(int iy=0; iy<m_gridSizeY; iy++)
			{
				for(int ix=0; ix<m_gridSizeX; ix++)
				{
					index = ix + iy*m_gridStepY + iz*m_gridStepZ;
					m_grid[index].dist = grid_output_tensor[index].item<float>();
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
		double minX, minY, minZ, maxX, maxY, maxZ, res;
        
		maxX = 8.0;
        minX = 0.0;
        maxY = 8.0;
        minY = 0.0;
        maxZ = 4.0;
        minZ = 0.0;
        res = 0.2;

        // maxX = 4.0;
        // minX = 0.0;
        // maxY = 4.0;
        // minY = 0.0;
        // maxZ = 2.0;
        // minZ = 0.0;
        // res = 0.1;

		m_maxX = (float)(maxX-minX);
		m_maxY = (float)(maxY-minY);
		m_maxZ = (float)(maxZ-minZ);
		m_resolution = (float)res;
		m_oneDivRes = 1.0/m_resolution;


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
