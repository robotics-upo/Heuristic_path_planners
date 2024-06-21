#ifndef ASTAR_HPP
#define ASTAR_HPP
/**
 * @file AStar.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
 * @author Jose Antonio Cobano (jacobsua@upo.es)
 * @brief 
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Planners/AlgorithmBase.hpp>


/**
 * @brief
 * This Header includes some auxiliar ROS features
 * that the child classes as Lazy Theta* and so on will inherit
 * These ROS Debug helps the user analyzing the inner 
 * behavior of the algorithm, allowing step-by-step node processing 
 * and visualizing the open and closed set in RViz.
 * 
 */
#ifdef ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <chrono>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include "utils/ros/ROSInterfaces.hpp"
#include "utils/FCNet.hpp"
#include <torch/script.h>
#endif

namespace Planners{

    /**
     * @brief 
     * 
     */
    class AStar : public AlgorithmBase
    {
        
    public:
        /**
         * @brief Construct a new AStar object
         * @param _use_3d This parameter allows the user to choose between 
         * planning on a plane (8 directions possibles) or in the 3D full space (26 directions)
         * 
         * @param _name Algorithm name stored internally
         * 
         */
        AStar(bool _use_3d, std::string _name);
        /**
         * @brief Construct a new AStar object
         * 
         * @param _use_3d 
         */
        AStar(bool _use_3d);
        /**
         * @brief Main function of the algorithm
         * 
         * @param _source Start discrete coordinates. It should be a valid coordinates, i.e. it should not
         * be marked as occupied and it should be inside the configured workspace.
         * @param _target Goal discrete coordinates. It should be a valid coordinates, i.e. it should not
         * be marked as occupied and it should be inside the configured workspace.
         * @return PathData PathData Results stored as PathData object
         * Reminder: 
         * PathData    = std::map<std::string, Planners::utils::DataVariant> 
         * DataVariant = std::variant<std::string, Vec3i, CoordinateList, double, size_t, int, bool, unsigned int>;
         * TODO: Replace map here by unordered_map. Not much important, but it does not make sense to use a map.
         */
        PathData findPath(const Vec3i &_source, const Vec3i &_target, torch::jit::script::Module& loaded_sdf) override;
        
        /**
         * @brief Published occupation markers map to visualize the loaded map in RVIZ
         * if the package is compiled without the ROS definition in the CMakeLists, this function will
         * be empty. It can help the user to verify if the inflation parameters produced the desired result.
         */
        void publishOccupationMarkersMap() override;
        
        /**
         * @brief For this function to be compiled you should
         * enable in CMakeLists all the ROS Related options (ROS and PUB_EXPLORED_NODES macros)
         * 
         * Note: If you want to do STEP BY STEP evaluation, uncomment the 
         * getchar(); line at the end of this function, and you will be able
         * to advance in the exploration just pressing a key.
         * 
         * @param _node The current node the algorithm is evaluating. Will be marked as a
         * sphere to distinguish it between the open set and closed set markers
         * @param _open_set: The closed set container
         * @param _closed_set: The open set container
         * TODO: Remove these sets parameter as they are no longer in the findPath function scope
         * (They are currently member functions of the class).
         */
        template<typename T, typename U>
        void publishROSDebugData(const Node* _node, const T &_open_set, const U &_closed_set);
        
    protected:

        /**
         * @brief This function is called by the constructor. 
         * It reserves 50000 nodes in the closed and openset to avoid container 
         * resizing operations. 
         * TODO: Change 50.000 by the number of total nodes in the discrete world?
         * If ROS macro is defined through CMake, it will also configure the markers
         * publishers and the markers objects
         */
        void configAlgorithm();

        /**
         * @brief This function is the secondary inside the main loop of findPath 
         * function. This secondary loop iterates over the neighbours of a node, 
         * skipping the explored or occupied ones, and performs the appropiate operations
         * and calculus of the H and G values, and the corresponding parents updates.
         * 
         * @param _current A pointer to the current node, the loop will iterate over
         * the neighbours of this node by using the directions vector in the AlgorithmBase
         * class
         * @param _target A reference to the target coordinates of the GOAL.
         * 
         * @param _index_by_pos A reference to the openset to insert the non-explored nodes 
         * or to erase and insert the re-explored ones if a better path is found. 
         * This operation of erase and re-insert is performed in order to update the position
         * of the node in the container. 
         */
        virtual void exploreNeighbours(Node* _current, const Vec3i &_target,node_by_position &_index_by_pos, torch::jit::script::Module& loaded_sdf);

        /**
         * @brief This functions implements the algorithm G function. 
         * 
         * Some efforts have been made to try to improve this function as it
         * one of the most called methods along with exploreNeighbours. 
         * For exmaple, we tried to avoid the if statement to reduce the number
         * of branches by ussing boolean masks, but it didn't show any
         * significative performance improvement, so for the seek of clarity and 
         * maintenance it's preferable to keep the if structure by the moment.
         * 
         * Example of using masks:
         * 
         *      unsigned int totalCost = _current->G;
         * 
         *      bool maskdd2d = ( using_2d_ && i >= 4 ) || ( !using_2d_ && i > 6 && i < 18);
         *      bool maskdd3d = ( !using_2d_ && i > 18 );
         *
         *      totalCost += dist_scale_factor_;
         *      totalCost += maskdd2d * dd_2D_f + maskdd3d * dd_3D_f;
         * here dd_2D_f = 0.41 (sqrt(2) - 1) and dd_3D_f = 0.3 (sqrt(3) - sqrt(2) )
         * 
         * The current implementation is also more efficient than
         * directly doing 
         * 
         *      unsigned int totalCost = _current->G + heuristic(_current->coordinates, newCoordinates);
         * 
         * 
         * @param _current Pointer to the current node
         * @param _suc Pointer to the successor node
         * @param _n_i The index of the direction in the directions vector. 
         * Depending on this index, the distance wi
         * @param _dirs Number of directions used (to distinguish between 2D and 3D)
         * @return unsigned int The G Value calculated by the function
         */
        virtual unsigned int computeG(const Node* _current, Node* _suc, unsigned int _n_i, unsigned int _dirs, torch::jit::script::Module& loaded_sdf);

        unsigned int line_of_sight_checks_{0};  /*!< TODO Comment */
        std::vector<Node*> closedSet_; /*!< TODO Comment */
        MagicalMultiSet openSet_; /*!< TODO Comment */
        
#ifdef ROS
        ros::NodeHandle lnh_{"~"}; /*!< TODO Comment */
        ros::Publisher explored_nodes_marker_pub_, occupancy_marker_pub_,  /*!< TODO Comment */
                       openset_marker_pub_, closedset_marker_pub_, /*!< TODO Comment */
                       best_node_marker_pub_, aux_text_marker_pub_; /*!< TODO Comment */
        visualization_msgs::Marker explored_node_marker_, openset_markers_,  /*!< TODO Comment */
                                   closed_set_markers_, best_node_marker_, aux_text_marker_; /*!< TODO Comment */
        ros::Duration duration_pub_{0.001}; /*!< TODO Comment */
        ros::Time last_publish_tamp_; /*!< TODO Comment */
        float resolution_; /*!< TODO Comment */
    	pcl::PointCloud<pcl::PointXYZ>  occupancy_marker_;  /*!< TODO Comment */

#endif
    };

}

#endif 
