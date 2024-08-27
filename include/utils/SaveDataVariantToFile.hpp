#ifndef SAVEDATAVARIANTTOFILE_HPP
#define SAVEDATAVARIANTTOFILE_HPP

/**
 * @file SaveDataVariantToFile.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com )
 * @brief  Save std::map<std::string, custom_varian> to file
 * @version 0.1
 * @date 2021-06-23
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <fstream>
#include <algorithm>
#include <numeric>
#include "utils/utils.hpp"

/* Test for GCC > 9.0.0 */
#if __GNUC__ >= 9 
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif
namespace Planners
{
    namespace utils
    {
        /**
         * @brief Deduction guide 
         * 
         * @tparam Ts 
         */
        template <class... Ts>
        struct overloaded : Ts...
        {
            using Ts::operator()...;
        };

        template <class... Ts>
        overloaded(Ts...) -> overloaded<Ts...>;
        
        /**
         * @brief This class allow to easily save information stored as "PathData" (std::map<std::string, DataVariant>) into files 
         * 
         */
        class DataVariantSaver
        {

        public:

            /**
             * @brief Construct a new Data Variant Saver object
             * 
             * @param _fields A std::vector<std::string> of fields to look for on the input map. The default values 
             * are: algorithm, goal_coords, start_coords, time_spent, explored_nodes, path_length, line_of_sight_checks and solved
             */
            DataVariantSaver(const std::vector<std::string> &_fields =
                            {"algorithm", "goal_coords", "start_coords", "time_spent",
                             "explored_nodes", "path_length", "total_cost1", "total_cost2", "h_cost", "g_cost1", "g_cost2", "c_cost", "grid_cost1", "grid_cost2", "g_final_node",
                             "line_of_sight_checks", "min_dist", "max_dist", "mean_dist", "std_dev",
                             "solved", "cost_weight","max_line_of_sight_cells", "av_curv", "std_dev_curv", "min_curv", "max_curv", 
                             "av_angles", "std_dev_angles", "min_angle", "max_angle", "angle_changes"}): fields_(_fields)
            {

            }
            /**
             * @brief Main function that reads the incoming pathdata object and save to file in append mode
             * 
             * @param _data pathdata input object 
             * @param _file_path
             * @return true Always returns true at this version
             * @return false Never returns false right now
             */
            bool savePathDataToFile(const PathData &_data, const std::string &_file_path)
            {
                if( ! fs::exists(_file_path) ){ //If file does not exist, write a header with field names
                    std::cout << "File does not exists. Creating header at first line" << std::endl;   
                    out_file_data_.open(_file_path, std::ofstream::app);
                    out_file_data_ << fields_ << std::endl;
                }else{
                    out_file_data_.open(_file_path, std::ofstream::app);
                }
                for (auto &it : fields_)
                {
                    auto field = _data.find(it);
                    if (field == _data.end())
                    {
                        out_file_data_ << " , ";
                        continue;
                    }

                    std::visit(overloaded{
                            [this](auto arg) { out_file_data_ << arg << ", "; }, },
                            field->second);
                }
                out_file_data_ << std::endl;
                out_file_data_.close();


                return true;
            }
            /**
             * @brief Main function that reads the incoming path coords and saves them to file in append mode
             * 
             * @param _data pathdata input object 
             * @param _file_path
             * @return true Always returns true at this version
             * @return false Never returns false right now
             */
            bool savePathCoordsToFile(const PathData &_data, const std::string &_file_path)
            {
                if( ! fs::exists(_file_path) ){ //If file does not exist, write a header with field names
                    std::cout << "File does not exists. Creating header at first line" << std::endl;   
                    out_file_data_.open(_file_path, std::ofstream::app);
                    //out_file_data_ << "path" << std::endl;
                }else{
                    out_file_data_.open(_file_path, std::ofstream::app);
                }
                auto field = _data.find("path");
                if (field != _data.end()) {
                    std::visit(overloaded{
                        [this](auto arg) { out_file_data_ << arg << ", "; },
                    }, field->second);
                }else{
                    out_file_data_ << " , ";
                }
                out_file_data_ << std::endl;
                out_file_data_.close();


                return true;
            }
            /**
             * @brief 
             * 
             * @param _path 
             * @param _results 
             * @param _file_path 
             * @return true 
             * @return false 
             */
            bool savePathDistancesToFile(const utils::CoordinateList &_path,
                                         const std::vector<std::pair<utils::Vec3i, double>> &_results,
                                         const std::string &_file_path){
                
                out_file_data_.open(_file_path, std::ofstream::app);

                if( _path.size() != _results.size() )
                    return false;
                
                for(size_t i = 0; i < _results.size() -1 ; ++i)
                    out_file_data_ << _results[i].second << ", ";

                out_file_data_ << _results[static_cast<size_t>(_results.size() -1)].second << std::endl;;

                out_file_data_.close();

                return true;
            }
            /**
             * @brief 
             * 
             * @param _angles 
             * @param _file_path 
             * @return true 
             * @return false 
             */
            bool saveAnglesToFile(const std::vector<double> &_angles, 
                                  const std::string &_file_path){

                out_file_data_.open(_file_path, std::ofstream::app);
                if ( _angles.size() == 0 )
                    return true;
                
                for(size_t i = 0; i < _angles.size() - 1; ++i)
                    out_file_data_ << _angles[i] << ", ";

                out_file_data_ << _angles[static_cast<size_t>(_angles.size() - 1)] << std::endl;

                out_file_data_.close();

                return true;    
            }

        private:
            std::ofstream out_file_data_;
            std::vector<std::string> fields_;
            std::string data_file_;
        };
    } // utils ns
} //planners ns

#endif