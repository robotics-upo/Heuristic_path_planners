#ifndef SAVEDATATOFILE_HPP
#define SAVEDATATOFILE_HPP
/**
 * @file SaveDataAnyToFile.hpp
 * @author Rafael Rey (rreyarc@upo.es)
 * @brief Old class used to save std::map<std::string, std::any> fields to file. It is not used 
 * anymore but not deleted because maybe in the future can be reused
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <iostream>
#include <fstream>
#include <any>
#include "utils/utils.hpp"

namespace Planners
{
    namespace utils
    {   
        /**
         * @brief Old class used to store std any map as file
         * 
         */
        class DataAnySaver
        {

        public:
            /**
             * @brief Construct a new Data Saver object
             * 
             * @param _data_file path of the file to append the data 
             */
            DataSaver(const std::string &_data_file)
            {
                out_file_data_.open(_data_file, std::ofstream::app);
            }
            /**
             * @brief 
             * 
             * @param _data pathdata 
             * @return true 
             * @return false 
             */
            bool savePathDataToFile(const PathData &_data)
            {
                std::string field;
                bool res = true;
                try
                {
                    field = "algorithm";
                    if (_data.find(field) != _data.end())
                    {
                        auto algorithm_name = std::any_cast<std::string>(_data.at(field));
                        out_file_data_ << algorithm_name << ", ";
                    }
                    else
                    {
                        out_file_data_ << ", ";
                        std::cerr << "Could not save field: " << field << " does not exist" << std::endl;
                    }
                    field = "goal_coords";
                    if (_data.find(field) != _data.end())
                    {
                        auto goal_vec = std::any_cast<utils::Vec3i>(_data.at(field));
                        out_file_data_ << goal_vec << ", ";
                    }
                    else
                    {
                        out_file_data_ << ", ";
                        std::cerr << "Could not save field: " << field << " does not exist" << std::endl;
                    }
                    field = "start_coords";
                    if (_data.find(field) != _data.end())
                    {
                        auto start_vec = std::any_cast<utils::Vec3i>(_data.at(field));
                        out_file_data_ << start_vec << ", ";
                    }
                    else
                    {
                        out_file_data_ << ", ";
                        std::cerr << "Could not save field: " << field << " does not exist" << std::endl;
                    }
                    field = "time_spent";
                    if (_data.find(field) != _data.end())
                    {
                        auto time_spent = std::any_cast<double>(_data.at(field));
                        out_file_data_ << time_spent << ", ";
                    }
                    else
                    {
                        out_file_data_ << ", ";
                        std::cerr << "Could not save field: " << field << " does not exist" << std::endl;
                    }
                    field = "explored_nodes";
                    if (_data.find(field) != _data.end())
                    {
                        auto explored_nodes = std::any_cast<size_t>(_data.at(field));
                        out_file_data_ << explored_nodes << ", ";
                    }
                    else
                    {
                        out_file_data_ << ", ";
                        std::cerr << "Could not save field: " << field << " does not exist" << std::endl;
                    }
                    field = "path_length";
                    if (_data.find(field) != _data.end())
                    {
                        auto path_length = std::any_cast<float>(_data.at(field));
                        out_file_data_ << path_length << ", ";
                    }
                    else
                    {
                        out_file_data_ << ", ";
                        std::cerr << "Could not save field: " << field << " does not exist" << std::endl;
                    }
                    field = "line_of_sight_checks";
                    if (_data.find(field) != _data.end())
                    {
                        auto line_of_sight_checks = std::any_cast<int>(_data.at(field));
                        out_file_data_ << line_of_sight_checks << ", ";
                    }
                    else
                    {
                        out_file_data_ << ", ";
                        std::cerr << "Could not save field: " << field << " does not exist" << std::endl;
                    }
                    out_file_data_ << std::endl;
                }
                catch (const std::bad_any_cast &e)
                {
                    std::cerr << "Any cast error: " << e.what() << std::endl;
                    res = false;
                }

                out_file_data_.close();
                return res;
            }

        private:
            std::ofstream out_file_data_;
        };
    }
}

#endif