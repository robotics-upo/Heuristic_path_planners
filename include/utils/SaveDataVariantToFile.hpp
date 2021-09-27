#ifndef SAVEDATAVARIANTTOFILE_HPP
#define SAVEDATAVARIANTTOFILE_HPP

/**
 * @file DataVariantToFile.hpp
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

#include "utils/utils.hpp"

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
             * @param _data_file data file path 
             * @param _fields A std::vector<std::string> of fields to look for on the input map. The default values 
             * are: algorithm, goal_coords, start_coords, time_spent, explored_nodes, path_length, line_of_sight_checks and solved
             */
            DataVariantSaver(const std::string &_data_file, 
                             const std::vector<std::string> &_fields =
                            {"algorithm", "goal_coords", "start_coords", "time_spent",
                             "explored_nodes", "path_length", "line_of_sight_checks", "solved", "cost_weight","max_line_of_sight_cells" }): fields_(_fields)
            {
                out_file_data_.open(_data_file, std::ofstream::app);
            }
            /**
             * @brief Main function that reads the incoming pathdata object and save to file in append mode
             * 
             * @param _data pathdata input object 
             * @return true Always returns true at this version
             * @return false Never returns false right now
             */
            bool savePathDataToFile(const PathData &_data)
            {
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

        private:
            std::ofstream out_file_data_;
            std::vector<std::string> fields_;
        };
    } // utils ns
} //planners ns

#endif