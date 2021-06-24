#ifndef SAVEDATAVARIANTTOFILE_HPP
#define SAVEDATAVARIANTTOFILE_HPP

/**
 * @file DataVariantToFile.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com )
 * @brief  Save std::map<std::string, custom_varian> to ile
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

        template <class... Ts>
        struct overloaded : Ts...
        {
            using Ts::operator()...;
        };

        template <class... Ts>
        overloaded(Ts...) -> overloaded<Ts...>;

        class DataVariantSaver
        {

        public:
        
            DataVariantSaver(const std::string &_data_file, 
                             const std::vector<std::string> &_fields =
                            {"algorithm", "goal_coords", "start_coords", "time_spent",
                             "explored_nodes", "path_length", "line_of_sight_checks", "solved"}): fields_(_fields)
            {
                out_file_data_.open(_data_file, std::ofstream::app);
            }
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