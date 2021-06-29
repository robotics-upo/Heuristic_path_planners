/**
 * @file time.hpp
 * @author Rafael Rey (rreyarc@upo.es)
 * @brief Helper Clock Class to easily measure time durations
 * @version 0.1
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <chrono>
#include <iostream>

namespace Planners
{
    namespace utils
    {
        /**
         * @brief Very simple Class to easily measure time duration with tic() and toc() functions
         * 
         */
        class Clock
        {

        public:
            /**
             * @brief Construct a new Clock object
             * 
             * @param _verbose boolean parameter to enable console output when calling the stop toc() function
             */
            Clock(const bool _verbose = false) : verbose_(_verbose)
            {
            }
            /**
             * @brief Start the chrono
             * 
             */
            void tic()
            {
                start_ = std::chrono::high_resolution_clock::now();
                started_ = true;
            }
            /**
             * @brief End the chorno 
             * 
             * @param _msg Message to put on the verbose output to differentiate it from other chronos
             */
            void toc(const std::string &_msg = "")
            {
                if (!started_)
                    return;

                stop_ = std::chrono::high_resolution_clock::now();
                started_ = false;
                
                elapsed_ = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_ - start_);

                if (verbose_)
                    std::cout << "[ " << _msg << "] " << "Elapsed time: " << elapsed_.count() * 1e-6 << std::endl;
            }
            /**
             * @brief Get the elapsed time in milliseconds
             * 
             * @return double the time elapsed, 0 if toc() was not called yet
             */
            double getElapsedMillisecs()
            {
                if( started_ )
                    return 0;
                    
                return elapsed_.count() * 1e-6;
            }
            /**
             * @brief Get the elapsed time in nanoseconds
             * 
             * @return double the time elapsed, 0 if toc() was not called yet
             */
            double getElapsedNanosecs()
            {
                if( started_ )
                    return 0;

                return elapsed_.count();
            }
            /**
             * @brief Get the elapsed time in seconds
             * 
             * @return double the time elapsed, 0 if toc() was not called yet
             */
            double getElapsedSeconds(){

                if( started_ )
                    return 0;

                return elapsed_.count() * 1e-9;
            }

        private:

            std::chrono::_V2::system_clock::time_point start_, stop_;
            std::chrono::nanoseconds elapsed_{0};

            bool started_{false};
            bool verbose_ { false };
        };
    }
}