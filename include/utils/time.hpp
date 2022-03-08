/**
 * @file time.hpp
 * @author Rafael Rey (reyarcenegui@gmail.com)
* @author Jose Antonio Cobano (jacobsua@upo.es)
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
                // start_ = std::chrono::high_resolution_clock::now();
                start_ = std::chrono::steady_clock::now();
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

                // stop_ = std::chrono::high_resolution_clock::now();
                stop_ = std::chrono::steady_clock::now();

                started_ = false;

                duration = (stop_ - start_);
                auto usec = std::chrono::duration_cast<std::chrono::microseconds>(duration);

                if (verbose_)
                    std::cout << "[ " << _msg << "] " << "Elapsed time: " << usec.count() << std::endl;
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
                    
                return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
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

                return duration.count();
            }
            /**
             * @brief Get the elapsed time in seconds
             * 
             * @return double the time elapsed, 0 if toc() was not called yet
             */
            double getElapsedSeconds(){

                if( started_ )
                    return 0;

                return std::chrono::duration_cast<std::chrono::seconds>(duration).count();
            }
            double getElapsedMicroSeconds(){

                if( started_ )
                    return 0;

                return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
            }
        private:

            std::chrono::nanoseconds duration{};
            std::chrono::steady_clock::time_point start_{}, stop_{};

            bool started_{false};
            bool verbose_ { false };
        };
    }
}