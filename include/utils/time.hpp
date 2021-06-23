#include <chrono>
#include <iostream>

namespace Planners
{
    namespace utils
    {
        class Clock
        {

        public:
            /**
             * @brief Construct a new Clock object
             * 
             * @param _verbose 
             */
            Clock(const bool _verbose = false) : verbose_(_verbose)
            {
            }
            /**
             * @brief 
             * 
             */
            void tic()
            {
                start_ = std::chrono::high_resolution_clock::now();
                started_ = true;
            }
            /**
             * @brief 
             * 
             * @param _msg 
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
             * @brief Get the Elapsed Millisecs object
             * 
             * @return double 
             */
            double getElapsedMillisecs()
            {
                if( started_ )
                    return 0;
                    
                return elapsed_.count() * 1e-6;
            }
            /**
             * @brief Get the Elapsed Nanosecs object
             * 
             * @return double 
             */
            double getElapsedNanosecs()
            {
                if( started_ )
                    return 0;

                return elapsed_.count();
            }
            /**
             * @brief Get the Elapsed Seconds object
             * 
             * @return double 
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