#include <chrono>
#include <iostream>

namespace Planners
{
    namespace utils
    {
        class Clock
        {

        public:

            Clock(const bool _verbose = false) : verbose_(_verbose)
            {
            }
            void tic()
            {
                start_ = std::chrono::high_resolution_clock::now();
                started_ = true;
            }

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
            double getElapsedMillisecs()
            {
                if( started_ )
                    return 0;
                    
                return elapsed_.count() * 1e-6;
            }
            double getElapsedNanosecs()
            {
                if( started_ )
                    return 0;

                return elapsed_.count();
            }
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