#include <catch2/catch.hpp>
#include "utils/time.hpp"
#include <time.h>

using namespace Planners::utils;
using namespace Catch::literals;

TEST_CASE("Can not call toc() on a non started timer", "[clock]"){

    Clock clock;

    clock.toc();

    CHECK(clock.getElapsedMillisecs() == 0);
    CHECK(clock.getElapsedNanosecs()  == 0);
    CHECK(clock.getElapsedSeconds()   == 0);
}

TEST_CASE( "Random durations match approximately (2% of error)", "[random_elapsed_time][clock]" ){
    
    Clock clock;
    struct timespec spec;
    spec.tv_sec = 0;
    // The value of the nanoseconds field must be in the range 0 to 999999999.
    spec.tv_nsec = GENERATE(take(50, random(0,999999999)));

    //If tic() is called multiple times before calling toc
    //the elapsed time is the time between last tic and first toc
    clock.tic();
    clock.tic();
    clock.tic();
    nanosleep(&spec, nullptr);
    clock.toc();

    Approx nanotarget     = Approx(spec.tv_nsec).epsilon(0.1);
    Approx milisecstarget = Approx(spec.tv_nsec/1e6).epsilon(0.1);
    Approx secstarget     = Approx(spec.tv_nsec/1e9).epsilon(0.1);

    CHECK( clock.getElapsedNanosecs()   == nanotarget );
    CHECK( clock.getElapsedMillisecs()  == milisecstarget );
    CHECK( clock.getElapsedSeconds()    == secstarget );
    //Caling toc again should return the same 
    clock.toc();

    CHECK( clock.getElapsedNanosecs()   == nanotarget );
    CHECK( clock.getElapsedMillisecs()  == milisecstarget );
    CHECK( clock.getElapsedSeconds()    == secstarget );

}
