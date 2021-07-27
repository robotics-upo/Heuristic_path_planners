#include <catch2/catch.hpp>
#include <cstdlib>
#include "utils/utils.hpp"
#include "utils/LineOfSight.hpp"
#include "utils/world.hpp"

#define N_TESTS 6
#define WORLD_TEST_SIZE 50
#define WORLD_TEST_RESOLUTION 0.1

using namespace Planners::utils;
using record = std::tuple<int, int, int>;
using record2d = std::tuple<int, int>;

TEST_CASE("It should exist line of sight between a random node and itself in an 3D empty world", "[bresenham3D][empty_world][nodes_itself]")
{
    DiscreteWorld world;
    int world_size = WORLD_TEST_SIZE;

    world.resizeWorld(world_size, world_size, world_size, WORLD_TEST_RESOLUTION);

    auto r = GENERATE_COPY(table<int, int, int>({
        record{world_size-1, world_size-1, world_size-1},
    }));

    auto x = GENERATE_COPY(take(N_TESTS, random(0, 0)));
    auto y = GENERATE_COPY(take(N_TESTS, random(0, std::get<1>(r))));
    auto z = GENERATE_COPY(take(N_TESTS, random(0, std::get<2>(r))));

    Vec3i vec1{x, y, z};

    // CHECK(LineOfSight::bresenham3D(vec1, vec1, world));
}
