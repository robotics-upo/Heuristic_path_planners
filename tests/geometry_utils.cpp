#include <catch2/catch.hpp>
#include <limits>
#include "utils/utils.hpp"
#include "utils/geometry_utils.hpp"
#include "utils/heuristic.hpp"

using namespace Planners::utils;
using namespace Catch::literals;

TEST_CASE("Distance between same node is 0", "[geometry_utils][euclidean]")
{

    Node n1{{1, 1, 2}, nullptr};

    CHECK(geometry::distanceBetween2Nodes(&n1, &n1) == 0);

    n1.coordinates = Vec3i{0, 0, 0};
    CHECK(geometry::distanceBetween2Nodes(&n1, &n1) == 0);

    n1.coordinates = Vec3i{-1, -1, -1};
    CHECK(geometry::distanceBetween2Nodes(&n1, &n1) == 0);
}

TEST_CASE("Distances between nodes on same axis", "[geometry_utils][euclidean]")
{
    SECTION("Dist between X axis nodes")
    {
        int x1 = GENERATE(take(100, random(std::numeric_limits<int>::min(), std::numeric_limits<int>::max())));
        int x2 = GENERATE(take(100, random(std::numeric_limits<int>::min(), std::numeric_limits<int>::max())));
        Node n1{{x1, 0, 0}, nullptr};
        Node n2{{x2, 0, 0}, nullptr};

        auto dist = geometry::distanceBetween2Nodes(n1, n2);
        CHECK(dist == dist_scale_factor_ * std::abs(x2 - x1));
        auto heur_dist = Planners::Heuristic::euclidean(n1.coordinates, n2.coordinates);
        CHECK(dist == heur_dist );

    }

    SECTION("Dist beetween Y axis nodes"){
        int y1 = GENERATE(take(100, random(std::numeric_limits<int>::min(), std::numeric_limits<int>::max())));
        int y2 = GENERATE(take(100, random(std::numeric_limits<int>::min(), std::numeric_limits<int>::max())));
        Node n1{{0, y1, 0}, nullptr};
        Node n2{{0, y2, 0}, nullptr};

        auto dist = geometry::distanceBetween2Nodes(n1, n2);
        CHECK(dist == dist_scale_factor_ * std::abs(y2 - y1));
        
        auto heur_dist = Planners::Heuristic::euclidean(n1.coordinates, n2.coordinates);
        CHECK(dist == heur_dist );
    }

    SECTION("Dist beetween Z axis nodes"){
        int z1 = GENERATE(take(100, random(std::numeric_limits<int>::min(), std::numeric_limits<int>::max())));
        int z2 = GENERATE(take(100, random(std::numeric_limits<int>::min(), std::numeric_limits<int>::max())));
        Node n1{{0, 0, z1}, nullptr};
        Node n2{{0, 0, z2}, nullptr};

        auto dist = geometry::distanceBetween2Nodes(n1, n2);
        CHECK(dist == dist_scale_factor_ * std::abs(z2 - z1));

        auto heur_dist = Planners::Heuristic::euclidean(n1.coordinates, n2.coordinates);
        CHECK(dist == heur_dist );
    }
}


TEST_CASE("Diagonal Distances", "[geometry_utils][euclidean]")
{
    SECTION("XY Diagonal"){
        int x1 = GENERATE(take(20, random(std::numeric_limits<int>::min(), std::numeric_limits<int>::max())));
        int x2 = GENERATE(take(20, random(std::numeric_limits<int>::min(), std::numeric_limits<int>::max())));
        int y1 = GENERATE(take(20, random(std::numeric_limits<int>::min(), std::numeric_limits<int>::max())));
        int y2 = GENERATE(take(20, random(std::numeric_limits<int>::min(), std::numeric_limits<int>::max())));
        Node n1{{x1, y1, 0}, nullptr};
        Node n2{{x2, y2, 0}, nullptr};

        unsigned int dist = geometry::distanceBetween2Nodes(n1, n2);
        unsigned int diag_dist = dist_scale_factor_ * sqrt( pow( n1.coordinates.x - n2.coordinates.x, 2 ) +
                                                           pow( n1.coordinates.y - n2.coordinates.y, 2 ) );
        CHECK(dist == diag_dist);
        
        auto heur_dist = Planners::Heuristic::euclidean(n1.coordinates, n2.coordinates);
        CHECK(dist == heur_dist );
    }  
    SECTION("YZ Diagonal"){

        int z1 = GENERATE(take(20, random(std::numeric_limits<int>::min(), std::numeric_limits<int>::max())));
        int z2 = GENERATE(take(20, random(std::numeric_limits<int>::min(), std::numeric_limits<int>::max())));
        int y1 = GENERATE(take(20, random(std::numeric_limits<int>::min(), std::numeric_limits<int>::max())));
        int y2 = GENERATE(take(20, random(std::numeric_limits<int>::min(), std::numeric_limits<int>::max())));
        Node n1{{0, y1, z1}, nullptr};
        Node n2{{0, y2, z2}, nullptr};

        unsigned int dist = geometry::distanceBetween2Nodes(n1, n2);
        unsigned int diag_dist = dist_scale_factor_ * sqrt( pow( n1.coordinates.z - n2.coordinates.z, 2 ) +
                                                            pow( n1.coordinates.y - n2.coordinates.y, 2 ) );
        CHECK(dist == diag_dist);

        auto heur_dist = Planners::Heuristic::euclidean(n1.coordinates, n2.coordinates);
        CHECK(dist == heur_dist );
    }
    SECTION("XZ Diagonal"){

        int x1 = GENERATE(take(20, random(std::numeric_limits<int>::min(), std::numeric_limits<int>::max())));
        int x2 = GENERATE(take(20, random(std::numeric_limits<int>::min(), std::numeric_limits<int>::max())));
        int z1 = GENERATE(take(20, random(std::numeric_limits<int>::min(), std::numeric_limits<int>::max())));
        int z2 = GENERATE(take(20, random(std::numeric_limits<int>::min(), std::numeric_limits<int>::max())));
        Node n1{{x1, 0, z1}, nullptr};
        Node n2{{x2, 0, z2}, nullptr};

        unsigned int dist = geometry::distanceBetween2Nodes(n1, n2);
        unsigned int diag_dist = dist_scale_factor_ * sqrt( pow( n1.coordinates.x - n2.coordinates.x, 2 ) +
                                                            pow( n1.coordinates.z - n2.coordinates.z, 2 ) );
        CHECK(dist == diag_dist);

        auto heur_dist = Planners::Heuristic::euclidean(n1.coordinates, n2.coordinates);
        CHECK(dist == heur_dist );
    }
}

TEST_CASE("Path lenght calculation for straight path in 3 axis", "[geometry_utils][path_length]"){

    CoordinateList coords;
    double resolution = ( static_cast<double>(rand()) / RAND_MAX);
    int len = std::floor((rand() % 300 ) +1);

    Approx len_target     = Approx(len * resolution).epsilon(0.001);

    for(int i = 0; i < len+1; ++i)
        coords.push_back({i, 0, 0});

    CHECK( geometry::calculatePathLength(coords, resolution) == len_target );
    
    coords.clear();
    for(int i = 0; i < len+1; ++i)
        coords.push_back({i, 0, 0});

    CHECK( geometry::calculatePathLength(coords, resolution) == len_target );

    coords.clear();
    for(int i = 0; i < len+1; ++i)
        coords.push_back({i, 0, 0});

    CHECK( geometry::calculatePathLength(coords, resolution) == len_target );

    Approx xylen_target     = Approx(len * resolution * sqrt(2)).epsilon(0.001);
    
    coords.clear();
    for(int i = 0; i < len+1; ++i)
        coords.push_back({i, i, 0});
    
    CHECK( geometry::calculatePathLength(coords, resolution) == xylen_target );

    coords.clear();
    for(int i = 0; i < len+1; ++i)
        coords.push_back({0, i, i});

    CHECK( geometry::calculatePathLength(coords, resolution) == xylen_target );

    coords.clear();
    for(int i = 0; i < len+1; ++i)
        coords.push_back({i, 0, i});

    CHECK( geometry::calculatePathLength(coords, resolution) == xylen_target );

    Approx xyzlen_target     = Approx(len * resolution * sqrt(3)).epsilon(0.001);

    coords.clear();
    for(int i = 0; i < len+1; ++i)
        coords.push_back({i, i, i});

    CHECK( geometry::calculatePathLength(coords, resolution) == xyzlen_target );

    

}


TEST_CASE( "Absolute value", "[geometry_utils][abs]" ){

    Vec3i v1{0,0,0}, v2{-1,0,0}, v3{0, -1, 0}, v4{0, 0, -1}, v5{1, 1, 1};

    Vec3i v1abs = geometry::abs(v1);
    Vec3i v2abs = geometry::abs(v2);
    Vec3i v3abs = geometry::abs(v3);
    Vec3i v4abs = geometry::abs(v4);
    Vec3i v5abs = geometry::abs(v5);

    Vec3i v1r = Vec3i{ std::abs(v1.x), std::abs(v1.y), std::abs(v1.z) };
    Vec3i v2r = Vec3i{ std::abs(v2.x), std::abs(v2.y), std::abs(v2.z) };
    Vec3i v3r = Vec3i{ std::abs(v3.x), std::abs(v3.y), std::abs(v3.z) };
    Vec3i v4r = Vec3i{ std::abs(v4.x), std::abs(v4.y), std::abs(v4.z) };
    Vec3i v5r = Vec3i{ std::abs(v5.x), std::abs(v5.y), std::abs(v5.z) };

    CHECK( v1abs.x == v1r.x );
    CHECK( v2abs.x == v2r.x );
    CHECK( v3abs.x == v3r.x );
    CHECK( v4abs.x == v4r.x );
    CHECK( v5abs.x == v5r.x );

    CHECK( v1abs.y == v1r.y );
    CHECK( v2abs.y == v2r.y );
    CHECK( v3abs.y == v3r.y );
    CHECK( v4abs.y == v4r.y );
    CHECK( v5abs.y == v5r.y );

    CHECK( v1abs.z == v1r.z );
    CHECK( v2abs.z == v2r.z );
    CHECK( v3abs.z == v3r.z );
    CHECK( v4abs.z == v4r.z );
    CHECK( v5abs.z == v5r.z );
}
