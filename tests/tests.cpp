#include <catch2/catch.hpp>
#include "utils/utils.hpp"
#include "utils/geometry_utils.hpp"

TEST_CASE("distance between same node is 0", "[geometry_utils]"){

    Planners::utils::Node n1, n2;
    n2.coordinates = Planners::utils::Vec3i{1,1,2};
    Planners::utils::Vec3i abs_vec{-1, -1, -1};

    n1.coordinates = Planners::utils::Vec3i{1,1,1};
    REQUIRE( Planners::utils::geometry::distanceBetween2Nodes(&n1, &n1) == 0 );
    REQUIRE( Planners::utils::geometry::distanceBetween2Nodes(&n1, &n2) == 100 );
    n1.coordinates = Planners::utils::Vec3i{0,0,0};
    REQUIRE( Planners::utils::geometry::distanceBetween2Nodes(&n1, &n1) == 0 );
    n1.coordinates = Planners::utils::Vec3i{-1,-1,-1};
    REQUIRE( Planners::utils::geometry::distanceBetween2Nodes(&n1, &n1) == 0 );
    // Planners::utils::Vec3i vec2{1,1,1};
    // auto abs = Planners::utils::geometry::abs(n1.coordinates); 
    // REQUIRE( abs ==  vec2 );


}