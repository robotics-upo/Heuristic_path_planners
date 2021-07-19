#include <catch2/catch.hpp>
#include "utils/utils.hpp"
#include "utils/LineOfSight.hpp"
#include "utils/world.hpp"

SCENARIO( "Line of sight in empty world", "[empty]" ) {

    GIVEN( "An empty 2D world with 0.1 resolution" ) {
        Planners::utils::DiscreteWorld world;
        world.resizeWorld(100,100,1, 0.1);

        WHEN( "Point is itseld" ) {
            
            Planners::utils::Vec3i vec1{1,1,1};
            // TODO Uncomment the requires when implementing overloaded functions to pass vec3i instead of node
            THEN( "It should exist line of sight between the node itself" ) {
                // REQUIRE( Planners::utils::LineOfSight::bresenham3D(vec1, vec1, world) );
                // REQUIRE( Planners::utils::LineOfSight::bresenham3D(vec1, vec1, world)  );
            }
            
        }
        WHEN( "One of the input coordinates is out of bounds" ){

            Planners::utils::Vec3i vec1{1,1,1};
            Planners::utils::Vec3i vec2{102,1,1};

            THEN( "It shouldn't exist line of sight"){
                // REQUIRE( !Planners::utils::LineOfSight::bresenham3D(vec1, vec2, world) );
            }

        }
    }


}