#include <catch2/catch.hpp>
#include "particle_filter.h"

TEST_CASE("Initialization does not occur on creation", "[Particle Filter]") {
  ParticleFilter pf;

  REQUIRE(  pf.initialized() == false );  
  REQUIRE(  pf.particles.size() == 0 );  
}

TEST_CASE("Initialization occurs after init() call", "[Particle Filter]") {
  ParticleFilter pf;

  double std[] = {0.5,0.5, 0.5};
  pf.init(1.0, 1.0, 1.0, std);

  REQUIRE(  pf.initialized() == true );
  REQUIRE(  pf.particles.size() == 100 ); 
}