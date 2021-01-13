#include <catch2/catch.hpp>
#include "particle_filter.h"

TEST_CASE("Initialization does not occur on creation", "[Particle Filter]") {
  ParticleFilter pf;

  REQUIRE(  pf.initialized() == false );
}