#include <catch2/catch.hpp>
#include <vector>
#include "helper_functions.h"
#include "particle_filter.h"

TEST_CASE("Initialization does not occur on creation", "[Particle Filter]") {
  ParticleFilter pf;

  REQUIRE(  pf.initialized() == false );  
  REQUIRE(  pf.particles.size() == 0 );  
}

TEST_CASE("init() initializes filter", "[Particle Filter]") {
  ParticleFilter pf;

  double std[] = {0.5,0.5, 0.5};
  pf.init(1.0, 1.0, 1.0, std);

  REQUIRE(  pf.initialized() == true );
  REQUIRE(  pf.particles.size() == 100 ); 
}

TEST_CASE("prediction() while x direction", "[Particle Filter]") {
  // ParticleFilter pf;

  // double std[] = {1.0, 1.0, 1.0};
  // double v = 1;
  // double x0 = 0.0;
  // double y0 = 0.0;
  // double theta0 = 0; // no yaw, driving in x direction.

  // double std_pos[] = {0.0, 0.0};
  // pf.prediction(1.0, std_pos, 120.0 * 1000 / 3600, M_PI/8);

  // 97.59, 75.08, (51pi/80)
}

TEST_CASE("dataAssociation() assigns closest prediction to each observation", "[Particle Filter]") {
  ParticleFilter pf;
  std::vector<LandmarkObs> predictions;
  std::vector<LandmarkObs> observations;

  
}