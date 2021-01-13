/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <float.h>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::min_element;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  
  std::default_random_engine gen;  // random generator
  double std_x, std_y, std_theta;  // Standard deviations for x, y, and theta
  num_particles = 100;
  weights = vector<double>(num_particles, 1.0);

  // TODO: Set standard deviations for x, y, and theta
  std_x = std[0];
  std_y = std[1];
  std_theta = std[2];

  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);
  normal_distribution<double> dist_noise(0, 0.1);

  for(int i = 0; i < num_particles; i++) {
    Particle particle;
    particle.id = i;
    particle.x = dist_x(gen) + dist_noise(gen);
    particle.y = dist_y(gen) + dist_noise(gen);
    particle.theta = dist_theta(gen) + dist_noise(gen);
    particle.weight = 1.0;
    particles.push_back(particle);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  std::default_random_engine gen;
  normal_distribution<double> dist_x_Q(0, std_pos[0]);
  normal_distribution<double> dist_y_Q(0.0, std_pos[1]);
  normal_distribution<double> dist_theta_Q(0.0, std_pos[2]);

  for(Particle &particle: particles) {
    particle.x += velocity/yaw_rate * (sin(particle.theta + (yaw_rate * delta_t) - sin(particle.theta)));
    particle.y += velocity/yaw_rate * (cos(particle.theta) - cos(particle.theta + (yaw_rate * delta_t)));
    particle.theta += yaw_rate * delta_t;

    particle.x += dist_x_Q(gen);
    particle.y += dist_y_Q(gen);
    particle.theta += dist_theta_Q(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
    double min_distance = DBL_MAX;
    LandmarkObs closest_prediction;
    
    for(LandmarkObs &observation: observations) {
      for(LandmarkObs &prediction: predicted) {
        double current_distance = dist(prediction.x, observation.x, prediction.y, observation.y);
        if(current_distance < min_distance) {
          closest_prediction = prediction;
        }
      }
      observation.id = closest_prediction.id;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */  
  // 1. transform observations to map coordinates.
  // xm=xp+(cosθ×xc)−(sinθ×yc)
  // ym=yp+(sinθ×xc)+(cosθ×yc)


}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}