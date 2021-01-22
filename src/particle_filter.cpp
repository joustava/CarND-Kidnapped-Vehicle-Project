/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 * 
 * Edited by
 * Joost Oostdijk
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

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::discrete_distribution;
using std::uniform_real_distribution;
using std::uniform_int_distribution;

static std::default_random_engine gen;

/**
 * @brief Initialised particles by setting their location randmomly based on a normal distribution and setting an equal weight.
 * 
 * @param x initial measurement in x
 * @param y initial measurement in y
 * @param theta initial heading
 * @param std standard deviations for each measurement
 */
void ParticleFilter::init(double x, double y, double theta, double std[]) {
  num_particles = 50;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  
  for (int i = 0; i < num_particles; i++) {
  	Particle particle;
    particle.id = 0;
    particle.x = dist_x(gen);
    particle.y= dist_y(gen);
    particle.theta = dist_theta(gen);
    particle.weight = 1.0;
    particles.push_back(particle);
    weights.push_back(1.0);
  }
  is_initialized = true;
}

/**
 * @brief Predict where a particle will be after delta_t
 * 
 * @param delta_t time
 * @param std_pos standard deviations for all measurements x, y, theta
 * @param velocity of the vehicle
 * @param yaw_rate heading of the vehicle
 */
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // distributions for random noise.
  normal_distribution<double> dist_x_Q(0, std_pos[0]);
  normal_distribution<double> dist_y_Q(0, std_pos[1]);
  normal_distribution<double> dist_theta_Q(0, std_pos[2]);
  
  for (Particle &particle: particles) {
    double delta_yaw = yaw_rate * delta_t;
    if (fabs(delta_yaw) < 0.000001) {  
      particle.x += velocity * delta_t * cos(particle.theta);
      particle.y += velocity * delta_t * sin(particle.theta);
    } else {
      particle.x += (velocity / yaw_rate) * (sin(particle.theta + delta_yaw) - sin(particle.theta));
      particle.y += (velocity / yaw_rate) * (cos(particle.theta) - (cos(particle.theta + delta_yaw)));
      particle.theta += delta_yaw;
    }
    particle.x += dist_x_Q(gen);
    particle.y += dist_y_Q(gen);
    particle.theta += dist_theta_Q(gen);
  }
}

/**
 * @brief foreach observation find the prediction thatis the most probably match due to being in proximity.
 * 
 * @param predicted 
 * @param observations 
 */
void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, vector<LandmarkObs>& observations) {

  double closest;
  double distance;
  
  for (LandmarkObs &observation: observations) {
    closest = std::numeric_limits<double>::max();
    
    for (const LandmarkObs &prediction: predicted) { 
      distance = dist(observation.x, observation.y, prediction.x, prediction.y);
      
      if(distance < closest) {
      	closest = distance;
        observation.id = prediction.id;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  // double weights_sum = 0;
  double i = 0;
  double std_x = std_landmark[0];
  double std_y = std_landmark[1];
  // precalcualting denominators of mutlivariate expression
  double MV = 2 * M_PI * std_x * std_y;
  double MV_X = 2 * pow(std_x, 2);
  double MV_Y = 2 * pow(std_y, 2);
      
  for(Particle &prediction: particles) {
    vector<LandmarkObs> candidates;
  
    // Find landmarks on the map that are closest to the current prediction  
    for(const auto &landmark: map_landmarks.landmark_list) {
    	double landmark_dist_square = dist(landmark.x_f, prediction.x, landmark.y_f, prediction.y);
    	double range = sensor_range * sensor_range;
    	if (landmark_dist_square <= range) {
          candidates.push_back(LandmarkObs{landmark.id_i, landmark.x_f, landmark.y_f});
    	}
    }
  	
    vector<LandmarkObs> transformations;
    
    for (const LandmarkObs &observation: observations) {
      double x = cos(prediction.theta) * observation.x - sin(prediction.theta) * observation.y + prediction.x;
      double y = sin(prediction.theta) * observation.x + cos(prediction.theta) * observation.y + prediction.y;
      transformations.push_back(LandmarkObs{observation.id, x, y});
    }
    
    dataAssociation(candidates, transformations);

    prediction.weight = 1.0;
    for (LandmarkObs &transformation: transformations) {
      double x_1 = transformation.x;
      double y_1 = transformation.y;
      double x_2 = 0;
      double y_2 = 0;

      for (LandmarkObs &candidate: candidates) {
        if (candidate.id == transformation.id) {
          x_2 = candidate.x;
          y_2 = candidate.y;
          break;
        }
      }

      // Calculate and update weight with multivariate gaussian.
      prediction.weight *= (1 / MV) * exp( -( pow(x_1 - x_2, 2) / MV_X + (pow(y_1 - y_2, 2) / MV_Y)));    
    }
    
    weights[i] = prediction.weight;
    i++;
  }

  double sum = accumulate(weights.begin(), weights.end(), 0.0);
  if (fabs(sum) > 0.0) {
    for (double &weight: weights) {
      weight /= sum;
    }
  }
}

/**
 * @brief Resample particles by their weighted index.
 */
void ParticleFilter::resample() {
  std::discrete_distribution<int> sampler(weights.begin(), weights.end());

  for (auto &particle: particles) {
    size_t idx = sampler(gen);
    particle = particles[idx];
  }
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