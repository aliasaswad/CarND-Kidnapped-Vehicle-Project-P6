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

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // Done: Set the number of particles

  //if the filter already init then it's done
  if (is_initialized) {
    return;
  }
  /**
  *Extrct Stdrd Dvns
  */
  double std_x    = std[0];
  double std_y    = std[1];
  double std_theta= std[2];
  /**
  *Creat Nrml Dist.
  */
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);
  /**
  * Gnrt Partcls w Nrml Dist. GPS Vls
  */
  for (int i = 0; i < num_particles; i++) {
    Particle particle;
    particle.id     = i;
    particle.x      = dist_x(gen);
    particle.y      = dist_y(gen);
    particle.theta  = dist_theta(gen);
    particle.weight = 1.0;
    particles.push_back(particle);
  }
  /**
  *  Fltr is_initialized
  */
  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  /**
  *Extrct Stdrd Dvns
  */
  double std_x     = std_pos[0];
  double std_y     = std_pos[1];
  double std_theta = std_pos[2];
  /**
  *Creat Nrml Dist.
  */
  normal_distribution<double> dist_x(0, std_x);
  normal_distribution<double> dist_y(0, std_y);
  normal_distribution<double> dist_theta(0, std_theta);
  /**
  * Calc. the state
  */
  for(int i = 0; i < num_particles; i++) {
    double theta = particles[i].theta;
    // Check if yaw not change
    if(fabs(yaw_rate) < EPS) { 
      particles[i].x += velocity * delta_t * cos(theta);
      particles[i].y += velocity * delta_t * sin(theta);
    }else{
      particles[i].x += velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin( theta));
      particles[i].y += velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }
    // Adding noise
    particles[i].x     += dist_x(gen);
    particles[i].y     += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  unsigned int number_obsrv = observations.size();
  unsigned int number_predc = predicted.size();

  for (unsigned int i = 0; i<number_obsrv; i++) {
    // Init min dist with max size
    double min_dist = numeric_limits<double>::max();
    // Init found map
    int map_id = -1;

    for (unsigned int j = 0; j<number_predc; j++) {
      double x_dist = observations[i].x - predicted[j].x;
      double y_dist = observations[i].y - predicted[j].y;
      double dist = x_dist * x_dist + y_dist * y_dist;
      // Check the dist to store the id and update min
      if (dist < min_dist) {
        min_dist = dist;
        map_id = predicted[j].id;
      }
    }
    observations[i].id = map_id;
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
  double std_landmark_range   = std_landmark[0];
  double std_landmark_bearing = std_landmark[1];
  for (int i = 0; i < num_particles; i++){
    double x     = particles[i].x;
    double y     = particles[i].y;
    double theta = particles[i].theta;
    // Find landmarks in particle's range.
    double sensor_range_2 = sensor_range * sensor_range;
    vector<LandmarkObs> in_range_landmarks;
    for(unsigned int j = 0; j<map_landmarks.landmark_list.size(); j++){
      float landmark_x = map_landmarks.landmark_list[j].x_f;
      float landmark_y = map_landmarks.landmark_list[j].y_f;
      int id = map_landmarks.landmark_list[j].id_i;
      double dx = x - landmark_x;
      double dy = y - landmark_y;
      if (dx*dx+dy*dy <= sensor_range_2) {
        in_range_landmarks.push_back(LandmarkObs{id, landmark_x, landmark_y});
      }
    }
    // Trans obsrv coord.
    vector<LandmarkObs> mapped_obsrv;
    for(unsigned int j = 0; j<observations.size(); j++){
      double xx = cos(theta)*observations[j].x - sin(theta)*observations[j].y + x;
      double yy = sin(theta)*observations[j].x + cos(theta)*observations[j].y + y;
      mapped_obsrv.push_back(LandmarkObs{observations[j].id, xx, yy});
    }
    // Obsrv assoc. to landmark
    dataAssociation(in_range_landmarks, mapped_obsrv);
    //Set weights
    particles[i].weight = 1.0;
    // Calc. weights
    for(unsigned int j = 0; j<mapped_obsrv.size(); j++){
      double observ_x = mapped_obsrv[j].x;
      double observ_y = mapped_obsrv[j].y;
      int landmark_id = mapped_obsrv[j].id;
      double landmark_x, landmark_y;
      unsigned int k = 0;
      unsigned int number_landmarks = in_range_landmarks.size();
      bool found = false;
      while( !found && k < number_landmarks ) {
        if ( in_range_landmarks[k].id == landmark_id) {
          found = true;
          landmark_x = in_range_landmarks[k].x;
          landmark_y = in_range_landmarks[k].y;
        }
        k++;
      }
      // Calc. weight
      double d_x = observ_x - landmark_x;
      double d_y = observ_y - landmark_y;
      double weight = (1/(2*M_PI*std_landmark_range*std_landmark_bearing)) * exp(-( d_x*d_x/(2*std_landmark_range*std_landmark_range) + (d_y*d_y/(2*std_landmark_bearing*std_landmark_bearing))));
      if (weight == 0) {
        particles[i].weight *= EPS;
      } else {
        particles[i].weight *= weight;
      }
    }
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  // Calc max weight
  vector<double> weights;
  double max_weight = numeric_limits<double>::min();
  for(int i = 0; i<num_particles; i++){
    weights.push_back(particles[i].weight);
    if (particles[i].weight > maxWeight){
      max_weight = particles[i].weight;
    }
  }
  // Cret dist.
  uniform_real_distribution<double> distDouble(0.0, max_weight);
  uniform_int_distribution<int> distInt(0, num_particles - 1);
  // Gen indx
  int index = distInt(gen);
  double beta = 0.0;
  // Whel
  vector<Particle> resampled_particles;
  for(int i = 0; i<num_particles; i++){
    beta += distDouble(gen) * 2.0;
    while( beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    resampled_particles.push_back(particles[index]);
  }
  particles = resampled_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  
  //Reset assoc. and then set
  particle.associations.clear();
  particle.sense_x.clear();
  particle.sense_y.clear();
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
  return particle;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSense_x(Particle best)
{
  vector<double> v = best.sense_x;
  std::stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSense_y(Particle best)
{
  vector<double> v = best.sense_y;
  std::stringstream ss;
  copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

// string ParticleFilter::getSenseCoord(Particle best, string coord) {
//   vector<double> v;

//   if (coord == "X") {
//     v = best.sense_x;
//   } else {
//     v = best.sense_y;
//   }

//   std::stringstream ss;
//   copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
//   string s = ss.str();
//   s = s.substr(0, s.length()-1);  // get rid of the trailing space
//   return s;
// }