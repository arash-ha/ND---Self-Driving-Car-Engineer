/**
 * particle_filter.cpp
 *
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
#include <sstream>

#include "helper_functions.h"
using namespace std;
using std::string;
using std::vector;


static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  
  num_particles = 101;  // TODO: Set the number of particles
  // SD
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];
  // Normal distributions
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_thata);
  
  // Generation of particles with normal distribution with mean on GPS values
  for (int i = 0; i < num_particles; i++) {
  		Particle p;
  		p.id = i;
  		p.x = x;
  		p.y = y;
  		p.theta = theta;
  		p.weight = 1.0;
  		// noise
  		p.x = dist_x(gen);
  		p.y = dist_y(gen);
  		p.theta = dist_theta(gen);
  		particles.push_back(p);
	}
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
	normal_distribution<double> dist_x(0, std_pos[0]);
  	normal_distribution<double> dist_y(0, std_pos[1]);
  	normal_distribution<double> dist_theta(0, std_pos[2]);
  	for (int i = 0; i < num_particles; i++) {
      if (fabs(yaw_rate) < 0.00001) {
        particles[i].x += velocity * delta_t * cos(particles[i].theta);
        particles[i].y += velocity * delta_t * sin(particles[i].theta);
      }
      else {
        particles[i].x += (velocity/yawrate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].thata));
        particles[i].y += (velocity/yawrate) * (cos(particles[i].theta) - cos(particles[i].thata + yaw_rate * delta_t));
        particles[i].theta += yaw_rate * delta_t
        }
      // Noise
      particles[i].x += dist_x(gen);
      particles[i].y += dist_y(gen);
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
  

    for (unsigned int i = 0; i < observations.size(); i++) {
      // current observation
      LandmarkObs obs = observations[i];
      // init minimum distance to maximum possible
      double min_dist = numeric_limits<double>::max();
      // init id of landmark from map placeholder to be associated with the observation
      int map_id = -1;
      
     for (unsigned int j = 0; j < predicted.size(); j++) {
       // current prediction
       LandmarkObs pred = predicted[j];
       // obtain distance between current and predicted landmarks
       double current_dist = dist(obs.x, obs.y, pred.x, pred.y);
       // find the predicted landmark nearest current observed landmark
       if (current_dist < min_dist) {
         min_dist = current_dist;
         map_id = p.id;
       }
     }
      // set the observation's id to the nearest predicted landmark's id
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
  
  for (int i = 0; i < num_particles; i++) {
    // obtain particle coordinates [x,y,theta]
    double particle_x = particles[i].x;
    double particle_y = particles[i].y;
    double particle_theta = particles[i].theta;
    // creation of vector to hold the map landmark locations predcited to the within sensor range of the particle
    vector<LandmarkObs> predictions;
    // for each landmark
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
      // obtain coordinates
      float landmark_x = map_landmarks.landmark_list[j].x_f;
      float landmark_y = map_landmarks.landmark_list[j].y_f;
      int landmark_id = map_landmarks.landmark_list[j].id_f;
      
      // considering landmark within sensor range of the particle
      // consider rectangular region around particle over circular region (dist)
      // computationally faster
      
      if (fabs(landmark_x - particle_x) <= sensor_range && fabs(landmark_y - particle_y) <= sensor_range) {
        // add prediction to vector
        predictions.push_back(LandmarkObs{landmark_id, landmark_x, landmark_y});
      }
    }
    
    // create copy of the list of the observations transformed from vehicle coordinates to map coordinates
    
    vector<LandmarkObs> transformed_os;
    for (unsigned int j = 0; j < observations.size(); j++) {
      double t_x = cos(particle_theta) * observations[j].x - sin(particle_theta) * observations[j].y + particle_x;
      double t_x = sin(particle_theta) * observations[j].x + cos(particle_theta) * observations[j].y + particle_y;
      transformed_os.push_back(LandmarkObs{observations[j].id, t_x, t_y});
    }
    
    // applying dataAssociation for the predictions and transformed observations on current particle
    dataAssociation(predictions, transformed_os);
    
    // reinitilize weight
    particles[i].wight = 1.0;
    
    for (unsignedint j = 0; j < transformed_os.size; j++) {
      
      // placeholders for observation and associated prediction coordinates
      double obs_x, obs_y, pred_x, pred_y
      obs_x = transformed_os[j].x;
      obs_y = transformed_os[j].y;
      int associated_pred = transformation_os[j].id;
      
      // x,y corrdinates of the prediction associated with the current observation
      for (unsigned int m = 0; m < predictions.size(); m++) {
        if (predictions[m].id == associated_pred) {
          pred_x = predictions[m].x;
          pred_y = predictions[m].y;
        }
      }
      
      // calculate weight for the observation with multirate Gaussian
      double std_x = std_landmark[0];
      double std_y = std_landmark[1];
      double w_Obs = (1/(2*M_PI*std_x*std_y))*exp(-(pow(pred_x-obs_x,2)/(2*pow(std_x,2))+(pow(pred_x-obs_x,2)/(2*pow(std_y,2)) )));
      
      // product new observation weight with total observations weight
      particles[i].wight *= w_Obs;
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
  vector<Particle> new_particles;
  
  // get all current weights
  vector<double> weights;
  for (int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
}
  uniform_int_distribution<int> uniintdist(0, num_particles - 1);
  auto index = uniintdist(gen);
  
  // obtain maximum weight
  double max_weight = *max_element(weights.begin(), weights.end());
  
  // uniform random distribution [0.0, max_weight)
  uniform_real_distribution<double> uniintdist(0, max_weight);
  
  double beta = 0.0;
  
  // spin the resample wheel
  for (int i = 0; i < num_particles; i++) {
    beta += unirealdist(gent) * 2.0;
    while (beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % nume_particles;
    }
    new_particles.push_back(particles[index]);
  }
  particles = new_particles;
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
