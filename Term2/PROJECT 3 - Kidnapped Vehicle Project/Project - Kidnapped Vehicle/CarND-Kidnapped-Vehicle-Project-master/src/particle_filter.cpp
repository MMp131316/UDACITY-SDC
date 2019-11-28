/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#define TH 0.00001

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    // TODO: tweak number of particles
    if (is_initialized) {
        return;
    }

    // Initializing the number of particles
    num_particles = 120;

    // Extracting standard deviations
    double x_std = std[0];
    double y_std = std[1];
    double theta_std = std[2];

    // Creating normal distributions
    normal_distribution<double> dist_x(x, x_std);
    normal_distribution<double> dist_y(y, y_std);
    normal_distribution<double> dist_theta(theta, theta_std);

    // Generate particles
    for (int i = 0; i < num_particles; i++) {

        Particle particle;
        particle.id = i;
        particle.x = dist_x(gen);
        particle.y = dist_y(gen);
        particle.theta = dist_theta(gen);
        particle.weight = 1.0;

        particles.push_back(particle);
    }

    // The filter is now initialized.
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/
    // Set up Gaussian noise, as in `ParticleFilter::init`
    // Some constants to save computation power

    // Extracting standard deviations
    double x_std = std_pos[0];
    double y_std = std_pos[1];
    double theta_std = std_pos[2];

    // Creating normal distributions
    normal_distribution<double> dist_x(0, x_std);
    normal_distribution<double> dist_y(0, y_std);
    normal_distribution<double> dist_theta(0, theta_std);

    for (int i = 0; i < num_particles; i++) {

        double theta = particles[i].theta;

        if ( fabs(yaw_rate) < TH ) { // When yaw is not changing.
            particles[i].x += velocity * delta_t * cos( theta );
            particles[i].y += velocity * delta_t * sin( theta );
            // yaw continue to be the same.
        } else {
            particles[i].x += velocity / yaw_rate * ( sin( theta + yaw_rate * delta_t ) - sin( theta ) );
            particles[i].y += velocity / yaw_rate * ( cos( theta ) - cos( theta + yaw_rate * delta_t ) );
            particles[i].theta += yaw_rate * delta_t;
        }

        // Adding noise.
        particles[i].x += dist_x(gen);
        particles[i].y += dist_y(gen);
        particles[i].theta += dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.
    unsigned int nObservations = observations.size();
    unsigned int nPredictions = predicted.size();

    for (unsigned int i = 0; i < nObservations; i++) { // For each observation

        // Initialize min distance as a really big number.
        double minDistance = 9999999;
        int mapId = -1;

        for (unsigned j = 0; j < nPredictions; j++ ) {
            double xDistance = observations[i].x - predicted[j].x;
            double yDistance = observations[i].y - predicted[j].y;

            double distance = xDistance * xDistance + yDistance * yDistance;

            if ( distance < minDistance ) {
                minDistance = distance;
                mapId = predicted[j].id;
            }
        }

        // Update the observation identifier.
        observations[i].id = mapId;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
    // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
    //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
    //   according to the MAP'S coordinate system. You will need to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to implement (look at equation
    //   3.33
    //   http://planning.cs.uiuc.edu/node99.html
    for(auto& p: particles){
        p.weight = 1.0;

        vector<LandmarkObs> predictions;
        for(const auto& lm: map_landmarks.landmark_list){
            double distance = dist(p.x, p.y, lm.x_f, lm.y_f);
            if( distance < sensor_range){ // if the landmark is within the sensor range, save it to predictions
                predictions.push_back(LandmarkObs{lm.id_i, lm.x_f, lm.y_f});
            }
        }

        vector<LandmarkObs> observations_map;
        double cos_theta = cos(p.theta);
        double sin_theta = sin(p.theta);

        for(const auto& obs: observations){
            LandmarkObs tmp;
            tmp.x = obs.x * cos_theta - obs.y * sin_theta + p.x;
            tmp.y = obs.x * sin_theta + obs.y * cos_theta + p.y;
            observations_map.push_back(tmp);
        }

         dataAssociation(predictions, observations_map);

         for(const auto& obs_m: observations_map){

            Map::single_landmark_s landmark = map_landmarks.landmark_list.at(obs_m.id-1);
            double x_term = pow(obs_m.x - landmark.x_f, 2) / (2 * pow(std_landmark[0], 2));
            double y_term = pow(obs_m.y - landmark.y_f, 2) / (2 * pow(std_landmark[1], 2));
            double w = exp(-(x_term + y_term)) / (2 * M_PI * std_landmark[0] * std_landmark[1]);
            p.weight *=  w;
        }

        weights.push_back(p.weight);

    }
}

void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    vector<Particle> resampledParticles;

    vector<double> weights;
    for (int i = 0; i < num_particles; i++) {
        weights.push_back(particles[i].weight);
    }

    uniform_int_distribution<int> uniintdist(0, num_particles-1);
    auto index = uniintdist(gen);
    double max_weight = *max_element(weights.begin(), weights.end());

    uniform_real_distribution<double> unirealdist(0.0, max_weight);

    double beta = 0.0;

    for (int i = 0; i < num_particles; i++) {
        beta += unirealdist(gen) * 2.0;
        while (beta > weights[index]) {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        resampledParticles.push_back(particles[index]);
    }

    particles = resampledParticles;

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                         const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

    return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
    vector<int> v = best.associations;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
    vector<double> v = best.sense_x;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
    vector<double> v = best.sense_y;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
