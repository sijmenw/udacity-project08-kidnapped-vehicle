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

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    // set all weights to 1
    weights.assign(num_particles, 1.0f);

    // create norm distributions for x, y and theta
    std::default_random_engine gen;
    std::normal_distribution<double> norm_x(x, std[0]);
    std::normal_distribution<double> norm_y(y, std[1]);
    std::normal_distribution<double> norm_t(theta, std[2]);

    // init all particles
    for (int i = 0; i < num_particles; ++i) {
        Particle tmp;
        tmp.x = norm_x(gen);
        tmp.y = norm_y(gen);
        tmp.theta = norm_t(gen);
        particles.push_back(tmp);
    }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    // updates step
    for (int i = 0; i < num_particles; ++i) {
        particles[i].x += (velocity/yaw_rate) * (sin(particles[i].theta + delta_t * yaw_rate) - sin(particles[i].theta));
        particles[i].y += (velocity/yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
        particles[i].theta += delta_t * yaw_rate;
    }

    // add noise step
    // create norm distributions for x, y and theta
    std::default_random_engine gen;
    std::normal_distribution<double> norm_x(0, std_pos[0]);
    std::normal_distribution<double> norm_y(0, std_pos[1]);
    std::normal_distribution<double> norm_t(0, std_pos[2]);

    for (int i = 0; i < num_particles; ++i) {
        particles[i].x += norm_x(gen);
        particles[i].y += norm_y(gen);
        particles[i].theta += norm_t(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

    // for each observation, find closest and set values to those of closest
    for (int i = 0; i < observations.size(); ++i) {
        double minDist = 999999999;
        int minI = -1;

        for (int j = 0; j < predicted.size(); ++j){
            double dist = HELPER_FUNCTIONS_H_::dist(observations[i].x, observations[i].y,
                                                   predicted[i].x, predicted[i].y);

            if (dist < minDist) {
                minDist = dist;
                minI = j;
            }
        }

        observations[i].x = predicted[minI].x;
        observations[i].y = predicted[minI].y;
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
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
