/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <iostream>
#include <sstream>

#include "particle_filter.h"

using namespace std;

static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    num_particles = 101;

    normal_distribution<double> dist_x(0, std[0]);
    normal_distribution<double> dist_y(0, std[1]);
    normal_distribution<double> dist_theta(0, std[2]);

    particles.resize(num_particles);

    for (int i = 0; i < num_particles; ++i) {
        particles[i].id = i;
        particles[i].x = x + dist_x(gen);
        particles[i].y = y + dist_x(gen);
        particles[i].theta = theta + dist_theta(gen);
        particles[i].weight = 1.0;
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/
    std::normal_distribution<double> dist_x(0, std_pos[0]);
    std::normal_distribution<double> dist_y(0, std_pos[1]);
    std::normal_distribution<double> dist_theta(0, std_pos[2]);

    for (auto &particle : particles) {
        // calculate new state
        if (fabs(yaw_rate) < 0.00001) {
            particle.x += velocity * delta_t * sin(particle.theta);
            particle.y += velocity * delta_t * cos(particle.theta);
        }
        else {
            particle.x += velocity / yaw_rate * (sin(particle.theta + yaw_rate*delta_t) - sin(particle.theta));
            particle.y += velocity / yaw_rate * (cos(particle.theta) - cos(particle.theta + yaw_rate*delta_t));
            particle.theta += yaw_rate * delta_t;
        }

        // Add the resulting noise values from the std_deviation to the points
        particle.x += dist_x(gen);
        particle.y += dist_y(gen);
        particle.theta += dist_theta(gen);
    }
}

std::vector<LandmarkObs> ParticleFilter::transformObservations(const Particle& particle, const std::vector<LandmarkObs>& observations) {
    std::vector<LandmarkObs> transformedObservations;
    for (auto obs : observations) {
        LandmarkObs transformed_observation;
        transformed_observation.id = obs.id;
        transformed_observation.x = obs.x * cos(-particle.theta) + obs.y * sin(-particle.theta) + particle.x;
        transformed_observation.y = -obs.x * sin(-particle.theta) + obs.y * cos(-particle.theta) + particle.y;
        transformedObservations.push_back(transformed_observation);
    }

    return transformedObservations;
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.
    for (auto &obs : observations) {
        double min_distance = std::numeric_limits<double>::max();
        for (auto predicted_landmark : predicted) {
            double distance = dist(predicted_landmark.x, predicted_landmark.y, obs.x, obs.y);
            if (distance < min_distance) {
                obs.id = predicted_landmark.id;
                min_distance = distance;
            }
        }
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   vector<LandmarkObs> observations, Map map_landmarks) {
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

    for (auto &particle : particles) {
        vector<LandmarkObs> predictions;

        // For each landmark
        for (auto landmark : map_landmarks.landmark_list) {
            if (fabs(particle.x - landmark.x_f) <= sensor_range && fabs(particle.y - landmark.y_f) <= sensor_range) {
                // Add the prediction to the vector
                predictions.push_back(LandmarkObs {landmark.id_i, landmark.x_f, landmark.y_f});
            }
        }

        vector<LandmarkObs> transformed_observations = transformObservations(particle, observations);

        // Associate the data with the landmark
        dataAssociation(predictions, transformed_observations);

        // reinitialize the weights
        particle.weight = 1.0;

        for (auto current_observation : transformed_observations) {
            int associated_prediction = current_observation.id;
            double pr_x, pr_y;
            int prediction_id;

            for (auto prediction : predictions) {
                if (prediction.id == associated_prediction) {
                    pr_x = prediction.x;
                    pr_y = prediction.y;
                    prediction_id = prediction.id;
                }
            }

            // Calculate the weight for this observation with multivariate Gaussian
            double s_x = std_landmark[0];
            double s_y = std_landmark[1];
            double obs_w = ( 1/(2*M_PI*s_x*s_y)) * exp(-( pow(pr_x-current_observation.x, 2)/(2*pow(s_x, 2)) + (pow(pr_y-current_observation.y,2)/(2*pow(s_y, 2))) ) );
            particle.weight *= obs_w;
        }
    }
}

void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    default_random_engine gen;
    vector<Particle> new_particles;

    vector<double> weights;
    for (auto particle : particles) {
        weights.push_back(particle.weight);
    }

    uniform_int_distribution<int> uni_int_dist(0, num_particles - 1);
    auto index = uni_int_dist(gen);

    // Get the max weight
    double max_weight = *max_element(weights.begin(), weights.end());

    double beta = 0;

    // Spin the wheel
    for (int i = 0; i < num_particles; ++i) {
        beta = uni_int_dist(gen) * 2.0 * max_weight;
        while (beta > weights[index]) {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }

        new_particles.push_back(particles[index]);
    }

    particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    //Clear the previous associations
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