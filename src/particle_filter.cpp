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

#define EPS 0.00001

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    default_random_engine gen;

    if(is_initialized){
        return ;
    }
    num_particles = 100;
    
    normal_distribution<double> dist_x(x,std[0]);
    normal_distribution<double> dist_y(y,std[1]);
    
    normal_distribution<double> dist_theta(theta,std[2]);
    
    for(int i = 0 ; i<num_particles ; i++){
        Particle my_particle ;
        my_particle.id = i;
        my_particle.x = dist_x(gen);
        my_particle.y= dist_y(gen);
        my_particle.theta = dist_theta(gen);
        my_particle.weight = 1.0;
        
        particles.push_back(my_particle);
        weights.push_back(my_particle.weight);
        
    }
    
    is_initialized = true;
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/
    
    // Extracting standard deviations
    default_random_engine gen;
    
    for(unsigned int i = 0 ; i < num_particles  ; i++){
        double X  = particles[i].x;
        double Y  = particles[i].y;
        double theta  = particles[i].theta;

        double predX ;
        double predY ;
        double predTheta ;
        
        if(fabs(yaw_rate) < 0.0001){
            predX = X + velocity * cos(theta) * delta_t;
            predY = Y + velocity * sin(theta) * delta_t;
            predTheta=theta;
        } else {
            predX = X + (velocity/yaw_rate) * (sin(theta + (yaw_rate * delta_t)) - sin(theta));
            predY = Y + (velocity/yaw_rate) * (cos(theta) - cos(theta + (yaw_rate * delta_t)));
            predTheta = theta + (yaw_rate * delta_t);
        }
        normal_distribution<double> dist_x(predX, std_pos[0]);
        normal_distribution<double> dist_y(predY, std_pos[1]);
        normal_distribution<double> dist_theta(predTheta, std_pos[2]);
        
        particles[i].x = dist_x(gen);
        particles[i].y = dist_y(gen);
        particles[i].theta = dist_theta(gen);
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.
    for (int i = 0;i< observations.size() ; i++){
        
        double minDistance = numeric_limits<double>::max();
        int mapId = -1;
        
        for (int j = 0 ; j < predicted.size() ; j++){
            double distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);
            if( distance <  minDistance ) {
                minDistance = distance;
                mapId = predicted[j].id;
            }
        }
        
        observations[i].id = mapId;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {    // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
    //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
    //   according to the MAP'S coordinate system. You will need to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to implement (look at equation
    //   3.33
    //   http://planning.cs.uiuc.edu/node99.html
    for(int i = 0 ; i  < num_particles;i++){
        double x = particles[i].x;
        double y = particles[i].y;
        double theta = particles[i].theta;
        
        vector<LandmarkObs> predicted_landmarks;
        for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
            double landmarkX = map_landmarks.landmark_list[j].x_f;
            double landmarkY = map_landmarks.landmark_list[j].y_f;
            int id = map_landmarks.landmark_list[j].id_i;
            double distance = dist(particles[j].x,particles[j].y,landmarkX,landmarkY);
            if( distance < sensor_range){ // if the landmark is within the sensor range, save it to predictions
                predicted_landmarks.push_back(LandmarkObs{id ,landmarkX, landmarkY});
            }
        }
        
        vector<LandmarkObs> transformed_observations;

        for (int j = 0; j < observations.size(); j++) {
            LandmarkObs transformed_obs;
            transformed_obs.id = j;
            transformed_obs.x = x + (cos(theta) * observations[j].x) - (sin(theta) * observations[j].y);
            transformed_obs.y = y + (sin(theta) * observations[j].x) + (cos(theta) * observations[j].y);
            transformed_observations.push_back(transformed_obs);
        }
        dataAssociation(predicted_landmarks, transformed_observations);
        
        particles[i].weight = 1.0;
        // Calculate weights.
        for(unsigned int j = 0; j < transformed_observations.size(); j++) {
            double observationX = transformed_observations[j].x;
            double observationY = transformed_observations[j].y;
            
            int landmarkId = transformed_observations[j].id;
            
            double landmarkX, landmarkY;
            unsigned int k = 0;
            unsigned int nLandmarks = predicted_landmarks.size();
            bool found = false;
            while( !found && k < nLandmarks ) {
                if ( predicted_landmarks[k].id == landmarkId) {
                    found = true;
                    landmarkX = predicted_landmarks[k].x;
                    landmarkY = predicted_landmarks[k].y;
                }
                k++;
            }
            double dX = observationX - landmarkX;
            double dY = observationY - landmarkY;
            
            double weight = ( 1/(2*M_PI*std_landmark[0]*std_landmark[1])) * exp( -( dX*dX/(2*std_landmark[0]*std_landmark[0]) + (dY*dY/(2*std_landmark[1]*std_landmark[1])) ) );
            if (weight == 0) {
                particles[i].weight *= EPS;
            } else {
                particles[i].weight *= weight;
            }
        }

    }
    
}

void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    default_random_engine gen;

    // Get weights and max weight.
    vector<double> weights;
    double maxWeight = numeric_limits<double>::min();
    for(int i = 0; i < num_particles; i++) {
        weights.push_back(particles[i].weight);
        if ( particles[i].weight > maxWeight ) {
            maxWeight = particles[i].weight;
        }
    }
    
    // Creating distributions.
    uniform_real_distribution<double> distDouble(0.0, maxWeight);
    uniform_int_distribution<int> distInt(0.0, num_particles - 1);
    
    // Generating index.
    int index = distInt(gen);
    
    double beta = 0.0;
    
    // the wheel
    vector<Particle> resampledParticles;
    for(int i = 0; i < num_particles; i++) {
        beta += distDouble(gen) * 2.0;
        while( beta > weights[index]) {
            beta -= weights[index];
            index = (index + 1) % num_particles;
        }
        resampledParticles.push_back(particles[index]);
    }
    
    particles = resampledParticles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations,
                                         const std::vector<double>& sense_x, const std::vector<double>& sense_y){
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
