// my implementation of a kalman filter
// kalman filter is a very useful recursive filtering algorithm often used for state estimation. This file will walk you through the initialization of a kalman filter in C++
// kalman filter is used in linear systems to estimate states using the equation: xk = F(xk-1) + B(uk-1) + wk-1, where k and k -1 are the current and previpus timestep respectively
// F is the state transition matrix, xk - 1 is the previous state vector, B is the control input matrix applied to control vector uk-1, and wk - 1 is process noise vector.
// Along with a process model, there is a measurement model that describes the relationship between state and measurement at timestep k. It is defined as zk = Hxk + vk
// zk is the measurement vector, H is the measurement matrix and vk is the measurement noise vector

//The code below shows how you can use a kalman filter given an input 
using namespace std;
#include <iostream>
#include <vector>
#include <cmath>

class KalmanFilter {
    // class variables, assigned in constructor
    private:
    double current_estimate;
    double process_noise;
    double sensor_noise;
    double estimated_error;


    // constructor for the filter
    public:
        KalmanFilter(double initial_value, double process_noise, double sensor_noise, double estimated_error) {
            this->current_estimate = initial_value;
            this->process_noise = process_noise;
            this->sensor_noise = sensor_noise;
            this->estimated_error = estimated_error;

        //update step to compute the prediction at the current state, and update the estimate of the filter
        void update(double measurement) {
            double prediction = this->current_estimate;
            double prediction_error = this->estimated_error + this->process_noise;

            //update step
            double kalman_gain = prediction_error/(prediction_error + this->sensor_noise);
            this->current_estimate = prediction + (kalman_gain*(measurement - prediction));
            this->estimated_error = (1-kalman_gain)*(prediction_error);
        }

        //method to return the current estimate
        double getPrediction() {
            return this->current_estimate;
        }

    }
    
};
//sample main sequence, users can pass their data in and use this code to create their kalman filter
int main() {
    //you can initialize an input vector that represents the data from your sensor
    // can iterate through the vector of data, updating the filter as we iterate through each element and returning the predictions
    double a;
    double b;
    double c;
    double d;
    //initialize a filter with the appropriate parameters given your data/sensors
    KalmanFilter myFilter(a, b, c, d);
    //vector that represents the readings, replace this placeholder vector with vector of actual datapoints
    vector<double> sensorReadings;
    
    //vector that represents the filtered readings returned by the filter
    vector<double> filteredReadings;

    //for loop to update the filtered vector with each prediction
    for (int i = 0; i < sensorReadings.size(); i++) {
        myFilter.update(sensorReadings[i]);
        double estimate = myFilter.getPrediction();
        filteredReadings.push_back(estimate);
    }
    return 0;
}
