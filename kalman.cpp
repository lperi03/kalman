#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
//Kalman Filter class definition
class KalmanFilter{
    private:
    VectorXD x;
    MatrixXD A;
    MatrixXd P;    
    MatrixXd B;  
    MatrixXd C;  
    MatrixXd Q;  
    MatrixXd R;      

    public:

    KalmanFilter(int OBS_SIZE, int STATE_SIZE, int CONTROL_INPUT)
    // A: State transition matrix (identity matrix of state size)
    A = MatrixXD::Identity(STATE_SIZE, STATE_SIZE);

    //B: Control input matrix
    B = MatrixXD::Zero(STATE_SIZE, CONTROL_INPUT)

    // P: Initial State covariance matrix (Identity matrix of state size)
    P = MatrixXD::Identity(STATE_SIZE, STATE_SIZE);

    //x: initial state vector, will be updated via measurements
    x = VectorXD::Zero(n);

    //C: Measurement/Observation matrix
    C = MatrixXD::Zero(OBS_SIZE, STATE_SIZE)

    //Q: Process noise covariance matrix
    MatrixXD::Identity(STATE_SIZE, STATE_SIZE)

    //R: Observation noise covariance matrix
    R = MatrixXd::Identity(STATE_SIZE, OBS_SIZE);

    //K: Kalman gain matrix
    K = MatrixXD::Zero(n, m);

    //predict state and covariance based on previous state/covariance and control vector
    void predict(VectorXD& u) {
        x = (A * x) + (B * u)
        P = A * P * A_.transpose() + Q_;
    }


    void update(VectorXD& y) {
        //computing kalman gain
        K = P * C.transpose() * (C * P * C.transpose() + R).inverse();

        //updating state/covariance based on new measurement
        x = x + K * (y - C * x);
        P = (I - K * C) * P;
    }

    //returns current state vector
    VectorXD getCurrentState() {
        return x;
    }

}

//sample loop for using this filter
int main() {
    int OBS_SIZE = 2; // Observation Dimension
    int STATE_SIZE = 2 // State dimension
    int CONTROL_INPUT = 0; //let us assume that there is no control vector
    KalmanFilter Filter1(OBS_SIZE, STATE_SIZE, CONTROL_INPUT);

    //Define/change your matrices depending on system requirements
    //Once finalizing matrices, we can use a sample set of readings to test the filter

    //Updating while still getting measurements
    while (true) {
        double measurement1;
        double measurement2;

        VectorXD measurement(OBS_SIZE);
        measurement << measurement1 << measurement2;



        //defining control input vector (vector of 0s)
        Vector control = VectorXD::Zero(CONTROL_INPUT);


        //predict next state, covariance 
        Filter1.predict(control);

        //update step
        Filter1.update(measurement)

        //state estimate vector
        VectorXD state_estimate = Filter1.getCurrentState() 
        }
    
}