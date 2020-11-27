#ifndef EKFSLAM_H
#define EKFSLAM_H
#include <vector>
#include "../include/sensor_info.h"
#include "../include/common.h"
#include "../include/Eigen/Dense"
#define INF 1000 // what is this?

class EKFSLAM {
 private:
    // Covariance Matrix for robot state variables
    Eigen::MatrixXd robotSigma;
    // Covariance Matrix for robot to landmarks
    Eigen::MatrixXd robMapSigma;
    // Covariances of landmark positions wrt to each other
    Eigen::MatrixXd mapSigma;
    // Full Covariance Matrix
    Eigen::MatrixXd Sigma;
    // Full State Vector
    Eigen::VectorXd mu;
    // Noise Matrix due to motion
    Eigen::MatrixXd R;
    // Noise Matrix due to sensors
    Eigen::MatrixXd Q;
    // Vector of observed landmarks
    vector<bool> observedLandmarks;

 public:
    // Default Constructor
    EKFSLAM();

    // Standard Destructor
    virtual ~EKFSLAM();

    /****** TODO *********/
    // Overloaded Constructor -> Initialize
    void Initialize(unsigned int landmark_size , unsigned int robot_pose_size = 3 , float _motion_noise = 0.1);

    /****** TODO *********/
    // Description: Prediction step for the EKF based off an odometry model
    // Inputs:
    // motion - struct with the control input for one time step
    void Prediction(const OdoReading& motion);


    /****** TODO *********/
    // Description: Correction step for EKF
    // Inputs:
    // observation - vector containing all observed landmarks from a laser scanner
    void Correction(const vector<LaserReading>& observation);

    // simplify the main.cpp by combining the prediction and correction steps
    void KalmanRun(const Record& record);

    VectorXd getMu() const {
        return mu;
    }

    MatrixXd getSigma() const {
        return Sigma;
    }
};

#endif  // EKFSLAM_H
