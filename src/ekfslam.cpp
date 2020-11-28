#include "ekfslam.h"

// constructor
EKFSLAM::EKFSLAM() {}

// destructor
EKFSLAM::~EKFSLAM() {}
/*
Inputs:
L - number of landmarks on the map
R - number of state variables to track on the robot
motion_noise - amount of noise to add due to motion
*/
/*
ie. first sensor dataset is visible to landmarks 1 and 2 with given odometry  
ODOMETRY r1 t r2
SENSOR 1 range_1 bearing_1
SENSOR 2 range_2 bearing_2
updated_mu = [mu(0) , mu(1) , mu(2) , mu(3) , mu(4)] + [t*cos(mu(4) + r1) , t*sin(mu(4) + r1) , t*cos(mu(4) + r1) , t*sin(mu(4) + r1) , r1 + r2]
*/
void EKFSLAM::Initialize(unsigned int landmark_size, unsigned int robot_pose_size, float _motion_noise) {

    int L = landmark_size;
    int R = robot_pose_size;
    float motion_noise =_motion_noise;
    int length = 2 * L + R;

    mu = VectorXd::Zero(length, 1); // initialize state vector mu as zero vector of size 2*L+R for x and y parameters per visible landmark and each robot state variable (three)

    // initialize entire EKF matrix with proper dimensions according to slide 12
    robotSigma = MatrixXd::Zero(R,R); // covariance for robot state variables
    robMapSigma = MatrixXd::Zero(R, 2*L); // covariance for robot to landmarks
    mapSigma = INF*MatrixXd::Zero(2*R, 2*R); // covariance between landmarks

    Sigma = MatrixXd::Zero(length,length);
    Sigma.topLeftCorner(R, R) = robotSigma;
    Sigma.topRightCorner(R, 2*L) = robMapSigma; 
    Sigma.bottomLeftCorner(2*L,R) = robMapSigma.transpose(); 
    Sigma.bottomRightCorner(2*L, 2*L) = mapSigma;

    // from Henry's notes, Rt is the Additive Gaussian White Noise (AWGN) extrinsic from sensor
    Rt = MatrixXd::Zero(length,length);
    Rt.topLeftCorner(3,3) << _motion_noise, 0, 0,
        0, _motion_noise, 0,
        0, 0, _motion_noise/10;

    observedLandmarks.resize(L);
    fill(observedLandmarks.begin(), observedLandmarks.end(), false);
}

void EKFSLAM::Prediction(const OdoReading& motion){
    // UPDATE STATE VECTOR mu ****

    double angle = mu(2);

    mu(0) = mu(0) + motion.t*cos(angle + motion.r1);
    mu(1) = mu(1) + motion.t*sin(angle + motion.r1);
    mu(2) = mu(2) + motion.r1 + motion.r2;
    // COMPUTE JACOBIAN ****
    
    // Jacobian = Jacobian of the motion + I(3)
    MatrixXd Gt = MatrixXd(3,3);
    Gt << 1, 0, -motion.t*sin(angle + motion.r1),
        0, 1, motion.t*cos(angle + motion.r1),
        0, 0, 0;
    
    // UPDATE COVARIANCE MATRICES ****
    // Sigma_updated = Gt*Sigma*Gt.T + Rt, from slide 17
    
    int s = Sigma.cols();
    Sigma.topLeftCorner(3,3) = Gt*Sigma.topLeftCorner(3,3) * Gt.transpose();
    Sigma.topRightCorner(3, s-3) = Gt*Sigma.topLeftCorner(3, s-3);
    Sigma.bottomLeftCorner(s-3, 3) = Sigma.topLeftCorner(3, s-3).transpose();
    Sigma = Sigma + Rt;

}

void EKFSLAM::Correction(const vector<LaserReading>& observation){
    int s = observation.size();
    VectorXd Z = VectorXd::Zero(2*s);
    VectorXd eZ = VectorXd::Zero(2*s);

    int m = observedLandmarks.size();
    MatrixXd H = MatrixXd::Zero(2*s, 2*m + 3);

    for (int i = 0; i < s; i++){
        auto& r = observation[i];
        // mu'(j_x) = x' + rt * cos(θ' + θj)
        // mu'(j_y) = y' + rt * sin(θ' + θj)
        if (!observedLandmarks[r.id - 1]){
            mu(2*r.id + 1) = mu(0) + r.range*cos(mu(2) + r.bearing);
            mu(2*r.id + 2) = mu(1) + r.range*sin(mu(2) + r.bearing);
            observedLandmarks[r.id - 1] = true;
        }
        Z(2*i) = r.range;
        Z(2*i + 1) = r.bearing;
        //δ_x = mu'(j_x) - x'
        //δ_y = mu'(j_y) - y'
        double dx = mu(2*r.id + 1) - mu(0);
        double dy = mu(2*r.id + 2) - mu(1);
        double q = sqrt(pow(dx, 2) + pow(dy,2));
        eZ(2*i) = q;
        eZ(2*i + i) = atan2(dy, dx) - mu(2);
        H.block<2,3>(2*i, 0) << -q*dx/pow(q,2) , -q*dy/pow(q,2), 0, dy/pow(q,2), -dx/pow(q, 2), -1;
        H.block<2,2>(2*i, 2*r.id + 1)<< q*dx/pow(q,2), q*dy/pow(q,2),-dy/pow(q,2), dx/pow(q,2);
    }
    Qt = MatrixXd::Identity(2*s, 2*s)*0.01; // AWGN
    MatrixXd k = Sigma*H.transpose()*(H*Sigma*H.transpose() + Qt);
    VectorXd diff = Z - eZ;
    mu = mu + k*diff;
    Sigma = Sigma - k*H*(H*Sigma*H.transpose() + Qt);
}

void KalmanRun(const Record& record){
    // Prediction() inputs odometry readings
    Prediction(record.odo);
    // Correction() inputs laser scan data
    Correction(record.scans);
}