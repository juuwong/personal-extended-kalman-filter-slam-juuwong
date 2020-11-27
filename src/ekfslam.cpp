#include "ekfslam.h"

EKFSLAM::EKFSLAM(unsigned int landmark_size, unsigned int robot_pose_size, float _motion_noise){
    mu = VectorXd::Zero(2*landmark_size + robot_pose_size, 1);
    robotSigma = MatrixXd::Zero(robot_pose_size,robot_pose_size);
    robMapSigma = MatrixXd::Zero(robot_pose_size, 2*landmark_size);
    mapSigma =  INF*MatrixXd::Zero(2*robot_pose_size, 2*robot_pose_size);
    Sigma = MatrixXd::Zero(2*landmark_size + robot_pose_size,2*landmark_size + robot_pose_size);
    Sigma.topLeftCorner(robot_pose_size, robot_pose_size) = robotSigma;
    Sigma.topRightCorner(robot_pose_size, 2*landmark_size) = robMapSigma; 
    Sigma.bottomLeftCorner(2*landmark_size,robot_pose_size) = robMapSigma.transpose(); 
    Sigma.bottomRightCorner(2*landmark_size, 2*landmark_size) = mapSigma;

    Q = MatrixXd::Zero(2*landmark_size + robot_pose_size,2*landmark_size + robot_pose_size);
    Q.topLeftCorner(3,3) << _motion_noise, 0, 0,
        0, _motion_noise, 0,
        0, 0, _motion_noise/10;

    observedLandmarks.resize(landmark_size);
    fill(observedLandmarks.begin(), observedLandmarks.end(), false);
}

void EKFSLAM::Prediction(const OdoReading& motion){
    double angle = mu(2);
    MatrixXd Gt = MatrixXd(3,3);
    Gt << 1, 0, -motion.t*sin(angle + motion.r1),
        0, 1, motion.t*cos(angle + motion.r1),
        0, 0, 0;
    
    mu(0) = mu(0) + motion.t*cos(angle + motion.r1);
    mu(1) = mu(1) + motion.t*sin(angle + motion.r1);
    mu(2) = mu(2) + motion.r1 + motion.r2;

    int s = Sigma.cols();
    Sigma.topLeftCorner(3,3) = Gt*Sigma.topLeftCorner(3,3) * Gt.transpose();
    Sigma.topRightCorner(3, s-3) = Gt*Sigma.topLeftCorner(3, s-3);
    Sigma.bottomLeftCorner(s-3, 3) = Sigma.topLeftCorner(3, s-3).transpose();
    Sigma = Sigma + Q;

}

void EKFSLAM::Correction(const vector<LaserReading>& observation){
    int s = observation.size();
    VectorXd Z = VectorXd::Zero(2*s);
    VectorXd eZ = VectorXd::Zero(2*s);

    int m = observedLandmarks.size();
    MatrixXd H = MatrixXd::Zero(2*s, 2*m + 3);

    for (int i = 0; i < s; i++){
        auto& r = observation[i];
        if (!observedLandmarks[r.id - 1]){
            mu(2*r.id + 1) = mu(0) + r.range*cos(mu(2) + r.bearing);
            mu(2*r.id + 2) = mu(1) + r.range*sin(mu(2) + r.bearing);
            observedLandmarks[r.id - 1] = true;
        }
        Z(2*i) = r.range;
        Z(2*i + 1) = r.bearing;
        double dx = mu(2*r.id + 1) - mu(0);
        double dy = mu(2*r.id + 2) - mu(1);
        double q = sqrt(pow(dx, 2) + pow(dy,2));
        eZ(2*i) = q;
        eZ(2*i + i) = atan2(dy, dx) - mu(2);
        H.block<2,3>(2*i, 0) << -q*dx/pow(q,2) , -q*dy/pow(q,2), 0,
            dy/pow(q,2), -dx/pow(q, 2), -1;
        H.block<2,2>(2*i, 2*r.id + 1)<< q*dx/pow(q,2), q*dy/pow(q,2),
            -dy/pow(q,2), dx/pow(q,2);
    }
    R = MatrixXd::Identity(2*s, 2*s)*0.01;
    MatrixXd k = Sigma*H.transpose()*(H*Sigma*H.transpose() + R);
    VectorXd diff = Z - eZ;
    mu = mu + k*diff;
    Sigma = Sigma - k*H*(H*Sigma*H.transpose() + R);
}
