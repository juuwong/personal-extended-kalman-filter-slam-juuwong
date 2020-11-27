#include "ekfslam.h"

// constructor
EKFSLAM::EKFSLAM() {}

// destructor
EKFSLAM::~EKFSLAM() {}

// Inputs:
// landmark_size - number of landmarks on the map
// robot_pose_size - number of state variables to track on the robot
// motion_noise - amount of noise to add due to motion
void EKFSLam::Initialize(unsigned int landmark_size, unsigned int robot_pose_size, float _motion_noise) {
  
    int L = landmark_size;
    int R = rob_pose_size;
    float motion_noise =_motion_noise;

    // initialize state vector mu as zero vector of size 2*L+R for x and y parameters per visible landmark and each robot state variable (three)
    /*
    ie. first sensor dataset is visible to landmarks 1 and 2 with given odometry  
    ODOMETRY r1 t r2
    SENSOR 1 range_1 bearing_1
    SENSOR 2 range_2 bearing_2
    updated_mu = [mu(0) , mu(1) , mu(2) , mu(3) , mu(4)] + [t*cos(mu(4) + r1) , t*sin(mu(4) + r1) , t*cos(mu(4) + r1) , t*sin(mu(4) + r1) , r1 + r2]
    */
    int length = 2 * L + R;
    mu = VectorXd::Zero(length, 1);
    // initialize entire EKF matrix with proper dimensions according to slide 12
    // Sigma.upperLeft - covariance for robot state variables
    // Sigma.upperRight - covariance for robot to landmarks
    // Sigma.lowerLeft - transpose of Sigma.upperRight
    // Sigma.lowerRight - covariance between landmarks
    Sigma = MatrixXd::Zero(length, length); // (2*L+3) x (2*L+3)
    Sigma.upperLeft(R, R) = MatrixXd::Zero(R, R); // 3 x 3
    Sigma.upperRight(R, length - R) = MatrixXd::Zero(R, length - R); // 3 x 2*L
    Sigma.lowerLeft(length - R, R) = MatrixXd::Zero(R, length - R).transpose(); // 2*L x 3
    Sigma.lowerRight(length - R, length - R) = MatrixXd::Identity(length - R, length - R); // 2*L x 2*L

    // from Henry's notes, Rt is the Additive Gaussian White Noise (AWGN) extrinsic from sensor
    // Sigma_updated = Signma + Rt from slides
    // Sigma_updated = Signma - Rt from Henry's notes
    Rt = MatrixXd::Zero(length , length);
    Rt = {
    {motion_noise , 0 , 0} ,
    {0 , motion_noise , 0} ,
    {0 , 0 , motion_noise}
    };
}

void EKFSLAM::Prediction(const OdoReading& motion) {
    // UPDATE STATE VECTOR mu ****

    // initialize the three state variables as type float from sensor_info.h
    float r1 = motion.r1;
    float t = motion.t;
    float r2 = motion.r2;
    // assuming the robot is traveling at constant velocity of 1 m/s across time t
    mu(0) = mu(0) + t*cos(mu(2) + r1); // x' = x + cos(θ + θ1)*t
    mu(1) = mu(1) + t*sin(mu(2) + r1); // y' = y + sin(θ + θ1)*t
    mu(2) = mu(2) + r1 + r2; // θ' = θ + θ1 + θ2
    
    // COMPUTE JACOBIAN ****
    
    // Jacobian = Jacobian of the motion + I(3) 
    // Gt = I(3) + partial derivative of mu wrt θ (I think) 
    MatrixXd Gt = MatrixXd(3,3);
    Gt = {
    {1 , 0 , -t*sin(mu(2) + r1)} ,
    {0 , 1 , t*cos(mu(2) + r1)} ,
    {0 , 0 , 1}
    };

    // UPDATE COVARIANCE MATRICES ****

    // Sigma_updated = Gt*Sigma*Gt.T + Rt, Rt = 0?, from slide 17

    Sigma.upperLeft(R, R) = Gt * Sigma.upperLeft(R, R) * Gt.transpose();
    Sigma.upperRight(R, length - R) = Gt * Sigma.upperRight(R, length - R);
    Sigma.lowerLeft(length - R, R) = Sigma.upperRight(R, length - R).transpose();
    // Sigma.lowerLeft remains unchanged since landmark to landmark info remains consistent
}

void EKFSLAM::Correction(const vector<LaserReading>& observation) {
    // this whole assignment is absurdly long
    // this part especially
    int n = observation.size();
 
    for (int i = 0; i < n; i++) {
        int landmarkId = observation.id;
        float range = observation.range; // rt
        float bearing = observation.bearing; // θj
        
        j=ct
        if (landmark not observed yet){
            // mu'(j_x) = x' + rt * cos(θ' + θj)
            mu(x of landmark j) = mu(0) + range * cos(mu(2) + bearing)
            // mu'(j_y) = y' + rt * sin(θ' + θj)
            mu(y of landmark j) = mu(1) + range * cos()
        }
        //δ_x = mu'(j_x) - x'
        dx = mu(x of landmark j) - mu(0);
        //δ_y = mu'(j_y) - y'
        dy = mu(y of landmark j) - mu(1);
        // q = δ_T * δ is the Gramian matrix. How do we compute?
        Z(first row of landmark) = sqrt(q)
        Z(second row of landmark) = atan2(dy,dx) - mu(2)
        F = MatrixXd::Identity(2*n,2*n) // with some zero columns
        H = (1/q)*lowH*F
        Qt = MatrixXd::Identity(2*n,2*n) * //AWGN 
        K = Sigma*H.transpose()*((H*Sigma*H.transpose() + Qt).inverse())
        mu = mu + K * (dz)
        
    }
}

void KalmanRun(const Record& record){
    // Prediction() inputs odometry readings
    Prediction(record.odo);
    // Correction() inputs laser scan data
    Correction(record.scans);
}