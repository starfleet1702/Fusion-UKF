#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
	
	/**
  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  // Dimensionality of x_
  n_x_ = 5;
  
  // Dimensionality of augmented x_
  n_aug_ = 7;
  
  // lambda : Spreadness from mean for sigma points
  lambda_ = 3-n_aug_;
  
  // Number of Sigma Points
  n_sigma_ = 2*n_aug+1;
  
  // Matrix to store Sigma points
  MatrixXd xSig_aug_(n_aug_,n_sigma_);
  
  n_z_radar_ = 3;
  
  n_z_lidar_ = 2;

  //the current NIS for radar
  NIS_radar_ = 0.0;

  //the current NIS for laser
  NIS_laser_ = 0.0;
	
	
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  //x_ = []
  x_ = VectorXd(n_x_);

  // initial covariance matrix - using identity matrix as initial value
  P_ = MatrixXd::Identity(n_x_ ,n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  weights = VectorXd::Zero(this.n_sigma_);
  this.initWeights();
  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  
  
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  Estimating the object's location. Modifying the state
  vector, x_. Predicting sigma points, the state, and the state covariance matrix.
  */
  
  this.GenerateSigmaPoints();
  this.predictSigmaPoints(delta_t);
  this.predictMeanAndCovariance();
  
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  // Lidar provides px and py only
  VectorXd z =
  double px = meas_package.raw_measurements_(0);
  double py = meas_package.raw_measurements_(1);
  
  // Defining measurement vector z
  VectorXd z(2);
  z<<px,
	 py;
	 
  
  
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  
  VectorXd z = meas_package.raw_measurements_;  
  MatrixXd zSig = this.predictSigmaPointsInMeasurementSpace();
  
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug+1; i++) {
      z_pred = z_pred + weights(i) * zSig.col(i);
  }
  
  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = zSig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
	//z_diff(1) = atan2(sin(z_diff(1)),cos(z_diff(1)));

    S = S + weights(i) * z_diff * z_diff.transpose();
  }
  
  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(this.n_z_radar_,this.n_z_radar_);
  R <<    std_radr*std_radr, 0, 0,
          0, std_radphi*std_radphi, 0,
          0, 0,std_radrd*std_radrd;
  S = S + R;
  
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(this.n_x_, this.n_z_radar_);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < this.n_sigma_; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = xSig_pred.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //calculate NIS
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
	
	
  //update state mean and covariance matrix
  this.x_ = this.x_ + K * z_diff;
  this.P_ = this.P_ - K*S*K.transpose();
  
}

void UKF::GenerateSigmaPoints(){
	// Defining x_aug , augmnting x
	VectorXd x_aug(this.n_aug_);
	x_aug.fill(0.0);
	x_aug.head(this.n_x_) = this.x_;
	
	MatrixXd P_aug(this.n_aug_,this.n_aug_);
	
	// Intializing P_aug
	P_aug.topLeftCorner(this.n_x_,this.n_x_)=this.P_;
	P_aug(n_x,n_x)=this.std_a_*this.std_a_;
	P_aug(n_x+1,n_x+1)=this.std_yawdd_*this.std_yawdd_;
	
	// Computing the square root of P
	MatrixXd A = P_aug_.llt().matrixL();
	double sqrt_alpha  = sqrt(this.lambda_+this.n_aug_);
	
	// First Sigma point will be mean itself
	this.xSig_aug_.col(0) = this.x_;
	VectorXd v ;
	
	for(unsigned int i=0;i<this.n_aug_;i++){
		v = sqrt_alpha*A.col(i)
		this.xSig_aug_.col(i+1) = x_aug + v;
		this.xSig_aug_.col(i+n_aug+1) = x_aug - v;
	}
	
}

void UKF::predictSigmaPoints(double delta_t){
	VectorXd dx = VectorXd::Zero(this.n_x_);
	VectorXd vk = VectorXd::Zero(this.n_x_);
	
	for (int i = 0;i<this.n_sigma_;i++){
		VectorXd x = xSig_aug_.col(i);
		
		double px = x(0);
		double py = x(1);
		double v = x(2);
		double yaw = x(3);
		double yaw_d = x(4);
		double va = x(5);
		double v_yaw_dd = x(6);
		
		double delta_t_sqr = delta_t*delta_t;
		
		// populating vk
		vk<<0.5*va*cos(yaw)*delta_t_sqr,
			0.5*va*sin(yaw)*delta_t_sqr,
			va*delta_t,
			0.5*v_yaw_dd*delta_t_sqr,
			v_yaw_dd*delta_t;
		
		if(fabs(yaw_d)<0.0001){
			// populating dx with linear equation as turning angle is smaller
			double v_dt = v*delta_t;
			dx<<v_dt*cos(yaw),
				v_dt*sin(yaw),
				0,
				yaw_d*delta_t,
				0;
		}else{
			// populating dx
			double v_by_yaw_d  = (v/yaw_d);
			dx<<v_by_yaw_d*(sin(yaw+yaw_d*delta_t)-sin(yaw)),
				v_by_yaw_d*(-cos(yaw+yaw_d*delta_t)+cos(yaw)),
				0,
				yaw_d*delta_t,
				0;
		}
		
		this.xSig_pred_.col(i)= x.head(this.n_x_)+dx+vk;
	}
	
	
	
}

void predictMeanAndCovariance(){
	
	// predicting mean
	this.x_.fill(0.0);
	for(int i=0;i<this.n_sigma_;i++){
		this.x_ = this.x_ + weights*this.xSig_pred_.col(i);
	}
	
	// predicting co-variance
	this.P_.fill(0.0);
	for(int i=0;i<this.n_sigma_;i++){
		VectorXd diff = xSig_pred_.col(i) - this.x_;
		// Angle Normalization
		x_diff(3) = atan2(sin(x_diff(3)),cos(x_diff(3)));
		
		this.P_ = this.P_ + weights(i) * x_diff * x_diff.transpose();
	}
}

MatrixXd predictSigmaPointsInMeasurementSpace(){
	MatrixXd zSig = MatrixXd::Zeros(this.n_z_radar_,this.n_sigma_);
	
	for(int i=0;i<this.n_sigma_;i++){
		// extract values for better readibility
		double p_x = Xsig_pred(0,i);
		double p_y = Xsig_pred(1,i);
		double v  = Xsig_pred(2,i);
		double yaw = Xsig_pred(3,i);
		
		double v1 = cos(yaw)*v;
		double v2 = sin(yaw)*v;
		
		// measurement model
		double rho = sqrt(p_x*p_x + p_y*p_y);
		zSig(0,i) = rho;                        //r
		zSig(1,i) = atan2(p_y,p_x);             //phi
		zSig(2,i) = (p_x*v1 + p_y*v2 ) / rho;   //r_dot
	}
	return zSig;
}

void initWeights(){
	double denom = (this.lambda_+this.n_aug_);
	weights(0) = this.lambda_/denom;
	double weight_other =  0.5/denom;
	for(int i=0;i<this.n_sigma_;i++){
		weights(i) = weight_other;
	}
}