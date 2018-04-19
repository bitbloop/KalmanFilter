#pragma once

#include <Eigen/Dense>
#include <list>

/**
* A Simple Kalman filter.
*   A - System dynamics matrix
*   C - Output matrix
*   Q - Process noise covariance
*   R - Measurement noise covariance
*   P - Estimate error covariance
* The inputs are 2d position, and 2d acceleration vector.
* Outputs are a 2d position, 2d velocity, and 2d acceleration vectors.
*/

class KalmanFilter_PositionAcceleration2D {

public:
	/**
	* Initialize the kalman filter.
	* dt - a filter time step.
	*/
	KalmanFilter_PositionAcceleration2D(const double dt);

	/** Update the estimated state based on measured values. */
	void Update(const Eigen::VectorXd& y);

	/** Returns the filter estimation. */
	inline Eigen::VectorXd GetEstimate() { return x_predicted; }
	
private:
	static const int n, m;				// The size of the filter output and input vectors.
	const double dt;					// The filter time step size.

	Eigen::VectorXd x_predicted;		// Last predicted value.
	Eigen::MatrixXd P, A, H, R, Q, I;	// Kalman filter matrices.
};
