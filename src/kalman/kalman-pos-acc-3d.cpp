
#include "kalman-pos-acc-3d.hpp"

#include <cmath>
#include <iostream>

const int KalmanFilter_PositionAcceleration3D::n{ 9 };	// Position, velocity, acceleration
const int KalmanFilter_PositionAcceleration3D::m{ 6 };	// Position, acceleration

// n - states, sensor inputs
// m - measurements, filter output
// x_predicted - n, 1
// P - n, n
// A - n, n
// H - m, n
// R - m, m
// Q - n, n
// I - n, n

KalmanFilter_PositionAcceleration3D::KalmanFilter_PositionAcceleration3D(double dt) :
	dt(dt),
	x_predicted(n),
	P(n, n),
	A(n, n),
	H(m, n),
	R(m, m),
	Q(n, n),
	I(n, n)
{
	// Initial state.
	x_predicted.setZero();

	// Dynamic matrix.
	A << 1.0, 0.0, 0.0, dt, 0.0, 0.0, 1 / 2.0*dt*dt, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 1 / 2.0*dt*dt, 0.0,
		0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0, 1 / 2.0*dt*dt,
		0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

	A.setIdentity();

	// Measurement matrix.
	H << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

	// Measurement Noise Covariance  R
	const auto ra = .003; 		// Noise of Acceleration Measurement
	const auto rp = .006; 		// Noise of Position Measurement
	R << rp, 0., 0.,	0., 0., 0.,				// The more variance in the measurement R, the less informative the measurement will be, and K will be smaller.
		0., rp, 0.,		0., 0., 0.,
		0., 0., rp,		0., 0., 0.,
		0., 0., 0.,		ra, 0., 0.,
		0., 0., 0.,		0., ra, 0.,
		0., 0., 0.,		0., 0., ra;

	// Process Noise Covariance Matrix  Q
	// The larger the state covariance, the more we trust the measurement, and K will be larger.

#define LINEAR
#if defined(LINEAR)
	const auto sa = 0.0001;		// linear_no_acc
#else
	const auto sa = 1.8;		// skewed_8
	//const auto sa = 1.2;		// one is constant
#endif


	Eigen::VectorXd G(n);
	G << 1 / 2.0*dt*dt,
		1 / 2.0*dt*dt,
		1 / 2.0*dt*dt,
		dt,
		dt,
		dt,
		1.0,
		1.0,
		1.0;

	Q = (G*sa).asDiagonal();

	// Initial uncertanty.
	Eigen::Matrix< double, n, 1> v;
	v << 10., 10., 10., 1., 1., 1., 1., 1., 1.;
	P = v.asDiagonal();

	// Identity Matrix I
	I.setIdentity();
}

void KalmanFilter_PositionAcceleration3D::Update(const Eigen::VectorXd& y)
{
	// The Prediction Step:		
	x_predicted = A * x_predicted;				// Project the state ahead
	P = A * P * A.transpose() + Q;						// Project the error covariance ahead

														// The Correction Step:
	auto S = H * P * H.transpose() + R;
	auto K = P * H.transpose() * S.inverse();			// Compute the Kalman Gain

	x_predicted += K * (y - (H*x_predicted));		// Update the estimate via y
	P = (I - K*H)*P;									// Update the error covariance
}
