# Sensor fusion and state estimation via a Kalman filter

![Kalman](http://radosjovanovic.com/projects/git/kalman_1.jpg)

## Basic Use Pseudocode

```cpp
#include "../kalman/kalman-pos-acc-3d.hpp"

void main()
{
	// X - Object being tracked
	// C - Camera

	// The kalman filter
	KalmanFilter_PositionAcceleration3D object_X_filter_pos_acc_xyz;

	{
		// Update loop, at fixed intervals
		const auto estimate_position{ sensor_X_position };					// Position of X in C space
		const auto estimate_acceleration{ sensor_X_acceleration - sensor_C_acceleration };	// Acceleration of X in C space
	
		Eigen::VectorXd measurementv3d(6); measurementv3d << estimate_position.x, estimate_position.y, estimate_position.z, estimate_acceleration.x, estimate_acceleration.y, estimate_acceleration.z;
		object_X_filter_pos_acc_xyz.Update(measurementv3d);
	}
	{
		// Filter estimation, callable at any time
		const auto filter_estimate{ object_X_filter_pos_acc_xyz.GetEstimate() };
	}
}
```

## Summary

The world consists of a mobile stereo camera, and a mobile object to track. We need to accurately estimate the target object position in camera space.

There are three sensors in place:  
-Stereo camera mounted accelerometer returning the acceleration vector.  
-Target object accelerometer returning the acceleration vector.  
-Target object location in camera space acquired via the stereo camera.  

All sensors have noise in their readings.

## Filter

Using a Kalman filter, the sensors are fused, and the camera space target object position is more accurately estimated.  
The state transition matrix for the filter are the kinematic equations of motion.  

Filter Inputs: target object acceleration vector, target location, in camera space.  
Filter Outputs: target object position, velocity, and acceleration, in camera space.  

A note: To obtain the camera space target object acceleration vector, we subtract the camera acceleration world space vector from the target object world space vector.

## Code details

The code uses the Eigen library for matrix operations.  
All sensor value queries and filter estimation queries, are put through a dead reckoning algorithm before a value is returned. This will help get the best estimate of sensor value readings at any given time.  

The Kalman filter implementation is in src/kalman/kalman-pos-acc-3d.* files.  
Main simulation logic is in the src/tracker/TrackerSystem_simulate.cpp file.  

## Authors

* **Rados Jovanovic** - *Initial work* - [bitbloop](https://github.com/bitbloop)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Hat tip to everyone contributing to science!


