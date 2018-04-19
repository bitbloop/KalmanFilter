#pragma once


#include "SimulatedWorldObject.h"
#include "Graph.h"
#include "Sensor.h"
#include "DeadReckoningPosAcc.h"
#include "Analysis.h"
#include "../kalman/kalman-pos-acc-2d.hpp"
#include "../kalman/kalman-pos-3d.hpp"
#include "../kalman/kalman-pos-acc-3d.hpp"


#define DATA_3D
//#define TRACK_KALMAN_FILTER_POSITION_ONLY

namespace
{
	const auto CAMERA_SENSOR_FREQUENCY{ 1 / 30. };
	const auto ACCELEROMETER_X_SENSOR_FREQUENCY{ 1 / 30. };
	const auto ACCELEROMETER_C_SENSOR_FREQUENCY{ 1 / 30. };
	const auto KALMAN_FILTER_FREQUENCY{ 1 / 30. };

	const double TIME_TO_FAST_FORWARD{ 30. };
	const double FAST_FORWARD_STEP{ 0.001 };

	const int SENSOR_DEAD_RECKONING_HISTORY{ 10 };
	const int FILTER_DEAD_RECKONING_HISTORY{ 10 };

	const auto FUTURE_PREDICTION_TIME{ KALMAN_FILTER_FREQUENCY * 6 };

	const auto GRAPH_DATA_COLLECTION_DURATION{ 60. };

#define LINEAR
#if defined(LINEAR)
	const auto OBJECT_MOVEMENT_TYPE{ SimulatedWorldObject::MotionType::skewed_8 };
	const auto CAMERA_MOVEMENT_TYPE{ SimulatedWorldObject::MotionType::skewed_8 };

	const glm::dvec3 OBJECT_START_POSITION{ glm::dvec3(0, 0, 0) };
	const glm::dvec3 CAMERA_START_POSITION{ glm::dvec3(0, 1, 0) };

	const double OBJECT_SIMULATION_TIME_MULTIPLIER{ 1. };
	const double OBJECT_SIMULATION_VELOCITY_MULTIPLIER{ 1.0 };
	const double CAMERA_SIMULATION_TIME_MULTIPLIER{ 1.0 };
	const double CAMERA_SIMULATION_VELOCITY_MULTIPLIER{ 1.0 };			// linear_no_acc
#else
	const auto OBJECT_MOVEMENT_TYPE{ SimulatedWorldObject::MotionType::skewed_8 };
	const auto CAMERA_MOVEMENT_TYPE{ SimulatedWorldObject::MotionType::skewed_8 };

	const glm::dvec3 OBJECT_START_POSITION{ glm::dvec3(0, 0, 0) };
	const glm::dvec3 CAMERA_START_POSITION{ glm::dvec3(0, 1, 0) };

	const double OBJECT_SIMULATION_TIME_MULTIPLIER{ 1. };
	const double OBJECT_SIMULATION_VELOCITY_MULTIPLIER{ 1.0 };
	const double CAMERA_SIMULATION_TIME_MULTIPLIER{ 1.0 };
	const double CAMERA_SIMULATION_VELOCITY_MULTIPLIER{ 2.5 };		// skewed_8
#endif
};

/**
*	A class which willrun the simulation.
*/

class TrackerSystem
{
public:
	void Init();
	void Loop();

	TrackerSystem();
	~TrackerSystem();

private:
	// Simulation functions
	void Render() const;
	void Update(const double& global_t, const double& delta_t);
	void FastForward(const double& total_time, const double& time_step);

	// Initialization functions
	void InitGL();
	void CreateWorldObjects();
	// Note that the shader which we would like to modify these matrices for, must be bound via glUseProgram prior to calling these functions. This is in addition to passing the shader_id to the function. The passed id is for querying variable adresses.
	void SetViewProjectionMatrix(const glm::vec3& camera_pos, const glm::vec3& camera_target, unsigned int shader_id) const;

	// Control functions
	void StartContinousCapture();
	void StopContinuousCapture();

	// Info functions
	void PrintLegend();

private:
	// Member variables

	// Objects
	SimulatedWorldObject* object_X;
	SimulatedWorldObject* object_C;

	// Sensors
	Sensor<glm::dvec3, SENSOR_DEAD_RECKONING_HISTORY> object_X_position_sensor;						// The sensor which will track the object's position, in world coordinates. To convert to camera space, we subtract the camera ground truth position.
	Sensor<glm::dvec3, SENSOR_DEAD_RECKONING_HISTORY> object_X_acceleration_sensor;					// The sensor which will track the object's acceleration, in world coordinates. To convert to camera space, we subtract the camera acceleration sensor value.
	Sensor<glm::dvec3, SENSOR_DEAD_RECKONING_HISTORY> object_C_acceleration_sensor;					// The sensor which will track the camera's acceleration, in world coordinates.

	// Filter
	double next_filter_update_t;
	const double filter_update_interval_t;
#ifdef DATA_3D
	KalmanFilter_Position3D object_X_filter_pos_xyz;
	KalmanFilter_PositionAcceleration3D object_X_filter_pos_acc_xyz;
#else
	KalmanFilter_PositionAcceleration2D object_X_filter_xy;
#endif
	
	// Filter dead reckoning.
	DeadReckoning_PositionAcceleration<glm::dvec3, FILTER_DEAD_RECKONING_HISTORY> object_X_filter_dead_reckoning;

	// Future prediction.
	glm::dvec3 future_position;				// The object position in the future.
	double future_resolution_time;			// The time we will check our future estimation.
	double last_future_prediction_error;	// The error in the future position prediction.

	// Graph drawing and analysis
	Analysis analysis;

	// Timer to keep track of time since the start of the simulation.
	math::time::Timer global_timer;

	double next_graph_bake_update_t;		// The time we will send the bake command for the graphs
	bool continuous_graph_data_capture;		// A flag which will override next_graph_bake_update_t and will never stop data collection.
	bool stop_graph_data_capture;			// A flag which will force baking.

	unsigned int pointcloud_shader_id;		// The shader id for pointcloud and line rendering.
	unsigned int quad_shader_id;			// The shader id for quad rendering.
};

