#include "TrackerSystem.h"
#include "TrackerSystemInputHandler.h"

/** Do the simulation loop. */
void TrackerSystem::Loop()
{
	{
		// Fast forward the simulation at the start.
		math::time::Timer fast_forward_timer;
		FastForward(TIME_TO_FAST_FORWARD, FAST_FORWARD_STEP);
		double fast_forward_end_time{ fast_forward_timer.elapsed() };
		std::cout << "Fast forward for " << TIME_TO_FAST_FORWARD << "s took " << fast_forward_end_time << "s." << std::endl;
	}

	// Rest the global simulation time.
	global_timer.reset();
	double last_frame_time{ global_timer.elapsed() };

	// Until we signal for the window to close
	while (!::renderer::window::should_window_close()) {
		const auto global_t{ global_timer.elapsed() };
		const auto delta_t{ global_t - last_frame_time };

		last_frame_time = global_t;

		Update(global_t, delta_t);
		Render();

		glFinish();
		::renderer::window::pool_events_and_swap_buffers();
	}
}


/** Perform the simulation update step. */
void TrackerSystem::Update(const double& global_t, const double& delta_t)
{
	// Upate the world objects
	object_X->Update(global_t, delta_t);
	object_C->Update(global_t, delta_t);


	// -----------------------------------------------------------------------------------
	// Give the sensors the new ground truth variable.
	object_X_acceleration_sensor.SetGroundTruth(object_X->GetAcceleration());
	object_X_acceleration_sensor.QueryForSensorValueChangedEvent(global_t);

	object_C_acceleration_sensor.SetGroundTruth(object_C->GetAcceleration());
	object_C_acceleration_sensor.QueryForSensorValueChangedEvent(global_t);

	// Only set the sensor ground truth if the object is within the 3m range
	const auto dXC{ glm::pow(object_X->GetPosition() - object_C->GetPosition(),{ 2, 2, 2 }) };
	if (dXC.x + dXC.y + dXC.z <= 9)
	{
		object_X_position_sensor.SetGroundTruth(object_X->GetPosition());
		object_X_position_sensor.QueryForSensorValueChangedEvent(global_t);
	}


	// -----------------------------------------------------------------------------------
	// If the interval to update the filter is here, update the filter with the new sensor values.
	if (next_filter_update_t <= global_t && next_filter_update_t != -1)
	{
		next_filter_update_t = global_t + filter_update_interval_t;

		{
			const auto estimate_X_position{ object_X_position_sensor.GetEstimate(global_t) };
			const auto estimate_X_acceleration{ object_X_acceleration_sensor.GetEstimate(global_t) };
			const auto estimate_C_acceleration{ object_C_acceleration_sensor.GetEstimate(global_t) };

			// convert the measurements to camera space
			const auto estimate_position{ estimate_X_position - object_C->GetPosition() };
			const auto estimate_acceleration{ estimate_X_acceleration - estimate_C_acceleration };

			// The code below will get and set proper filter values depending on the chosen model.

#ifdef DATA_3D
			{
				Eigen::VectorXd measurementv3d(3); measurementv3d << estimate_position.x, estimate_position.y, estimate_position.z;
				object_X_filter_pos_xyz.Update(measurementv3d);
			}
			{
				Eigen::VectorXd measurementv3d(6); measurementv3d << estimate_position.x, estimate_position.y, estimate_position.z, estimate_acceleration.x, estimate_acceleration.y, estimate_acceleration.z;
				object_X_filter_pos_acc_xyz.Update(measurementv3d);
			}
#else
			{
				Eigen::VectorXd measurementv2d(4); measurementv2d << estimate_position.x, estimate_position.y, estimate_acceleration.x, estimate_acceleration.y;
				object_X_filter_xy.Update(measurementv2d);
			}
#endif
		}

#ifdef DATA_3D
		{
			// Update the dead reckoning algorithm
#ifdef TRACK_KALMAN_FILTER_POSITION_ONLY
			const auto filter_estimate{ object_X_filter_pos_xyz.GetEstimate() };
			const auto glm_filter_estimate_pos{ glm::dvec3({ filter_estimate[0], filter_estimate[1], filter_estimate[2] }) };
			object_X_filter_dead_reckoning.SetSensorReading(glm_filter_estimate_pos, glm::dvec3(0), global_t);
#else
			const auto filter_estimate{ object_X_filter_pos_acc_xyz.GetEstimate() };
			const auto glm_filter_estimate_pos{ glm::dvec3({ filter_estimate[0], filter_estimate[1], filter_estimate[2] }) };
			const auto glm_filter_estimate_acc{ glm::dvec3({ filter_estimate[3], filter_estimate[4], filter_estimate[5] }) };
			object_X_filter_dead_reckoning.SetSensorReading(glm_filter_estimate_pos, glm_filter_estimate_acc, global_t);
#endif

		}
#else
		// Update the dead reckoning algorithm
		const auto filter_estimate{ object_X_filter_xy.GetEstimate() };
		const auto glm_filter_estimate_pos{ glm::dvec3({ filter_estimate[0], filter_estimate[1], 0 }) };
		const auto glm_filter_estimate_acc{ glm::dvec3({ filter_estimate[4], filter_estimate[5], 0 }) };

		object_X_filter_dead_reckoning.SetSensorReading(glm_filter_estimate_pos, glm_filter_estimate_acc, global_t);
#endif
	}


	// -----------------------------------------------------------------------------------
	// Update and bake the graphs

	// Mark the next filter update interval if it is the first iteration of the loop
	if (next_graph_bake_update_t == 0) next_graph_bake_update_t = global_t + GRAPH_DATA_COLLECTION_DURATION;

	// Add more measurements to the graphs if the graphs were not baked and it is the allowed time to record a new measurement.
	if (!analysis.IsBaked() && (global_t < next_graph_bake_update_t || continuous_graph_data_capture) && !stop_graph_data_capture)
	{
		const auto ground_truth_position_X{ object_X->GetPosition() };
		const auto ground_truth_position_C{ object_C->GetPosition() };
		const auto sensor_read_position{ object_X_position_sensor.GetEstimate(global_t) - object_C->GetPosition() };
		const auto dead_reckoning_position{ object_X_filter_dead_reckoning.GetPositionEstimate(global_t) };
		const auto sensor_read_acceleration{ object_X_acceleration_sensor.GetEstimate(global_t) - object_C_acceleration_sensor.GetEstimate(global_t) };
		const auto dead_reckoning_acceleration{ object_X_filter_dead_reckoning.GetAccelerationEstimate(global_t) };

		// Future position estimation
		if (global_t >= future_resolution_time)
		{
			const glm::dvec3 d{ (ground_truth_position_X - ground_truth_position_C) - future_position };
			last_future_prediction_error = { (d.x*d.x + d.y*d.y + d.z*d.z) };
			future_resolution_time = global_t + FUTURE_PREDICTION_TIME;
			future_position = object_X_filter_dead_reckoning.GetPositionEstimate(global_t + FUTURE_PREDICTION_TIME);
		}

		analysis.AddPoint(ground_truth_position_X, ground_truth_position_C, sensor_read_position, sensor_read_acceleration, dead_reckoning_position, dead_reckoning_acceleration, last_future_prediction_error);
	}
	// If we are not capturing, and also the graphs are not baked, bake them.
	else if (!analysis.IsBaked())
	{
		analysis.BakeGraphs(pointcloud_shader_id);
	}
}

/** Fast forward the soimulation */
void TrackerSystem::FastForward(const double & total_time, const double& time_step)
{
	double ff_global_t{ 0 };							// The global time relative to the fast forward state

	// Pause the sensor threads
	object_X_position_sensor.SetPauseThread(true);
	object_X_acceleration_sensor.SetPauseThread(true);
	object_C_acceleration_sensor.SetPauseThread(true);

	// Set the states to start collecting data
	StartContinousCapture();

	// Update the world
	Update(0, time_step);

	while (ff_global_t < total_time)
	{
		// Advance the total time elapsed
		ff_global_t += time_step;

		// Simulate sensors firing
		object_X_position_sensor.SimulateSensorReading(ff_global_t);
		object_X_position_sensor.QueryForSensorValueChangedEvent(ff_global_t);
		object_X_acceleration_sensor.SimulateSensorReading(ff_global_t);
		object_X_acceleration_sensor.QueryForSensorValueChangedEvent(ff_global_t);
		object_C_acceleration_sensor.SimulateSensorReading(ff_global_t);
		object_C_acceleration_sensor.QueryForSensorValueChangedEvent(ff_global_t);

		// Update the world
		Update(ff_global_t, time_step);
	}

	// Set the states to finish collecting data
	StopContinuousCapture();


	// Reset to factory settings
	object_X_position_sensor.Reset();
	object_X_acceleration_sensor.Reset();
	object_C_acceleration_sensor.Reset();
	object_X->Reset();
	object_C->Reset();

	// Disable further filter updating, since we are not collecting data
	next_filter_update_t = -1;

	// Resume the sensor threads
	object_X_position_sensor.SetPauseThread(false);
	object_X_acceleration_sensor.SetPauseThread(false);
	object_C_acceleration_sensor.SetPauseThread(false);
}