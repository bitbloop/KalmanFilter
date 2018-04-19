#pragma once

#include <list>

/**
* A class for dead reckoning. It is used for the outputs of the Kalman filter.
*
* Template parameters are:	
*	T - type of measurements.
*	HISTORY_LENGTH - How far back do we keep the history.
*
* It works by simple averaging past values for the velocity and acceleration measurements.
* It takes the last measured position as a starting point of dead reckoning.
* Combining the start position, velocity, and acceleration we get a predicted position.
*/

template <typename T, int HISTORY_LENGTH>
class DeadReckoning_PositionAcceleration
{
	static_assert(HISTORY_LENGTH > 0, "The history size must include at least one point for calculation.");
public:
	/** The constructor */
	DeadReckoning_PositionAcceleration():
		last_read_global_t(0)
	{ };
	/** The destructor */
	~DeadReckoning_PositionAcceleration() { };


	/** Applies a dead reckoning algorithm to return an current approximated value of the target object. */
	T GetPositionEstimate(const double global_t) const
	{
		if (last_read_sensor_values_history[0].size() < 1) return T(0);

		// Dead reckoning
		T average_value[SensorType::Count];
		for (int i = 0; i < static_cast<int>(SensorType::Count); ++i)
			average_value[i] = AverageHistory(last_read_sensor_values_history[i]);
	
		// Get the average state until time t-1
		//const auto s0{ average_value[POSITION_INDEX] };
		const auto a{ average_value[ACCELERATION_INDEX] };
		const auto v0{ average_value[VELOCITY_INDEX] };
		
		// Get the state at time t-1
		const auto s0{ last_read_sensor_values_history[POSITION_INDEX].front().second };
		//const auto a{ last_read_sensor_values_history[ACCELERATION_INDEX].front().second };
		//const auto v0{ last_read_sensor_values_history[VELOCITY_INDEX].front().second };

		// Compute the elapsed time since the last simulation step
		const auto elapsed_time{ global_t - last_read_global_t };

		// Compute the velocity at this instant, given a constant acceleration
		const auto v{ v0 + a * elapsed_time }; // v = v0 + a * t

		// Compute the new position estimate
		T position_estimate{ s0 + v*elapsed_time + 0.5*a*elapsed_time*elapsed_time };	// s = s0 + v*t + .5*a*t*t

		return position_estimate;
	};

	/** Returns the average past acceleration. */
	T GetAccelerationEstimate(const double global_t) const
	{
		return AverageHistory(last_read_sensor_values_history[ACCELERATION_INDEX]);
	};

	/** Adds the sensor reading to the history. */
	void SetSensorReading(const T& position_value, const T& acceleration_value, const double time)
	{
		// estimate and update the velocity
		const bool have_history{ last_read_sensor_values_history[0].size() > 0 };
		const T& s0{ (have_history) ? last_read_sensor_values_history[POSITION_INDEX].front().second : position_value};

		const auto elapsed_time{ time - last_read_global_t };
		last_read_global_t = time;									// Update the last time we read a measurement

		// Add the position, velocity, and acceleration, to the history.
		last_read_sensor_values_history[VELOCITY_INDEX].push_front(std::make_pair(time, (elapsed_time < 0.00001)?T(0):(position_value - s0) / elapsed_time));
		last_read_sensor_values_history[ACCELERATION_INDEX].push_front(std::make_pair(time, acceleration_value));
		last_read_sensor_values_history[POSITION_INDEX].push_front(std::make_pair(time, position_value));

		// Prune the history if it is too long.
		if (last_read_sensor_values_history[0].size() > HISTORY_LENGTH)
			for (int i = 0; i < static_cast<int>(SensorType::Count); ++i) last_read_sensor_values_history[i].pop_back();

		return;
	}

private:

	/** Computes the average value in the history. */
	static T AverageHistory(const std::list<std::pair<double, T>>& history)
	{
		assert(history.size() > 0);

		T total_value{ T(0) };
		T last_recorded_value{ history.front().second };

		// Compute the average value accross the recorded history values.
		for (const auto point : history)
		{
			const auto delta_value{ last_recorded_value - point.second };

			total_value += point.second;
			last_recorded_value = point.second;
		}

		// Get the average value
		total_value /= history.size();

		return total_value;
	}

private:
	// Different sensor measurement types
	enum class SensorType { position = 0, velocity, acceleration, Count };

	// Helper cast values
	static const int POSITION_INDEX{ static_cast<unsigned int>(SensorType::position) };
	static const int VELOCITY_INDEX{ static_cast<unsigned int>(SensorType::velocity) };
	static const int ACCELERATION_INDEX{ static_cast<unsigned int>(SensorType::acceleration) };

	// The history of measurements for each of the measurement types
	std::list<std::pair<double, T>> last_read_sensor_values_history[SensorType::Count]; 	// <global_t, array of values>, array index 0 is position, 1 is velocity, 2 is acceleration

	double last_read_global_t;
};