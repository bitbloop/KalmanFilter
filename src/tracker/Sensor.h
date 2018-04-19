#pragma once

#include <atomic> 
#include <mutex>
#include <thread>
#include <list>
#include "util\random.h"

/**
* A class for sensors 
* Template parameters are:
*	T - type of measurements.
*	HISTORY_LENGTH - How far back do we keep the history.
*
* Class responsibilities:
* - Creates and manages the sensor thread.
* - Can simulate sensor reading bypassing threads. (used for fast forwarding the simulation)
* - The sensor value can always be queried. It tries to give the best estimate via dead reckoning approximation. (used when reading values not in the same interval as updating sensor values)
*
* When creating the threads: we supply the thread frequency of ground truth sampling.
*
* General flow:
* - The master simulator feeds the ground truth values at every simulation iteration.
* - The sensor thread will periodically sample the ground truth, apply noise to it, save that value, and notify the simulation that a new sensor read value is ready.
* - We can ask for the sensor value in between sensor updates, a dead reckoning algorithm and return that value.
*/


template <typename T, int HISTORY_LENGTH>
class Sensor
{
	static_assert(HISTORY_LENGTH > 1, "The history size must include at least two points for the velocity to be calculated.");
public:

	/** The function returns no noise by default. */
	template<typename T> T WhiteGaussianNoise(const double mean = 0, const double variance = 1.) { return T{ 0 }; };
	/** Template specialization. Returns a scaled normal distribution for each of the dvec3 components. */
	template<> glm::dvec3 WhiteGaussianNoise<glm::dvec3>(const double mean, const double variance) { return glm::dvec3({ util::random::get_normal_gaussian(mean, variance), util::random::get_normal_gaussian(mean, variance), util::random::get_normal_gaussian(mean, variance) }); };

	/** Create a sensor with supplied noise mean and variance. */
	Sensor(const double noise_mean = 0., const double noise_variance = 1.) :
		noise_variance(noise_variance),
		noise_mean(noise_mean),
		sensor_data_ready(false), 
		stop_thread(false),
		pause_thread(false)
	{ 
		last_read_sensor_values.push_front(std::make_pair(0, T(0)));				// Push one history value to avoid a crash when we sample a sensor with no read values.
	};
	
	/** Destroy the sensor. */
	~Sensor() { stop_thread = true; sensor_thread.join(); };

	/** Applies a linear dead reckoning alforithm to return an current approximated value of the target object. */
	T GetEstimate(const double global_t) 
	{ 
		assert(last_read_sensor_values.size() > 0);					// Make sure that we have at least one recorded history step.
		
		// Dead reckoning
		double last_recorded_time{ last_read_sensor_values.front().first }; // This will hold the last touched recorded time, used to compute the relative time change between two recorded history steps.
		T last_recorded_value{ last_read_sensor_values.front().second };	// This will hold the last touched recorded value, used to compute the relative value change between two recorded history steps.
		T value{ 0 };														// This will hold the sum of all values in the history. Afterwards the values will be averaged.

		// Compute the average value accross the recorded history values.
		double total_time{ 0 };
		for (const auto point : last_read_sensor_values)
		{
			if (point.first > global_t) continue;	// don't consider points which will come to us in the future

			total_time += last_recorded_time - point.first;
			value += last_recorded_value - point.second;

			last_recorded_time = point.first;
			last_recorded_value = point.second;
		}

		// Adjust for the numerical computation error.
		if (total_time < 0.0001) return last_read_sensor_values.front().second;

		// get the velocity
		value /= total_time;

		// The time elapsed since the last reading
		const auto elapsed_time{ global_t - last_read_global_t };	

		// Predict
		return last_read_sensor_values.front().second + value*elapsed_time; 
	};

	/** This will inspect if the sensor value got updated. If so, that is the new truth. */
	void QueryForSensorValueChangedEvent(const double global_t)
	{
		T tmp;
		{
			std::lock_guard<std::mutex> lock(sensor_data_mutex);
			if (!sensor_data_ready) return;
			tmp = sensor_reading;
			sensor_data_ready = false;
		}
		if (last_read_sensor_values.size() > HISTORY_LENGTH) {
			last_read_sensor_values.pop_back();
		}
		last_read_global_t = global_t;
		last_read_sensor_values.push_front(std::pair<double, T>(global_t, tmp));
	};
	
	/** Set the ground truth the sensor will sample when it is time. */
	void SetGroundTruth(const T& ground_truth)
	{
		std::lock_guard<std::mutex> lock(sensor_data_mutex);
		most_recent_ground_truth = ground_truth;
	}

	/** Returns the raw, non interpolated, last sensor reading data. */
	T GetSensorReading()
	{
		std::lock_guard<std::mutex> lock(sensor_data_mutex);
		return sensor_reading;
	}

	/** Simulate a sensor reading of the ground truth, while obeying the sensor sampling frequency. */
	void SimulateSensorReading(const double& global_t)
	{
		if (last_read_global_t + frequency > global_t) return;
		std::lock_guard<std::mutex> lock(sensor_data_mutex);
		sensor_reading = most_recent_ground_truth + WhiteGaussianNoise<T>(noise_mean, noise_variance);
		sensor_ground_truth = most_recent_ground_truth;
		sensor_data_ready = true;
	}

	/** Change the frequency of the thread execution. Will only be effective at the subsequent thread loop iteration. */
	inline void SetFrequency(const double& new_frequency) { frequency = new_frequency; }

	/** Change the frequency of the thread execution. Will only be effective at the subsequent thread loop iteration. */
	inline void SetPauseThread(const bool new_pause_thread) { pause_thread = new_pause_thread; }	

	/** Start the sensor ground truth reading thread. */
	void LaunchSensorThread(const double frequency_in = 1 / 300.)
	{
		frequency = frequency_in;

		// Thread execution function
		const auto worker_fn = [this]() {
			while (!stop_thread)
			{
				const long frequency_ms_i{ static_cast<long>(frequency * 1000) };

				// Simulate thread waiting
				if (pause_thread)
				{
					std::this_thread::sleep_for(std::chrono::milliseconds(frequency_ms_i));
					return;
				}
				
				{
					std::lock_guard<std::mutex> lock(sensor_data_mutex);
					sensor_reading = most_recent_ground_truth + WhiteGaussianNoise<T>(noise_mean, noise_variance);
					sensor_ground_truth = most_recent_ground_truth;
					sensor_data_ready = true;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(frequency_ms_i));
			}
		};

		// Create the thread
		sensor_thread = std::thread(worker_fn);
	}

	/** Reset all sensor values to the beginning state. */
	void Reset(const double global_t = 0.)
	{
		last_read_sensor_values.clear();
		last_read_global_t = global_t;
		most_recent_ground_truth = T(0);
		sensor_reading = T(0);
		sensor_ground_truth = T(0);
	}


private:
	const double noise_mean;
	const double noise_variance;

	std::thread sensor_thread;					// The sensor thread.
	std::atomic<bool> stop_thread;				// A thread safe signal for if the thread to stop executing.
	std::atomic<bool> pause_thread;				// A thread safe signal for if the thread to be paused.

	std::atomic<bool> sensor_data_ready;		// The thread safe signal new data is ready.
	std::mutex sensor_data_mutex;				// Mutex to update the data in a thread safe way.

	std::atomic<double> frequency;				// The frequency at which the sensor will provide new data

	T most_recent_ground_truth;					// The most recent ground truth set by the simulation.
	T sensor_reading;							// The last read ground truth value with noise
	T sensor_ground_truth;						// The last read ground truth value
	
	std::list<std::pair<double, T>> last_read_sensor_values;	// History: global_t, reading array of values
	double last_read_global_t;					// The time last time we read the sensor new value.		

};