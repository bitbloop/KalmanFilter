#pragma once

#include "Graph.h"
#include "util\timer.h"

/**
*	A class which will accept data while the simmulation is running, and then bake the data into graphs and analyze it.
*/

class Analysis
{
public:
	/**
	* Initialize the analysis class.
	* analysis_x_scale - The scaling along x when creating the analysis graphs.
	*/
	Analysis():
		analysis_x_scale( 0.001f ),
		is_baked( false ),
		analysis_graph_ground_truth_position_X({ 1, 0, 0 }, { 0, 0, 0 }),
		analysis_graph_ground_truth_position_C({ 0.8, 0.8, 0.8 }, { 0, 0, 0 }),
		analysis_graph_ground_truth_position_X_in_C({ 0.5, 0.0, 0.0 }, { 0, 0, 0 }),
		analysis_graph_sensor_position({ 0, 0, 1 }, { 0, 0, 0 }),
		analysis_graph_sensor_acceleration({ 0, 0, 1 }, { 0, 0, 0 }),
		analysis_graph_filter_position({ 0, 1, 0 }, { 0, 0, 0 }),
		analysis_graph_filter_acceleration({ 0, 1, 0 }, { 0, 0, 0 }),
		analysis_graph_future_prediction_position({ 1, 1, 0 }, { 0, 0, 0.1 }),
		analysis_graph_ground_truth_position_error({ 1, 0, 0 }, { 0, 0, 0 }),
		analysis_graph_sensor_position_error({ 0, 0, 1 }, { 0, 0, 0 }),
		analysis_graph_filter_position_error({ 0, 1, 0 }, { 0, 0, 0 }),
		analysis_graph_camera_object_distance({ 1, 1, 1 }, { 0, 0, 0 })
	{
	};

	/** Add the supplied measurements to appropriate graphs. */
	void AddPoint(const glm::dvec3& ground_truth_position_X,
		const glm::dvec3& ground_truth_position_C,
		const glm::dvec3& sensor_position,
		const glm::dvec3& sensor_acceleration,
		const glm::dvec3& filter_position,
		const glm::dvec3& filter_acceleration,
		const double prediction_error)
	{
		analysis_graph_ground_truth_position_X.AddDataPoint(ground_truth_position_X);
		analysis_graph_ground_truth_position_C.AddDataPoint(ground_truth_position_C);
		analysis_graph_ground_truth_position_X_in_C.AddDataPoint(ground_truth_position_X - ground_truth_position_C);
		analysis_graph_sensor_position.AddDataPoint(sensor_position);
		analysis_graph_sensor_acceleration.AddDataPoint(sensor_acceleration);
		analysis_graph_filter_position.AddDataPoint(filter_position);
		analysis_graph_filter_acceleration.AddDataPoint(filter_acceleration);
		analysis_graph_future_prediction_position.AddDataPoint(analysis_graph_future_prediction_position.GetPointNum() * analysis_x_scale, static_cast<float>(prediction_error));
	}

	/** Bake the graphs of the supplied measurements. */
	void BakeGraphs(const unsigned int shader_id)
	{
		if (is_baked) return;

		{
			math::time::Timer fast_forward_timer;
			CreateDetailAnalysisGraphs();
			double fast_forward_end_time{ fast_forward_timer.elapsed() };
			std::cout << std::endl << "Analysis took " << fast_forward_end_time << "s." << std::endl;
		}

		{
			math::time::Timer fast_forward_timer;
			BakeDetailAnalysisGraphs(shader_id);
			double fast_forward_end_time{ fast_forward_timer.elapsed() };
			std::cout << "Analysis graph creation took " << fast_forward_end_time << "s." << std::endl;
		}
		

		analysis_graph_sensor_position.Bake(shader_id, "analysis_graph_sensor_position.txt", false, analysis_graph_ground_truth_position_C.GetDataPointsXYZ());		// The sensor predicted position, shifted from camera to world space.
		analysis_graph_filter_position.Bake(shader_id, "analysis_graph_filter_position.txt", false, analysis_graph_ground_truth_position_C.GetDataPointsXYZ());		// The filter predicted position, shifted from camera to world space.

		analysis_graph_ground_truth_position_X.Bake(shader_id, "analysis_graph_ground_truth_X.txt");
		analysis_graph_ground_truth_position_C.Bake(shader_id, "analysis_graph_ground_truth_C.txt");
		analysis_graph_ground_truth_position_X_in_C.Bake(shader_id, "analysis_graph_ground_truth_C.txt");
		
		analysis_graph_sensor_acceleration.Bake(shader_id, "analysis_graph_sensor_acceleration.txt");		
		analysis_graph_filter_acceleration.Bake(shader_id, "analysis_graph_filter_acceleration.txt");

		is_baked = true;
	}

	/** Render the baked graphs. */
	void Render(const unsigned int shader_id) const
	{
		if (!is_baked) return;
		
		analysis_graph_ground_truth_position_X.Render(shader_id, false);
		analysis_graph_ground_truth_position_C.Render(shader_id, false);
		//analysis_graph_future_prediction_position.Render(shader_id, true);
		analysis_graph_sensor_position.Render(shader_id, true);
		analysis_graph_filter_position.Render(shader_id, true);
		

		analysis_graph_ground_truth_position_error.Render(shader_id, true);
		analysis_graph_sensor_position_error.Render(shader_id, true);
		analysis_graph_filter_position_error.Render(shader_id, true);
		analysis_graph_camera_object_distance.Render(shader_id, false);
	}

	/** Did we bake the graph? */
	inline bool IsBaked() const { return is_baked; };

private:
	/** A function which will accept two float arrays, of the format xyz, and compute the distance between them. */
	double DistVectorComponentsXYZ(const float* x1, const float* x2)
	{
		const auto x{ x1[0] - x2[0] };
		const auto y{ x1[1] - x2[1] };
		const auto z{ x1[2] - x2[2] };
		return std::sqrt(x*x + y*y + z*z);
	}

	/** Create additional analysis graphs from the supplied data. In this same function, it also outputs some properties of the created graphs. */
	void CreateDetailAnalysisGraphs()
	{
		const auto analysis_graph_ground_truth_data_xyz{ analysis_graph_ground_truth_position_X_in_C.GetDataPointsXYZ() };
		const auto analysis_graph_sensor_position_data_xyz{ analysis_graph_sensor_position.GetDataPointsXYZ() };
		const auto analysis_graph_filter_position_data_xyz{ analysis_graph_filter_position.GetDataPointsXYZ() };
		const auto analysis_graph_future_prediction_position_xyz{ analysis_graph_future_prediction_position.GetDataPointsXYZ() };

		double sum_ground_truth_error{ 0 };
		double sum_sensor_position_error{ 0 };
		double sum_filter_position_error{ 0 };
		double sum_future_position_error{ 0 };
		double maximum_distance{ 0 };

		for (int i = 0; i < analysis_graph_ground_truth_data_xyz.size() / 3; ++i)
		{
			const auto ground_truth_error{ DistVectorComponentsXYZ(&analysis_graph_ground_truth_data_xyz[i * 3], &analysis_graph_ground_truth_data_xyz[i * 3]) };
			const auto sensor_position_error{ DistVectorComponentsXYZ(&analysis_graph_ground_truth_data_xyz[i*3], &analysis_graph_sensor_position_data_xyz[i*3]) };
			const auto filter_position_error{ DistVectorComponentsXYZ(&analysis_graph_ground_truth_data_xyz[i*3], &analysis_graph_filter_position_data_xyz[i*3]) };
			const auto camera_object_distance{ DistVectorComponentsXYZ(&analysis_graph_ground_truth_position_X.GetDataPointsXYZ()[i * 3], &analysis_graph_ground_truth_position_C.GetDataPointsXYZ()[i * 3]) };

			analysis_graph_ground_truth_position_error.AddDataPoint(i * analysis_x_scale, static_cast<float>(ground_truth_error));
			analysis_graph_sensor_position_error.AddDataPoint(i * analysis_x_scale, static_cast<float>(sensor_position_error));
			analysis_graph_filter_position_error.AddDataPoint(i * analysis_x_scale, static_cast<float>(filter_position_error));
			analysis_graph_camera_object_distance.AddDataPoint(i * analysis_x_scale, static_cast<float>(camera_object_distance));

			sum_ground_truth_error += ground_truth_error;
			sum_filter_position_error += filter_position_error;
			sum_sensor_position_error += sensor_position_error;
			sum_future_position_error += analysis_graph_future_prediction_position_xyz[i * 3 + 1];

			maximum_distance = std::max(maximum_distance, camera_object_distance);
		}

		std::cout << "Analysis:" << std::endl;
		std::cout << "Sensor position average error:" << sum_sensor_position_error / (analysis_graph_ground_truth_data_xyz.size() / 3) << std::endl;
		std::cout << "Filter position average error:" << sum_filter_position_error / (analysis_graph_ground_truth_data_xyz.size() / 3) << std::endl;
		//std::cout << "Prediction error:" << sum_future_position_error / (analysis_graph_future_prediction_position_xyz.size() / 3) << std::endl;
		std::cout << std::endl;
		std::cout << "Maximum distance between C and X:" << maximum_distance << std::endl;
	}

	/** Bake the generated anaysis graphs. */
	void BakeDetailAnalysisGraphs(const unsigned int shader_id)
	{
		analysis_graph_ground_truth_position_error.Bake(shader_id, "analysis_graph_ground_truth_position_error.txt");
		analysis_graph_sensor_position_error.Bake(shader_id, "analysis_graph_sensor_position_error.txt");
		analysis_graph_filter_position_error.Bake(shader_id, "analysis_graph_filter_position_error.txt");
		analysis_graph_camera_object_distance.Bake(shader_id, "analysis_graph_camera_object_distance.txt");
		analysis_graph_future_prediction_position.Bake(shader_id, "analysis_graph_future_prediction_position.txt");

		const auto graph_position{ glm::dvec3({ 20., 0., 0. }) };
		analysis_graph_ground_truth_position_error.SetGraphPosition(graph_position);
		analysis_graph_sensor_position_error.SetGraphPosition(graph_position);
		analysis_graph_filter_position_error.SetGraphPosition(graph_position);
		analysis_graph_camera_object_distance.SetGraphPosition(graph_position);
		analysis_graph_future_prediction_position.SetGraphPosition(graph_position);
	}

private:
	const float analysis_x_scale;		// The amount of x axis scaling when generating the analysis graphs. Setting this control the width of the graph.
	bool is_baked;						// Did we bake the graphs, ie. create appropriate OpenGL objects and transfter the data to the GPU.

	// These will be continously updated with new measurements
	Graph analysis_graph_ground_truth_position_X;
	Graph analysis_graph_ground_truth_position_C;
	Graph analysis_graph_ground_truth_position_X_in_C;
	Graph analysis_graph_sensor_position;
	Graph analysis_graph_sensor_acceleration;
	Graph analysis_graph_filter_position;
	Graph analysis_graph_filter_acceleration;
	Graph analysis_graph_future_prediction_position;

	// Once all measurements are in, these are the generated analysis graphs
	Graph analysis_graph_ground_truth_position_error;
	Graph analysis_graph_sensor_position_error;
	Graph analysis_graph_filter_position_error;
	Graph analysis_graph_camera_object_distance;
};