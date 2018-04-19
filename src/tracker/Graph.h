#pragma once

/**
*	A class for storing measurements, and creating OpenGL visualization based on them.
*/

#include "renderer\renderer.h"
#include <random>
#include <vector>
#include <fstream>


class Graph
{
public:
	/**
	* Initialize the Graph class.
	* color - Graph color, as RGB.
	* position - Graph global position, as RGB;
	*/
	Graph(
		const glm::vec3 color = { rand() / RAND_MAX*0.4f + 0.6f, rand() / RAND_MAX*0.4f + 0.6f, rand() / RAND_MAX*0.4f + 0.6f }, 
		const glm::vec3 position = { 0,0,0 }) :
		single_rgb{ color.x, color.y, color.z },
		is_baked{ false },
		position{ position }
	{
	}

	/** The destructor. */
	~Graph()
	{
		// Only kill the GL object if it was previously baked.
		if (is_baked) ::renderer::pointcloud::kill(pointcloud_id);
	}

	/** Reset the values to initial settings. */
	void Reset()
	{
		if (is_baked) ::renderer::pointcloud::kill(pointcloud_id);
		pointcloud_id = -1;
		is_baked = false;
		data_points_xyz.clear();
		position = { 0, 0, 0 };	
	}

	/** Saves the measurements to a file. */
	void SaveDataToFile(const char* out_filename = nullptr)
	{
		return;

		if (!out_filename) return;

		std::ofstream output_file(out_filename);
		if (!output_file) return;

		for (int i = 0; i < data_points_xyz.size() / 3; ++i)
		  output_file << data_points_xyz[i * 3] << " " << data_points_xyz[i * 3 + 1] << " " << data_points_xyz[i * 3 + 2] << std::endl;

		output_file.close();
	}

	/** Bakes the measurements into an OpenGL object. */
	void Bake(const unsigned int shader_id, const char* out_filename = nullptr, const bool keep_data_in_cpu_memory = false, const std::vector<float>& offset_xyz = std::vector<float>())
	{
		if (is_baked) return;
		const unsigned int data_num{ static_cast<unsigned int>(data_points_xyz.size()) };
		const unsigned int vert_num{ data_num / 3 };
		std::vector<float> data_rgb(data_num);
		auto raw_data_rgb = data_rgb.data();
		for (unsigned int vi = 0; vi < vert_num; ++vi)
		{
			raw_data_rgb[vi * 3 + 0] = static_cast<float>(single_rgb[0]);
			raw_data_rgb[vi * 3 + 1] = static_cast<float>(single_rgb[1]);
			raw_data_rgb[vi * 3 + 2] = static_cast<float>(single_rgb[2]);

			// we supplied a bake offset
			if (offset_xyz.size() > 0)
			{
				data_points_xyz[vi * 3 + 0] += offset_xyz[vi * 3 + 0];
				data_points_xyz[vi * 3 + 1] += offset_xyz[vi * 3 + 1];
				data_points_xyz[vi * 3 + 2] += offset_xyz[vi * 3 + 2];
			}
		}


		pointcloud_id = ::renderer::pointcloud::create(shader_id, vert_num, data_points_xyz.data(), raw_data_rgb);

		if (out_filename) SaveDataToFile(out_filename);

		if (!keep_data_in_cpu_memory) data_points_xyz.clear(); // we no longer need this information in the CPU memory
		else // undo the data offseting
		{
			if (offset_xyz.size() > 0)
			{
				for (unsigned int vi = 0; vi < vert_num; ++vi)
				{
					data_points_xyz[vi * 3 + 0] -= offset_xyz[vi * 3 + 0];
					data_points_xyz[vi * 3 + 1] -= offset_xyz[vi * 3 + 1];
					data_points_xyz[vi * 3 + 2] -= offset_xyz[vi * 3 + 2];
				}
			}
		}

		// We baked an OGL object.
		is_baked = true;
	};

	/** Add a new measurement point, given a 3d vector. */
	void AddDataPoint(const glm::dvec3 point)
	{
		if (is_baked) return;
		data_points_xyz.push_back(static_cast<float>(point.x));
		data_points_xyz.push_back(static_cast<float>(point.y));
		data_points_xyz.push_back(static_cast<float>(point.z));
	};

	/** Add a new measurement point, given a 3d vector, with z = 0 by default. */
	void AddDataPoint(const float x, const float y, const float z = 0)
	{
		if (is_baked) return;
		data_points_xyz.push_back(x);
		data_points_xyz.push_back(y);
		data_points_xyz.push_back(z);
	};

	/** Add a new measurement point, given a 1d vector, with x = current_measurement_num and z = 0 */
	void AddDataPoint(const float y)
	{
		if (is_baked) return;
		data_points_xyz.push_back(static_cast<float>(data_points_xyz.size()));
		data_points_xyz.push_back(y);
		data_points_xyz.push_back(0.f);
	};

	/** Add sets the world position of the graph. */
	void SetGraphPosition(const glm::dvec3& graph_position)
	{
		position = graph_position;
	}

	/** Render the OpenGL graph. */
	void Render(const unsigned int shader_id, const bool render_as_lines = true) const
	{
		if (!is_baked) return;

		const auto model_matrix{ glm::translate(glm::mat4(1),{ position.x, position.y, position.z }) };
		SetModelMatrix(model_matrix, shader_id);

		::renderer::pointcloud::render(pointcloud_id, render_as_lines);
	}

	/** Did we bake OpenGL objects. */
	inline bool GetBaked() const { return is_baked; };
	/** Returns the number of measurements. */
	inline unsigned int GetPointNum() const { return static_cast<unsigned int>(data_points_xyz.size() / 3); };
	/** Returns an array of measurements, represented as an array of floats of the form xyzxyz... */
	const std::vector<float> GetDataPointsXYZ() const { return data_points_xyz; };

private:
	/** Send the supplied model matrix to the shader. */
	void SetModelMatrix(const glm::mat4x4& model_transform, const unsigned int shader_id) const
	{
		// get shader uniform locations
		const auto u_model_matrix{ glGetUniformLocation(shader_id, "u_model_matrix") };
		// update the shader uniforms
		if (u_model_matrix != -1) glUniformMatrix4fv(u_model_matrix, 1, GL_FALSE, glm::value_ptr(model_transform));
	}
private:
	bool is_baked;							// Did we bake the graphs, ie. create appropriate OpenGL objects and transfter the data to the GPU.
	const float single_rgb[3];				// An RGB array storing the colour of the graph.
	std::vector<float> data_points_xyz;		// The graph data, styored as an array of coordinates, ie. xyzxyz...
	unsigned int pointcloud_id;				// The ID of the created OpenGL object.

	glm::vec3 position;				
};