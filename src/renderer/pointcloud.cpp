#include "renderer.h"

#define GLEW_STATIC 
#include <GL/glew.h>
#include <GLFW/glfw3.h>


#include <glm\gtc\type_ptr.hpp>
#include "window.h"
#include "space.h"
#include "shader.h"

namespace
{
	const GLenum COMPONENT_TYPE = GL_FLOAT;
	const auto SIZE_OF_COMPONENT = sizeof(float);
	const int POSITION_OFFSET = 0;
	const int POSITION_SIZE = 3;
	const int VERTEX_COMPONENTS = POSITION_SIZE;

	struct GL3Pointcloud
	{
		unsigned int a_vertex_position; // xyz
		unsigned int a_vertex_colour; // rgb

		unsigned int vbo_id;
		unsigned int indexbo_id;

		unsigned int point_num;
		unsigned int vertex_component_num;

		bool has_rgb;
	};

	void get_pointcloud_shader_attributes(const unsigned int& program_id, unsigned int& a_vertex_position, unsigned int& a_vertex_colour)
	{
		a_vertex_position = glGetAttribLocation(program_id, "a_vertex_position_modelspace");
		a_vertex_colour = glGetAttribLocation(program_id, "a_vertex_colour");
	}

	// will make a pointcloud
	void make_pointcloud_vbo(const unsigned int data_num, const float* data_xyz, const float* data_rgb, unsigned int& vbo_id, unsigned int& indexbo_id)
	{
		const int vertex_component_num{ ((data_rgb)?6:3) };

		// todo this will create memory fragmentation
		std::vector<float> packed_vert(vertex_component_num * data_num);
		for (unsigned int i = 0; i < data_num; ++i)
		{
			packed_vert[vertex_component_num * i + 0] = data_xyz[i * 3 + 0];
			packed_vert[vertex_component_num * i + 1] = data_xyz[i * 3 + 1];
			packed_vert[vertex_component_num * i + 2] = data_xyz[i * 3 + 2];

			if (data_rgb != nullptr) {
				packed_vert[vertex_component_num * i + 3] = data_rgb[i * 3 + 0];
				packed_vert[vertex_component_num * i + 4] = data_rgb[i * 3 + 1];
				packed_vert[vertex_component_num * i + 5] = data_rgb[i * 3 + 2];
			}
		}

		glGenBuffers(1, &vbo_id);
		glBindBuffer(GL_ARRAY_BUFFER, vbo_id);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float)* vertex_component_num * data_num, packed_vert.data(), GL_STATIC_DRAW);

		unsigned int* indices = new unsigned int[data_num];
		for (unsigned int i = 0; i < data_num; ++i) indices[i] = i;

		glGenBuffers(1, &indexbo_id);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexbo_id);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * data_num, indices, GL_STATIC_DRAW);

		delete[] indices;
	}



	unsigned int _total_pointclouds_created = 0; // pointclouds created since the beginning of the simulation. will be used as tjhe key into the pointer map
	std::unordered_map<unsigned int, GL3Pointcloud*> _pointcloud_map;

	void create_pointcloud_instance(unsigned int& index_out, GL3Pointcloud*& pointcloud_ptr_out)
	{
		auto pointcloud_obj = new GL3Pointcloud();
		_pointcloud_map[++_total_pointclouds_created] = pointcloud_obj;
		index_out = _total_pointclouds_created;
		pointcloud_ptr_out = pointcloud_obj;
	}
};

unsigned int renderer::pointcloud::create(const unsigned int program_id, const unsigned int point_num, const float* data_xyz, const float* data_rgb)
{
	GL3Pointcloud* pointcloud_ptr;
	unsigned int pointcloud_id;
	create_pointcloud_instance(pointcloud_id, pointcloud_ptr);
	get_pointcloud_shader_attributes(program_id, pointcloud_ptr->a_vertex_position, pointcloud_ptr->a_vertex_colour);
	make_pointcloud_vbo(point_num, data_xyz, data_rgb, pointcloud_ptr->vbo_id, pointcloud_ptr->indexbo_id);
	pointcloud_ptr->point_num = point_num;
	pointcloud_ptr->has_rgb = (data_xyz != nullptr);
	pointcloud_ptr->vertex_component_num = ((data_rgb) ? 6 : 3);
	return pointcloud_id;
}

void renderer::pointcloud::kill(const unsigned int pointcloud_id)
{
	assert(_pointcloud_map.find(pointcloud_id) != _pointcloud_map.end());
	auto pointcloud_ptr = _pointcloud_map[pointcloud_id];
	// TODO: Clean OpenGL resources.
	delete pointcloud_ptr;
}

void renderer::pointcloud::render(const unsigned int pointcloud_id, const bool render_as_line_strip)
{
	assert(_pointcloud_map.find(pointcloud_id) != _pointcloud_map.end());
	const auto pointcloud = _pointcloud_map[pointcloud_id];	

	glBindBuffer(GL_ARRAY_BUFFER, pointcloud->vbo_id);
	// xyz
	glEnableVertexAttribArray(pointcloud->a_vertex_position);
	glVertexAttribPointer(
		pointcloud->a_vertex_position,                  // must match the layout in the shader.
		POSITION_SIZE,        // size
		COMPONENT_TYPE,           // type
		GL_FALSE,           // normalized?
		SIZE_OF_COMPONENT * pointcloud->vertex_component_num,    // stride
		0            // array buffer offset
	);

	// rgb
	if (pointcloud->has_rgb) {
		glEnableVertexAttribArray(pointcloud->a_vertex_colour);
		glVertexAttribPointer(
			pointcloud->a_vertex_colour,                  // must match the layout in the shader.
			POSITION_SIZE,        // size
			COMPONENT_TYPE,           // type
			GL_FALSE,           // normalized?
			SIZE_OF_COMPONENT * pointcloud->vertex_component_num,    // stride
			(void*)(3 * SIZE_OF_COMPONENT)            // array buffer offset
		);
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, pointcloud->indexbo_id);

	//glDrawArrays(GL_POINTS, 0, pointcloud->point_num); // Starting from vertex 0
	glDrawArrays((render_as_line_strip)?GL_LINE_STRIP:GL_POINTS, 0, pointcloud->point_num); // Starting from vertex 0
	//glDrawElements(GL_TRIANGLE_STRIP, QUAD_INDEX_NUM, GL_UNSIGNED_INT, 0);

	glDisableVertexAttribArray(pointcloud->a_vertex_position);
}
