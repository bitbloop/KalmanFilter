#include "renderer.h"


#define GLEW_STATIC 
#include <GL/glew.h>
#include <GLFW/glfw3.h>


#include <glm\gtc\type_ptr.hpp>
#include "window.h"
#include "space.h"
#include "texture.h"
#include "shader.h"


namespace
{
	const GLenum COMPONENT_TYPE = GL_FLOAT;
	const auto SIZE_OF_COMPONENT = sizeof(float);
	const int POSITION_OFFSET = 0;
	const int POSITION_SIZE = 3;
	const int UV_OFFSET = POSITION_SIZE;
	const int UV_SIZE = 2;
	const int VERTEX_COMPONENTS = POSITION_SIZE + UV_SIZE;

	const int QUAD_VERT_NUM = 4;
	const int QUAD_INDEX_NUM = 4;
	const int CUBE_VERT_NUM = 14;
	const int CUBE_INDEX_NUM = 14;

	struct GL3Quad
	{
		unsigned int a_vertex_position;
		unsigned int a_vertex_UV;

		unsigned int vbo_id;
		unsigned int indexbo_id;

		bool is_a_cube;
	};

	void get_quad_shader_attributes(const unsigned int& program_id, unsigned int& a_vertex_position, unsigned int& a_vertex_UV)
	{
		a_vertex_position = glGetAttribLocation(program_id, "a_vertex_position_modelspace");
		a_vertex_UV = glGetAttribLocation(program_id, "a_vertex_UV");
	}

	// will make [-1, 1] quad
	void make_quad_vbo(unsigned int& vbo_id, unsigned int& indexbo_id, const bool make_cube)
	{
		static const float packed_vert_cube[VERTEX_COMPONENTS * CUBE_VERT_NUM] = {
			-1.f, 1.f, 1.f, 0, 1,	// Front-top-left
			1.f, 1.f, 1.f, 1, 1,	// Front-top-right
			-1.f, -1.f, 1.f, 0, 0,	// Front-bottom-left
			1.f, -1.f, 1.f, 1, 0,	// Front-bottom-right
			1.f, -1.f, -1.f, 1, 0,	// Back-bottom-right
			1.f, 1.f, 1.f, 1, 1,	// Front-top-right
			1.f, 1.f, -1.f, 1, 1,	// Back-top-right
			-1.f, 1.f, 1.f, 0, 1,	// Front-top-left
			-1.f, 1.f, -1.f, 0, 1,	// Back-top-left
			-1.f, -1.f, 1.f, 0, 0,	// Front-bottom-left
			-1.f, -1.f, -1.f, 0, 0,	// Back-bottom-left
			1.f, -1.f, -1.f, 1, 0,	// Back-bottom-right
			-1.f, 1.f, -1.f, 0, 1,	// Back-top-left
			1.f, 1.f, -1.f, 1, 1	// Back-top-right
		};
		static const float packed_vert[VERTEX_COMPONENTS * QUAD_VERT_NUM]{
			-1, -1, 0, 0, 0,	// Bottom-left
			-1, 1, 0, 0, 1,		// Top-left
			1, -1, 0, 1, 0,		// Bottom-right
			1, 1, 0, 1, 1 };	// Top-right

		glGenBuffers(1, &vbo_id);
		glBindBuffer(GL_ARRAY_BUFFER, vbo_id);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float)* VERTEX_COMPONENTS * ((make_cube) ? CUBE_VERT_NUM:QUAD_VERT_NUM), ((make_cube) ? packed_vert_cube:packed_vert), GL_STATIC_DRAW);

		const unsigned int packed_ind[QUAD_INDEX_NUM]{ 0, 1, 2, 3 };
		const unsigned int packed_cube_ind[CUBE_INDEX_NUM]{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13 };
		glGenBuffers(1, &indexbo_id);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexbo_id);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * ((make_cube) ? CUBE_INDEX_NUM:QUAD_INDEX_NUM), ((make_cube) ? packed_cube_ind:packed_ind), GL_STATIC_DRAW);
	}



	unsigned int _total_quads_created = 0; // quads created since the beginning of the simulation. will be used as tjhe key into the pointer map
	std::unordered_map<unsigned int, GL3Quad*> _quad_map;

	void create_quad_instance(unsigned int& index_out, GL3Quad*& quad_ptr_out)
	{
		auto quad_obj = new GL3Quad();
		_quad_map[++_total_quads_created] = quad_obj;
		index_out = _total_quads_created;
		quad_ptr_out = quad_obj;
	}
};

unsigned int renderer::quad::create(const unsigned int program_id, const bool make_cube)
{
	GL3Quad* quad_ptr;
	unsigned int quad_id;
	create_quad_instance(quad_id, quad_ptr);
	get_quad_shader_attributes(program_id, quad_ptr->a_vertex_position, quad_ptr->a_vertex_UV);
	make_quad_vbo(quad_ptr->vbo_id, quad_ptr->indexbo_id, make_cube);

	quad_ptr->is_a_cube = make_cube;

	return quad_id;
}

void renderer::quad::kill(const unsigned int quad_id)
{
	assert(_quad_map.find(quad_id) != _quad_map.end());
	auto quad_ptr = _quad_map[quad_id];
	// TODO: Clean OpenGL resources.
	delete quad_ptr;
}

void renderer::quad::render(const unsigned int quad_id)
{
	assert(_quad_map.find(quad_id) != _quad_map.end());
	const auto quad = _quad_map[quad_id];	

	glBindBuffer(GL_ARRAY_BUFFER, quad->vbo_id);
	// 1rst attribute buffer : vertices
	glEnableVertexAttribArray(quad->a_vertex_position);
	glVertexAttribPointer(
		quad->a_vertex_position,                  // must match the layout in the shader.
		POSITION_SIZE,        // size
		COMPONENT_TYPE,           // type
		GL_FALSE,           // normalized?
		SIZE_OF_COMPONENT * VERTEX_COMPONENTS,    // stride
		(void*)(POSITION_OFFSET * SIZE_OF_COMPONENT)            // array buffer offset
	);

	if (quad->a_vertex_UV != -1)
	{
		glEnableVertexAttribArray(quad->a_vertex_UV);
		glVertexAttribPointer(
			quad->a_vertex_UV,                  // must match the layout in the shader.
			UV_SIZE,                  // size
			COMPONENT_TYPE,           // type
			GL_FALSE,           // normalized?
			SIZE_OF_COMPONENT * VERTEX_COMPONENTS,   // stride
			(void*)(UV_OFFSET * SIZE_OF_COMPONENT)           // array buffer offset
		);
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, quad->indexbo_id);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, ((quad->is_a_cube) ? CUBE_INDEX_NUM:QUAD_INDEX_NUM)); // Starting from vertex 0
	//glDrawArrays(GL_LINE_STRIP, 0, ((quad->is_a_cube) ? CUBE_INDEX_NUM : QUAD_INDEX_NUM)); // Starting from vertex 0
	
																							 
	//glDrawElements(GL_TRIANGLE_STRIP, QUAD_INDEX_NUM, GL_UNSIGNED_INT, 0);

	glDisableVertexAttribArray(quad->a_vertex_position);
	glDisableVertexAttribArray(quad->a_vertex_UV);
}
