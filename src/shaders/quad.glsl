#version 330 core


#ifdef VERTEX
#definesub INOUT out
#endif
#ifdef FRAGMENT
#definesub INOUT in
#endif

// output of the geometry shader, input to the fragment shader
#sub INOUT  VertexData {
	vec2 uv;
	vec3 normal;
	vec3 colour;
} vertex_#sub INOUT ;



//---------------------------------------------------------------------------------------------

#ifdef VERTEX

in vec3 a_vertex_position_modelspace;
in vec3 a_vertex_colour;
in vec2 a_vertex_UV;
in vec3 a_vertex_normal_modelspace;

uniform mat4 u_projection_matrix;
uniform mat4 u_view_matrix;
uniform mat4 u_model_matrix;

void main()
{
	gl_Position = u_projection_matrix * u_view_matrix * u_model_matrix * vec4(a_vertex_position_modelspace, 1.0);
	vertex_out.uv = a_vertex_UV;
	vertex_out.normal = a_vertex_normal_modelspace;
	vertex_out.colour = a_vertex_colour;
}


#endif





//---------------------------------------------------------------------------------------------

#ifdef FRAGMENT

//uniform sampler2D u_texture_0;

layout(location = 0) out vec4 color;

void main()
{
	//vec4 tex0 = texture(u_texture_0, vertex_in.uv);
	vec2 uvcolor = vec2(vertex_in.uv.x, vertex_in.uv.y) * 0.5 + 0.4;
	color = vec4(uvcolor.x, uvcolor.y, 0.0, 0.5);
}


#endif
