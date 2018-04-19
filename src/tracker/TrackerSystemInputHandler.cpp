#include "TrackerSystemInputHandler.h"

#include "renderer\renderer.h"

#include <iostream>

float TrackerSystemInputHandler::camera_pan_speed{ 0.01f };
float TrackerSystemInputHandler::camera_zoom_speed{ 0.1f };
TrackerSystemInputHandler::CameraTracking TrackerSystemInputHandler::camera_tracking_data;

bool TrackerSystemInputHandler::move_state_active{ false };
bool TrackerSystemInputHandler::initialized{ false };

double TrackerSystemInputHandler::current_mouse_position[2]{ 0,0 };
double TrackerSystemInputHandler::previous_mouse_position[2]{ 0,0 };


void TrackerSystemInputHandler::mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT) move_state_active = (action == GLFW_PRESS);
}

void TrackerSystemInputHandler::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
	
	const glm::vec3 delta{ 0, 0, -yoffset };
	if (delta.z < 0 && camera_tracking_data.position.z < 0.1) {
		camera_tracking_data.position.z = 0.1;
		return;
	}
	camera_tracking_data.position += delta * camera_zoom_speed * camera_tracking_data.position.z;
}

void TrackerSystemInputHandler::cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
	previous_mouse_position[0] = current_mouse_position[0];
	previous_mouse_position[1] = current_mouse_position[1];

	current_mouse_position[0] = xpos;
	current_mouse_position[1] = ypos;

	if (move_state_active)
	{
		const glm::vec3 delta{ previous_mouse_position[0] - current_mouse_position[0], current_mouse_position[1] - previous_mouse_position[1], 0 };
		camera_tracking_data.position += delta * camera_pan_speed * camera_tracking_data.position.z * 0.5f;
		camera_tracking_data.target += delta * camera_pan_speed * camera_tracking_data.position.z * 0.5f;

	}
}

void TrackerSystemInputHandler::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		::renderer::window::kill();
}

void TrackerSystemInputHandler::window_size_callback(GLFWwindow* window, int width, int height)
{
	::renderer::space::update_viewport_matrix(width, height);
	::renderer::space::update_projection_matrix(width, height);
}

void TrackerSystemInputHandler::window_close_callback(GLFWwindow* window)
{
	::renderer::window::kill();
}

void TrackerSystemInputHandler::Init(GLFWwindow* window)
{
	if (initialized) return;

	current_mouse_position[0] = 0;
	current_mouse_position[1] = 0;
	previous_mouse_position[0] = 0;
	previous_mouse_position[1] = 0;


	glfwSetCursorPosCallback(window, cursor_position_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetKeyCallback(window, key_callback);
	glfwSetWindowSizeCallback(window, window_size_callback);
	glfwSetWindowCloseCallback(window, window_close_callback);

	initialized = true;
}




