#pragma once

#include "glm\vec3.hpp"

struct GLFWwindow;


class TrackerSystemInputHandler
{
public:
	class CameraTracking
	{
	public:
		CameraTracking() : position{ 0,0,50 }, target{ 0,0,0 }, up{ 0,1,0 } {};
		glm::vec3 position;
		glm::vec3 target;
		glm::vec3 up;
	};

public:
	static void Init(GLFWwindow* window);

	/** Returns a copy of the most current OpenGL camera position data. */
	inline static CameraTracking GetCameraTracking() { return camera_tracking_data; };


	// callback events
	static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
	static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
	static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos);
	static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
	static void window_size_callback(GLFWwindow* window, int width, int height);
	static void window_close_callback(GLFWwindow* window);

private:
	TrackerSystemInputHandler();
	~TrackerSystemInputHandler() {};

	// Data
	static float camera_pan_speed;
	static float camera_zoom_speed;
	static CameraTracking camera_tracking_data;

private:
	// States
	static bool initialized;
	static bool move_state_active;
	static double current_mouse_position[2];
	static double previous_mouse_position[2];
};